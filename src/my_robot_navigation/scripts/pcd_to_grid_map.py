#!/usr/bin/env python3
"""将离线 PCD 点云投影成 Nav2 可加载的 2D 栅格地图。

输入一般来自 PointLIO 保存的 PCD 地图。脚本读取 x/y/z 字段，按高度范围
筛选障碍点，再输出 map.pgm 和 map.yaml。默认把没有点的位置视为空闲；
如果希望未观测区域保持 unknown，可以使用 --unknown。
"""

import argparse
import math
import os
import struct
import sys
from collections import defaultdict

import yaml


PCD_TYPE_TO_STRUCT = {
    ('F', 4): 'f',
    ('F', 8): 'd',
    ('I', 1): 'b',
    ('I', 2): 'h',
    ('I', 4): 'i',
    ('I', 8): 'q',
    ('U', 1): 'B',
    ('U', 2): 'H',
    ('U', 4): 'I',
    ('U', 8): 'Q',
}


def parse_args():
    parser = argparse.ArgumentParser(
        description='Project a PCD point cloud into a Nav2 map.pgm/map.yaml pair.'
    )
    parser.add_argument('pcd', help='Input PCD file. ASCII and binary PCD are supported.')
    parser.add_argument(
        '--output-prefix',
        required=True,
        help='Output prefix, for example src/my_robot_navigation/maps/real_site_map',
    )
    parser.add_argument('--resolution', type=float, default=0.05, help='Grid resolution in meters.')
    parser.add_argument('--min-z', type=float, default=0.05, help='Ignore points below this z.')
    parser.add_argument('--max-z', type=float, default=1.50, help='Ignore points above this z.')
    parser.add_argument('--origin-x', type=float, help='Map origin x. Auto-computed if omitted.')
    parser.add_argument('--origin-y', type=float, help='Map origin y. Auto-computed if omitted.')
    parser.add_argument('--width-m', type=float, help='Map width in meters. Auto-computed if omitted.')
    parser.add_argument('--height-m', type=float, help='Map height in meters. Auto-computed if omitted.')
    parser.add_argument('--padding', type=float, default=1.0, help='Auto bounds padding in meters.')
    parser.add_argument('--inflate-radius', type=float, default=0.15, help='Obstacle inflation in meters.')
    parser.add_argument(
        '--min-points-per-cell',
        type=int,
        default=1,
        help='A cell becomes occupied only after at least this many points fall into it.',
    )
    parser.add_argument(
        '--unknown',
        action='store_true',
        help='Initialize non-obstacle cells as unknown instead of free.',
    )
    parser.add_argument('--occupied-thresh', type=float, default=0.65)
    parser.add_argument('--free-thresh', type=float, default=0.25)
    return parser.parse_args()


def read_pcd_header(path):
    header_lines = []
    with open(path, 'rb') as stream:
        while True:
            line = stream.readline()
            if not line:
                raise ValueError('PCD header ended before DATA line.')
            decoded = line.decode('utf-8', errors='replace').strip()
            header_lines.append(decoded)
            if decoded.startswith('DATA'):
                data_offset = stream.tell()
                break

    header = {}
    for line in header_lines:
        if not line or line.startswith('#'):
            continue
        parts = line.split()
        key = parts[0].upper()
        header[key] = parts[1:]

    if 'FIELDS' not in header:
        raise ValueError('PCD header has no FIELDS entry.')
    if 'DATA' not in header:
        raise ValueError('PCD header has no DATA entry.')

    fields = header['FIELDS']
    sizes = [int(value) for value in header.get('SIZE', ['4'] * len(fields))]
    types = header.get('TYPE', ['F'] * len(fields))
    counts = [int(value) for value in header.get('COUNT', ['1'] * len(fields))]
    points = int(header.get('POINTS', header.get('WIDTH', ['0']))[0])
    data_type = header['DATA'][0].lower()
    return {
        'fields': fields,
        'sizes': sizes,
        'types': types,
        'counts': counts,
        'points': points,
        'data_type': data_type,
        'data_offset': data_offset,
    }


def field_layout(meta):
    offsets = {}
    offset = 0
    for name, size, count, typ in zip(
        meta['fields'], meta['sizes'], meta['counts'], meta['types']
    ):
        offsets[name] = (offset, size, count, typ)
        offset += size * count
    return offsets, offset


def read_ascii_points(path, meta):
    indices = {name: index for index, name in enumerate(meta['fields'])}
    for name in ('x', 'y', 'z'):
        if name not in indices:
            raise ValueError(f'PCD file has no {name} field.')

    with open(path, 'rb') as stream:
        stream.seek(meta['data_offset'])
        for raw_line in stream:
            if not raw_line.strip():
                continue
            parts = raw_line.decode('utf-8', errors='replace').split()
            try:
                yield (
                    float(parts[indices['x']]),
                    float(parts[indices['y']]),
                    float(parts[indices['z']]),
                )
            except (IndexError, ValueError):
                continue


def unpack_binary_value(record, offset, size, typ):
    fmt = PCD_TYPE_TO_STRUCT.get((typ, size))
    if fmt is None:
        raise ValueError(f'Unsupported PCD field type: TYPE={typ}, SIZE={size}')
    return struct.unpack_from('<' + fmt, record, offset)[0]


def read_binary_points(path, meta):
    offsets, point_step = field_layout(meta)
    for name in ('x', 'y', 'z'):
        if name not in offsets:
            raise ValueError(f'PCD file has no {name} field.')

    with open(path, 'rb') as stream:
        stream.seek(meta['data_offset'])
        for _ in range(meta['points']):
            record = stream.read(point_step)
            if len(record) < point_step:
                break
            values = []
            for name in ('x', 'y', 'z'):
                offset, size, _count, typ = offsets[name]
                values.append(float(unpack_binary_value(record, offset, size, typ)))
            yield tuple(values)


def read_points(path):
    meta = read_pcd_header(path)
    if meta['data_type'] == 'ascii':
        return list(read_ascii_points(path, meta))
    if meta['data_type'] == 'binary':
        return list(read_binary_points(path, meta))
    if meta['data_type'] == 'binary_compressed':
        raise ValueError('binary_compressed PCD is not supported yet. Save as ascii or binary first.')
    raise ValueError(f'Unsupported PCD DATA type: {meta["data_type"]}')


def finite_filtered_points(points, min_z, max_z):
    filtered = []
    for x, y, z in points:
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
            continue
        if min_z <= z <= max_z:
            filtered.append((x, y))
    return filtered


def compute_bounds(points, args):
    if not points:
        raise ValueError('No points remained after z filtering.')

    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    min_x = min(xs) - args.padding
    max_x = max(xs) + args.padding
    min_y = min(ys) - args.padding
    max_y = max(ys) + args.padding

    origin_x = args.origin_x if args.origin_x is not None else min_x
    origin_y = args.origin_y if args.origin_y is not None else min_y
    width_m = args.width_m if args.width_m is not None else max_x - origin_x
    height_m = args.height_m if args.height_m is not None else max_y - origin_y
    width_cells = max(1, int(math.ceil(width_m / args.resolution)))
    height_cells = max(1, int(math.ceil(height_m / args.resolution)))
    return origin_x, origin_y, width_cells, height_cells


def mark_inflated(data, width, height, mx, my, inflate_cells, occupied_value):
    for dy in range(-inflate_cells, inflate_cells + 1):
        for dx in range(-inflate_cells, inflate_cells + 1):
            if dx * dx + dy * dy > inflate_cells * inflate_cells:
                continue
            x = mx + dx
            y = my + dy
            if 0 <= x < width and 0 <= y < height:
                data[y * width + x] = occupied_value


def write_pgm(path, data, width, height):
    with open(path, 'wb') as stream:
        stream.write(f'P5\n{width} {height}\n255\n'.encode('ascii'))
        for y in reversed(range(height)):
            start = y * width
            stream.write(bytes(data[start:start + width]))


def write_yaml(path, image_name, resolution, origin_x, origin_y, occupied_thresh, free_thresh):
    content = {
        'image': image_name,
        'mode': 'trinary',
        'resolution': float(resolution),
        'origin': [float(origin_x), float(origin_y), 0.0],
        'negate': 0,
        'occupied_thresh': float(occupied_thresh),
        'free_thresh': float(free_thresh),
    }
    with open(path, 'w', encoding='utf-8') as stream:
        yaml.safe_dump(content, stream, sort_keys=False)


def main():
    args = parse_args()
    if args.resolution <= 0.0:
        raise SystemExit('--resolution must be positive.')

    points = finite_filtered_points(read_points(args.pcd), args.min_z, args.max_z)
    origin_x, origin_y, width, height = compute_bounds(points, args)
    default_value = 205 if args.unknown else 254
    occupied_value = 0
    data = [default_value] * (width * height)
    cell_counts = defaultdict(int)

    for x, y in points:
        mx = int(math.floor((x - origin_x) / args.resolution))
        my = int(math.floor((y - origin_y) / args.resolution))
        if 0 <= mx < width and 0 <= my < height:
            cell_counts[(mx, my)] += 1

    inflate_cells = max(0, int(math.ceil(args.inflate_radius / args.resolution)))
    occupied_cells = 0
    for (mx, my), count in cell_counts.items():
        if count >= args.min_points_per_cell:
            mark_inflated(data, width, height, mx, my, inflate_cells, occupied_value)
            occupied_cells += 1

    output_prefix = os.path.abspath(args.output_prefix)
    os.makedirs(os.path.dirname(output_prefix), exist_ok=True)
    pgm_path = output_prefix + '.pgm'
    yaml_path = output_prefix + '.yaml'
    write_pgm(pgm_path, data, width, height)
    write_yaml(
        yaml_path,
        os.path.basename(pgm_path),
        args.resolution,
        origin_x,
        origin_y,
        args.occupied_thresh,
        args.free_thresh,
    )

    print(f'Loaded filtered points: {len(points)}')
    print(f'Grid: {width} x {height} cells, resolution={args.resolution:.3f} m')
    print(f'Origin: [{origin_x:.3f}, {origin_y:.3f}, 0.0]')
    print(f'Occupied source cells before inflation: {occupied_cells}')
    print(f'Wrote: {pgm_path}')
    print(f'Wrote: {yaml_path}')


if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        print(f'pcd_to_grid_map.py: error: {exc}', file=sys.stderr)
        raise SystemExit(1)
