#!/usr/bin/env python3
import argparse
import os
import sys
from dataclasses import dataclass

import cv2
import numpy as np
import yaml


UNKNOWN = 0
FORBID = 1
ALLOW = 2

MODE_NAMES = {
    FORBID: 'FORBID(red)',
    ALLOW: 'ALLOW(green)',
    UNKNOWN: 'ERASE',
}


@dataclass
class MapMeta:
    resolution: float
    origin: list
    occupied_thresh: float
    free_thresh: float
    negate: int
    mode: str


class SemanticMaskEditor:
    def __init__(self, args):
        self.args = args
        self.base = self.load_base_image(args.image)
        self.height, self.width = self.base.shape[:2]
        self.mask = self.load_or_create_mask(args.mask)
        self.history = []
        self.mode = FORBID
        self.brush = max(1, int(args.brush_size))
        self.alpha = min(1.0, max(0.0, float(args.alpha)))
        self.display_scale = max(0.05, float(args.scale))
        self.drawing = False
        self.last_xy = None
        self.dirty = False
        self.window_name = 'Semantic Mask Editor'

    @staticmethod
    def load_base_image(path):
        image = cv2.imread(path, cv2.IMREAD_UNCHANGED)
        if image is None:
            raise RuntimeError(f'Cannot read image: {path}')
        if image.ndim == 2:
            return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        if image.shape[2] == 4:
            return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
        return image

    def load_or_create_mask(self, path):
        if not path:
            return np.zeros((self.height, self.width), dtype=np.uint8)

        raw = cv2.imread(path, cv2.IMREAD_UNCHANGED)
        if raw is None:
            raise RuntimeError(f'Cannot read mask: {path}')
        if raw.shape[:2] != (self.height, self.width):
            raise RuntimeError(
                f'Mask size {raw.shape[1]}x{raw.shape[0]} does not match '
                f'image size {self.width}x{self.height}: {path}'
            )

        if raw.ndim == 2:
            mask = np.zeros_like(raw, dtype=np.uint8)
            mask[raw == FORBID] = FORBID
            mask[raw == ALLOW] = ALLOW
            mask[raw >= 128] = ALLOW
            return mask

        bgr = raw[:, :, :3]
        red = (bgr[:, :, 2] > 140) & (bgr[:, :, 1] < 120)
        green = (bgr[:, :, 1] > 140) & (bgr[:, :, 2] < 120)
        mask = np.zeros((self.height, self.width), dtype=np.uint8)
        mask[red] = FORBID
        mask[green] = ALLOW
        return mask

    def push_history(self):
        self.history.append(self.mask.copy())
        if len(self.history) > self.args.undo_steps:
            self.history.pop(0)

    def undo(self):
        if not self.history:
            return
        self.mask = self.history.pop()
        self.dirty = True

    def screen_to_image(self, x, y):
        ix = int(round(x / self.display_scale))
        iy = int(round(y / self.display_scale))
        ix = min(self.width - 1, max(0, ix))
        iy = min(self.height - 1, max(0, iy))
        return ix, iy

    def paint_at(self, x, y):
        value = 0 if self.mode == UNKNOWN else self.mode
        cv2.circle(self.mask, (x, y), self.brush, int(value), thickness=-1, lineType=cv2.LINE_8)
        self.dirty = True

    def paint_line(self, p0, p1):
        value = 0 if self.mode == UNKNOWN else self.mode
        cv2.line(self.mask, p0, p1, int(value), thickness=self.brush * 2, lineType=cv2.LINE_8)
        cv2.circle(self.mask, p1, self.brush, int(value), thickness=-1, lineType=cv2.LINE_8)
        self.dirty = True

    def mouse_callback(self, event, x, y, flags, _userdata):
        ix, iy = self.screen_to_image(x, y)

        if event == cv2.EVENT_LBUTTONDOWN:
            self.push_history()
            self.drawing = True
            self.last_xy = (ix, iy)
            self.paint_at(ix, iy)
        elif event == cv2.EVENT_RBUTTONDOWN:
            self.push_history()
            self.drawing = True
            self.last_xy = (ix, iy)
            old_mode = self.mode
            self.mode = UNKNOWN
            self.paint_at(ix, iy)
            self.mode = old_mode
        elif event == cv2.EVENT_MOUSEMOVE and self.drawing:
            current = (ix, iy)
            if flags & cv2.EVENT_FLAG_RBUTTON:
                old_mode = self.mode
                self.mode = UNKNOWN
                self.paint_line(self.last_xy, current)
                self.mode = old_mode
            else:
                self.paint_line(self.last_xy, current)
            self.last_xy = current
        elif event in (cv2.EVENT_LBUTTONUP, cv2.EVENT_RBUTTONUP):
            self.drawing = False
            self.last_xy = None
        elif event == cv2.EVENT_MOUSEWHEEL:
            if flags > 0:
                self.brush = min(self.args.max_brush_size, self.brush + 2)
            else:
                self.brush = max(1, self.brush - 2)

    def make_overlay(self):
        overlay = self.base.copy()
        colors = np.zeros_like(self.base)
        colors[self.mask == FORBID] = (0, 0, 255)
        colors[self.mask == ALLOW] = (0, 210, 0)
        active = self.mask > 0
        overlay[active] = cv2.addWeighted(
            self.base[active],
            1.0 - self.alpha,
            colors[active],
            self.alpha,
            0,
        )
        return overlay

    def render(self):
        view = self.make_overlay()
        status = (
            f'{MODE_NAMES[self.mode]} | brush={self.brush}px | alpha={self.alpha:.2f} | '
            f'red={int(np.count_nonzero(self.mask == FORBID))}px | '
            f'green={int(np.count_nonzero(self.mask == ALLOW))}px'
        )
        cv2.rectangle(view, (0, 0), (self.width, 32), (25, 25, 25), -1)
        cv2.putText(
            view,
            status,
            (8, 22),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )
        if self.display_scale != 1.0:
            view = cv2.resize(
                view,
                None,
                fx=self.display_scale,
                fy=self.display_scale,
                interpolation=cv2.INTER_NEAREST,
            )
        return view

    def print_help(self):
        print(
            '''
Semantic Mask Editor controls:
  Left drag        paint current mode
  Right drag       erase
  Mouse wheel      change brush size
  r / 1            red forbidden area
  g / 2            green allowed area
  e / 0            erase mode
  [ / ]            smaller / larger brush
  - / =            lower / higher overlay alpha
  u                undo
  c                clear whole mask
  s                save/export
  h                print this help
  q / Esc          quit
'''
        )

    def run(self):
        self.print_help()
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)

        while True:
            cv2.imshow(self.window_name, self.render())
            key = cv2.waitKey(20) & 0xFF
            if key == 255:
                continue
            if key in (27, ord('q')):
                break
            if key in (ord('r'), ord('1')):
                self.mode = FORBID
            elif key in (ord('g'), ord('2')):
                self.mode = ALLOW
            elif key in (ord('e'), ord('0')):
                self.mode = UNKNOWN
            elif key == ord('['):
                self.brush = max(1, self.brush - 2)
            elif key == ord(']'):
                self.brush = min(self.args.max_brush_size, self.brush + 2)
            elif key in (ord('-'), ord('_')):
                self.alpha = max(0.05, self.alpha - 0.05)
            elif key in (ord('='), ord('+')):
                self.alpha = min(0.95, self.alpha + 0.05)
            elif key == ord('u'):
                self.undo()
            elif key == ord('c'):
                self.push_history()
                self.mask.fill(UNKNOWN)
                self.dirty = True
            elif key == ord('s'):
                self.save_outputs()
            elif key == ord('h'):
                self.print_help()

        if self.dirty and not self.args.no_prompt:
            answer = input('Mask has unsaved changes. Save now? [Y/n] ').strip().lower()
            if answer in ('', 'y', 'yes'):
                self.save_outputs()
        cv2.destroyAllWindows()

    def save_outputs(self):
        prefix = self.args.output_prefix
        os.makedirs(os.path.dirname(os.path.abspath(prefix)), exist_ok=True)

        overlay_path = f'{prefix}_overlay.png'
        index_path = f'{prefix}_mask_index.png'
        color_path = f'{prefix}_mask_color.png'
        nav2_pgm_path = f'{prefix}_nav2.pgm'
        nav2_yaml_path = f'{prefix}_nav2.yaml'

        color = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        color[self.mask == FORBID] = (0, 0, 255)
        color[self.mask == ALLOW] = (0, 255, 0)

        nav2 = np.full((self.height, self.width), 205, dtype=np.uint8)
        nav2[self.mask == FORBID] = 0
        nav2[self.mask == ALLOW] = 254
        if self.args.unlabeled_as_free:
            nav2[self.mask == UNKNOWN] = 254
        elif self.args.unlabeled_as_occupied:
            nav2[self.mask == UNKNOWN] = 0

        cv2.imwrite(overlay_path, self.make_overlay())
        cv2.imwrite(index_path, self.mask)
        cv2.imwrite(color_path, color)
        cv2.imwrite(nav2_pgm_path, nav2)
        self.write_map_yaml(nav2_yaml_path, os.path.basename(nav2_pgm_path))

        print('Saved:')
        print(f'  {overlay_path}')
        print(f'  {index_path}')
        print(f'  {color_path}')
        print(f'  {nav2_pgm_path}')
        print(f'  {nav2_yaml_path}')
        self.dirty = False

    def load_map_meta(self):
        if self.args.map_yaml:
            with open(self.args.map_yaml, 'r', encoding='utf-8') as f:
                data = yaml.safe_load(f) or {}
        else:
            data = {}

        origin = self.args.origin
        if origin is not None:
            origin_value = [float(v) for v in origin.split(',')]
            if len(origin_value) != 3:
                raise RuntimeError('--origin must be formatted as x,y,yaw')
        else:
            origin_value = data.get('origin', [0.0, 0.0, 0.0])

        return MapMeta(
            resolution=float(self.args.resolution or data.get('resolution', 0.05)),
            origin=origin_value,
            occupied_thresh=float(data.get('occupied_thresh', 0.65)),
            free_thresh=float(data.get('free_thresh', 0.25)),
            negate=int(data.get('negate', 0)),
            mode=str(data.get('mode', 'trinary')),
        )

    def write_map_yaml(self, yaml_path, image_name):
        meta = self.load_map_meta()
        data = {
            'image': image_name,
            'mode': meta.mode,
            'resolution': meta.resolution,
            'origin': meta.origin,
            'negate': meta.negate,
            'occupied_thresh': meta.occupied_thresh,
            'free_thresh': meta.free_thresh,
        }
        with open(yaml_path, 'w', encoding='utf-8') as f:
            yaml.safe_dump(data, f, sort_keys=False, allow_unicode=True)


def default_output_prefix(image_path):
    root, _ext = os.path.splitext(image_path)
    return f'{root}_semantic'


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description='Draw red forbidden and green allowed masks over a 2D projection image.'
    )
    parser.add_argument('--image', required=True, help='2D projection image, PGM/PNG/JPG are supported.')
    parser.add_argument('--mask', help='Optional existing mask image to continue editing.')
    parser.add_argument('--output-prefix', help='Output path prefix.')
    parser.add_argument('--map-yaml', help='Optional existing Nav2 map YAML to copy resolution/origin metadata.')
    parser.add_argument('--resolution', type=float, help='Map resolution for exported *_nav2.yaml.')
    parser.add_argument('--origin', help='Map origin for exported YAML, formatted as x,y,yaw.')
    parser.add_argument('--brush-size', type=int, default=12, help='Initial brush radius in pixels.')
    parser.add_argument('--max-brush-size', type=int, default=200, help='Maximum brush radius in pixels.')
    parser.add_argument('--alpha', type=float, default=0.45, help='Overlay alpha, 0.0 to 1.0.')
    parser.add_argument('--scale', type=float, default=1.0, help='Display scale for large/small images.')
    parser.add_argument('--undo-steps', type=int, default=30, help='Number of undo snapshots kept in memory.')
    parser.add_argument('--unlabeled-as-free', action='store_true', help='Export unlabeled pixels as free in *_nav2.pgm.')
    parser.add_argument('--unlabeled-as-occupied', action='store_true', help='Export unlabeled pixels as occupied in *_nav2.pgm.')
    parser.add_argument('--no-prompt', action='store_true', help='Do not ask to save unsaved changes on exit.')
    args = parser.parse_args(argv)

    if args.unlabeled_as_free and args.unlabeled_as_occupied:
        parser.error('--unlabeled-as-free and --unlabeled-as-occupied cannot be used together.')
    if not args.output_prefix:
        args.output_prefix = default_output_prefix(args.image)
    return args


def main(argv=None):
    args = parse_args(argv or sys.argv[1:])
    editor = SemanticMaskEditor(args)
    editor.run()


if __name__ == '__main__':
    main()
