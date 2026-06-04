# Git 提交与协作说明

本文档用于后续协作时统一代码提交、同步和推送方式。

## 1. 仓库地址

当前远程仓库：

```bash
git@github.com:t1415926/robo_sys.git
```

网页地址：

```text
https://github.com/t1415926/robo_sys
```

## 2. 首次克隆仓库

推荐使用 SSH：

```bash
git clone git@github.com:t1415926/robo_sys.git
cd robo_sys
```

如果使用 HTTPS：

```bash
git clone https://github.com/t1415926/robo_sys.git
cd robo_sys
```

## 3. 每次开始开发前

先同步远程最新代码：

```bash
git checkout main
git pull --rebase origin main
```

查看当前状态：

```bash
git status -sb
```

## 4. 修改代码后检查变更

查看改了哪些文件：

```bash
git status -sb
```

查看具体差异：

```bash
git diff
```

只看已暂存差异：

```bash
git diff --cached
```

## 5. 提交代码

暂存指定文件，推荐显式写路径：

```bash
git add README.md
git add src/my_robot_navigation/config/nav2_params.yaml
```

如果确认全部变更都属于本次提交，可以使用：

```bash
git add .
```

提交：

```bash
git commit -m "说明本次修改内容"
```

提交信息建议简短清楚，例如：

```text
add simulation bringup script
update nav2 params
fix sim localization transform
add git workflow docs
```

## 6. 推送到 GitHub

推送当前 `main` 分支：

```bash
git push origin main
```

如果是第一次推送当前分支：

```bash
git push -u origin main
```

## 7. 推荐的日常完整流程

```bash
cd /home/dtc/robo_sys
git checkout main
git pull --rebase origin main

# 修改代码或文档

git status -sb
git diff
git add <需要提交的文件>
git commit -m "本次修改说明"
git push origin main
```

## 8. 多人协作建议

如果多人同时开发，建议不要直接在 `main` 上改大功能，而是创建功能分支：

```bash
git checkout main
git pull --rebase origin main
git checkout -b feature/sim-map
```

开发完成后：

```bash
git status -sb
git add <需要提交的文件>
git commit -m "add simple simulation map"
git push -u origin feature/sim-map
```

然后在 GitHub 上创建 Pull Request，确认无问题后再合并到 `main`。

## 9. 不要提交的文件

以下文件或目录是构建产物、缓存或运行日志，不应提交：

```text
build/
install/
log/
__pycache__/
*.pyc
*.bag
*.db3
*.mcap
frames.pdf
```

这些规则已经写入 `.gitignore`。

## 10. 提交前建议检查

ROS2 包修改后建议至少执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```

如果 Anaconda 抢占 Python，使用：

```bash
env -u PYTHONPATH -u PYTHONHOME \
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  bash -c 'source /opt/ros/humble/setup.bash && colcon build --symlink-install'
```

检查脚本语法：

```bash
bash -n start_sim_nav.sh
```

## 11. 常见问题

### 11.1 push 提示无法读取 GitHub 用户名

HTTPS 方式需要 GitHub Token。推荐改用 SSH：

```bash
git remote set-url origin git@github.com:t1415926/robo_sys.git
ssh -T git@github.com
git push origin main
```

### 11.2 本地代码落后于远程

先同步远程：

```bash
git pull --rebase origin main
```

如果出现冲突，解决冲突后：

```bash
git add <冲突已解决的文件>
git rebase --continue
git push origin main
```

### 11.3 不小心暂存了不该提交的文件

取消暂存：

```bash
git restore --staged <文件路径>
```

只撤销工作区某个文件的修改：

```bash
git restore <文件路径>
```

执行 `git restore` 前要确认这个修改确实不需要保留。
