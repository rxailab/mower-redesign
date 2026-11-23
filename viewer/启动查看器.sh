#!/bin/sh
# 双击或在终端运行：启动本地服务器并在浏览器中打开 3D 查看器
cd "$(dirname "$0")" || exit 1
exec python3 serve.py "$@"
