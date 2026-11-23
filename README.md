# 割草机外壳重新设计（方案 A）与 3D 查看器

机器人割草机外壳的重新设计：保留原有底盘、前保险杠和车轮，重新设计车身。当前版本是 v17：一体式车身，加一块卡扣式上盖，盖住所有固定螺丝；盖板与车顶齐平，下面是 STOP 键和显示屏。

- 公开查看器：<https://mower-3d-viewer.vercel.app>（原版设计在 `/original/`）

## 目录

| 路径 | 内容 |
| --- | --- |
| `viewer/` | three.js 网页查看器（静态文件）。说明见 [viewer/README.md](viewer/README.md) |
| `viewer/models/` | 查看器用的 glb 模型、`design.json`、`hotspots.json`、配色表 |
| `viewer/original/` | 原版设计的查看器页面 |
| `scripts/pipeline/` | 参数化生成器和检查、渲染脚本（当前版本） |
| `scripts/pipeline/history/` | 生成器和渲染脚本的旧版本（v1–v16） |
| `scripts/` | 早期的装配、导出和渲染脚本 |

## 主要脚本

- `scripts/pipeline/design_gen.py`：读取设计参数（如 `a_cov.json`），生成车身、上盖、盖板网格，输出 `.npz`，并打印检查结果。需要同目录下的 `constraints.json`、`keep.npz`、`control.npz`、`chassis_top.npy`。
- `scripts/pipeline/check_cover.py`：检查上盖是否盖住所有螺丝孔。
- `scripts/pipeline/run_cov.sh`：一次完成 v17 的生成、检查和全套渲染。
- `scripts/export_design_web.py`：把生成结果导出为查看器用的 glb，例如：

  ```bash
  blender -b --factory-startup -P scripts/export_design_web.py -- <设计目录> <设计>.json <设计>.npz viewer/models
  ```

- `scripts/pipeline/make_artifact_chunks.py`：生成 claude.ai 页面用的 base64 分块模型。

运行环境：Blender 4.2.3（默认路径 `~/blender-4.2.3-linux-x64/`），Python 虚拟环境放在 `scripts/venv/`（未纳入仓库）。

## 本地预览和部署

```bash
python3 viewer/serve.py
```

```bash
cd viewer && vercel deploy --prod
```

`viewer/.vercelignore` 决定哪些文件不上线（本地启动脚本、claude.ai 版页面、README 等）。

## 没有放进仓库的文件

这些文件超过 GitHub 单个文件 100 MB 的上限，或者是可再生成的输出：

- `mower_assembly.blend`、`mower_shell.blend`（各约 1.4 GB / 124 MB）：原始装配和外壳模型。
- `renders/`、`renders.zip`：各版本渲染图（约 1.1 GB）。
- `work/`：从装配模型拆出来的中间网格（底盘、车轮、充电座等，共约 9 GB）。`design_render.py` 渲染时需要其中的 `asm_dock.npz`。
- `scripts/pipeline/out_*/`：渲染输出。
