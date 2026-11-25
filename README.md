# 割草机外壳重新设计（方案 A）与 3D 查看器

机器人割草机外壳的重新设计：保留原有底盘、前保险杠和车轮，重新设计车身。当前版本是 v18“Orbit”（f5）：

- 浅陶瓷灰的卡扣式上盖罩在深石墨色的一体式车身上，2.5 mm 实体壁厚；
- 车尾在平面和侧面都做成大圆角，车顶以更柔和的曲线落到车尾（f5）；
- 全车只有一条分缝，带防水迷宫；
- 前后是同样的烟色镜面灯带，用密封灯组；
- 车头是对称的充电口；
- 盖板与车顶齐平，用隐藏的齿轮弧轨铰链，自动开启、双向阻尼；
- 盖板上是符合 EN 50636-2-107 的红色凸起 STOP 键；
- 底盘只切两处旧挡泥板件。

另有一个独立方案 v18“Horizon”：

- 缎面银上盖罩在深色车身上；
- 离地 180 mm 的一条水平分缝，下方有一条细灯线；
- 一条烟色中脊饰带，盖板是饰带的一段；
- 后看是梯形的姿态。

它沿用 Orbit 第 4 轮的全部工程（STOP 键、铰链、卡扣、防水迷宫、底盘切割）。灯线后面的导光件、摄像头模组和充电口内部结构还没做。查看器里在 `/horizon/` 页面。

v17 的模型和生成器还保留在仓库里。

- 产品介绍页：<https://mower-3d-viewer.vercel.app>（以割草性能为主：割草、路径、避障、地形、回充、安全；标 * 的性能数值是设计目标）
- 3D 查看器：<https://mower-3d-viewer.vercel.app/viewer.html>（Horizon 在 `/horizon/`，原版设计在 `/original/`）

## 新设计的完整模型

[new_design/v18_orbit/](new_design/v18_orbit/) 是 v18 Orbit（f5）的全分辨率装配模型：

- `mower_new_v18_orbit.blend`（233 MB）：按零件分成 6 个集合：
  - 车身（含下裙、台面、螺丝、灯组、线束）；
  - 卡扣式上盖（含盖板槽和铰链座）；
  - 盖板（含 STOP 键）；
  - 前保险杠；
  - 控制面板；
  - 原有的底盘、驱动轮和万向轮（底盘按规格切掉两处）。
- `mower_new_v18_orbit.glb`（17 MB）：同一套模型，单个 glTF 文件，用 Draco 压缩（Blender 和大多数 glTF 软件都能直接打开）。
- `preview.png`：预览图。

重新导出（在 `scripts/pipeline_v18/` 里，先按该目录 `CHANGES.txt` 第 4、7、8 节生成 `f5.npz`）：

```bash
FULL=1 NAME=mower_new_v18_orbit BASE=../../viewer/models/base.glb blender -b --factory-startup -P export_web.py -- f5.json f5.npz ../../new_design/v18_orbit
```

[new_design/v18_horizon/](new_design/v18_horizon/) 是 v18 Horizon 的全分辨率装配模型：

- `mower_new_v18_horizon.blend`：300 MB；
- `mower_new_v18_horizon.glb`：20 MB，用 Draco 压缩；
- `preview.png`。

生成器在 `scripts/pipeline_v18_horizon/`：参数 `h.json` 由 `mkh.sh` 合成，说明见 `CHANGES_horizon.txt`，检查结果在 `checks/h/`。

[new_design/v17_top_cover/](new_design/v17_top_cover/) 是 v17 的全分辨率装配模型，可以直接用 Blender 或任何支持 glTF 的软件打开：

- `mower_new_v17.blend`（89 MB）：按零件分成 6 个集合，分别是车身（含下裙、台面、14 颗螺丝）、卡扣式上盖、盖板（含铰链）、前保险杠、控制面板（STOP 键、显示屏），以及原有的底盘、驱动轮和万向轮。
- `mower_new_v17.glb`（79 MB）：同一套模型，单个 glTF 文件。
- `preview.png`：预览图。

新设计的零件没有减面，约 280 万个三角面。底盘和车轮取自网页查看器用的 `viewer/models/base.glb`，是减面后的版本；它们的原始全分辨率模型在 `mower_assembly.blend` 里。单位是米，Z 轴向上，+X 指向车头。

重新导出（例如换成 v16 的 `a_one`）：

```bash
FULL=1 NAME=mower_new_v17 BASE=viewer/models/base.glb blender -b --factory-startup -P scripts/export_design_web.py -- scripts/pipeline scripts/pipeline/a_cov.json scripts/pipeline/a_cov.npz new_design/v17_top_cover
```

## 目录

| 路径 | 内容 |
| --- | --- |
| `scripts/pipeline_v18_horizon/` | v18 Horizon 生成器（同一套代码，多了 Horizon 的参数和功能），参数 `h.json`，说明见 `CHANGES_horizon.txt` |
| `viewer/horizon/` | v18 Horizon 的查看器页面（共用 `app.js`，模型和配色单独） |
| `scripts/pipeline_v18/` | v18“Orbit”生成器、全部检查和渲染脚本。最终参数是 `f5.json`（由 `mk_f3.py` 从 `f2.json` 和 `fix3_*.json`、`fix4_*.json`、`fix5_tail.json` 合成）。说明见 `CHANGES.txt`：第 6 节是第 3 轮修复，第 7 节是第 4 轮修复，第 8 节是圆车尾（f5）。检查结果在 `checks/` |
| `renders/redesign/v18_orbit/`、`v18_orbit_f4/` | v18 f2 和 f4 的渲染图（不在仓库里，`renders/` 被忽略） |
| `new_design/` | 新设计的完整模型（.blend / .glb）：v18 Orbit、v18 Horizon 和 v17 |
| `viewer/` | 网站（静态文件）：`index.html` 是产品介绍页（样式、脚本和图片在 `viewer/site/`），`viewer.html` 是 three.js 3D 查看器。说明见 [viewer/README.md](viewer/README.md) |
| `scripts/site/` | 产品介绍页的图片：`render_site.sh` 渲染 f5（含 3D 分解序列），`bake_site.py` 把渲染图烘焙到页面底色并转成 WebP（车身边缘保持原始合成，不出白边）；`*_shoot.mjs` 用无头 Chrome 截图检查页面 |
| `viewer/models/` | 查看器用的 glb 模型、`design.json`、`hotspots.json`、配色表 |
| `viewer/original/` | 原版设计的查看器页面 |
| `scripts/pipeline/` | 参数化生成器和检查、渲染脚本（当前版本） |
| `scripts/pipeline/history/` | 生成器和渲染脚本的旧版本（v1–v16） |
| `scripts/` | 早期的装配、导出和渲染脚本 |

## 主要脚本

- `scripts/pipeline/design_gen.py`：读取设计参数（如 `a_cov.json`），生成车身、上盖、盖板网格，输出 `.npz`，并打印检查结果。需要同目录下的 `constraints.json`、`keep.npz`、`control.npz`、`chassis_top.npy`。
- `scripts/pipeline/check_cover.py`：检查上盖是否盖住所有螺丝孔。
- `scripts/pipeline/run_cov.sh`：一次完成 v17 的生成、检查和全套渲染。
- `scripts/pipeline_v18/export_web.py`：把 v18 生成结果导出为查看器用的 glb、`design.json`、`hotspots.json`、`chassis_cut.json`，再用 gltfpack（meshoptimizer）压缩：

  ```bash
  blender -b --factory-startup -P export_web.py -- f5_web.json f5_web.npz <输出目录>
  ```

  ```bash
  npx gltfpack -i body.glb -o ../viewer/models/body.glb -cc -kn -km -ke -vp 16 -vn 12
  ```

- `scripts/export_design_web.py`：v17 的导出脚本，例如：

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

`viewer/.vercelignore` 决定哪些文件不上线（本地启动脚本、claude.ai 版页面、README 等）。网站首页是产品介绍页，3D 查看器在 `viewer.html`。

## 用 Git LFS 存放的大文件

下面这些文件用 Git LFS 存放（规则见 `.gitattributes`）。克隆前先装好 Git LFS，`git clone` 时会自动下载；只要普通文件的话，可以设 `GIT_LFS_SKIP_SMUDGE=1` 再克隆。

| 文件 | 大小 | 内容 |
| --- | --- | --- |
| `mower_assembly.blend` | 1.4 GB | 原始装配模型 |
| `mower_shell.blend` | 124 MB | 原始外壳模型 |
| `source/dock.stl` | 208 MB | 充电座原始网格 |
| `scripts/work/asm_dock.npz` | 105 MB | 充电座网格，`design_render.py` 渲染时要用 |
| `scripts/pipeline/a_cov.npz` | 86 MB | v17（一体式车身 + 卡扣上盖）全分辨率设计网格 |
| `scripts/pipeline/a_one.npz` | 85 MB | v16（一体式车身）全分辨率设计网格 |
| `scripts/pipeline/a_pro.npz` | 69 MB | v15 全分辨率设计网格 |
| `new_design/v18_orbit/mower_new_v18_orbit.blend` | 233 MB | v18 Orbit（f5）全分辨率装配模型 |
| `new_design/v18_horizon/mower_new_v18_horizon.blend` | 300 MB | v18 Horizon 全分辨率装配模型 |

网页查看器里的 glb 是减面或压缩后的版本，不走 LFS。`scripts/pipeline_v18/` 里的输入网格（`keep.npz`、`control.npz`、`dev/base_parts.npz`、`base.glb`）也用 LFS 存放。

## 没有放进仓库的文件

- `*.blend1`：Blender 自动备份。
- `renders/`、`renders.zip`：各版本渲染图（约 1.1 GB）。
- 其余中间网格（从装配模型拆出的底盘、车轮等，以及各轮试验版本，约 15 GB）。
- `scripts/pipeline/out_*/`：渲染输出。
