# DJI Flight Record Summarizer

扫描大疆无人机飞行数据文件夹，从影像 EXIF/XMP 和 KMZ 航线文件中提取元数据，每个飞行文件夹生成一行汇总记录，输出为 CSV。

## 功能

- 自动识别文件夹中的 DJI 照片（可见光、热红外、多光谱）
- 从 EXIF/XMP 提取 GPS、拍摄时间、相机型号、焦距等参数
- 从 KMZ 航线文件读取无人机型号、航线类型、重叠率、测区多边形
- 无 KMZ 时，基于 GPS 航带几何计算航向/旁向重叠率（自适应航带分割）
- 支持 `camera_params.yaml` 配置相机传感器参数，用于 GSD 和重叠率回退计算
- 多进程并行处理，支持进度条

## 快速开始

```bash
# 基本用法
uv run python summarize_uav_flight_info.py --data-root "F:/Photo data/flight_folders"

# 指定输出路径
uv run python summarize_uav_flight_info.py --data-root "F:/Photo data/flights" --output-csv result.csv

# 递归扫描子目录
uv run python summarize_uav_flight_info.py --data-root "F:/Photo data/flights" --recursive

# 控制并行数（默认: min(cpu_count, 8)）
uv run python summarize_uav_flight_info.py --data-root "F:/Photo data/flights" --workers 4
uv run python summarize_uav_flight_info.py --data-root "F:/Photo data/flights" --workers 1  # 单进程，便于调试
```

## 输入数据结构

```
DATA_ROOT/
  flight_folder_1/          <- 每个子文件夹 = 一次飞行
    DJI_20240904121232_0001_V.JPG
    DJI_20240904121233_0002_V.JPG
    ...
  flight_folder_2/
    1_rgb/
      DJI_20250824140215_0001_W.JPG
      ...
    2_nir-raw/
      ...
```

支持的 DJI 照片后缀：

| 类型 | 后缀 |
|------|------|
| 可见光 (VIS) | `_V.JPG`, `_D.JPG`, `_W.JPG` |
| 热红外 (TIR) | `_T.JPG` |
| 多光谱 (MS) | `_MS_G.TIF`, `_MS_NIR.TIF`, `_MS_R.TIF`, `_MS_RE.TIF` |

## 输出字段

| 字段 | 说明 |
|------|------|
| 数据文件夹 | 飞行文件夹名称 |
| 观测日期 | YYYY-MM-DD |
| 观测开始时间 | YYYYMMDD HHMMSS |
| 观测结束时间 | YYYYMMDD HHMMSS |
| 操作员 | 默认 XXX，可在脚本中修改 |
| 飞行目的 | 默认 XXX，可在脚本中修改 |
| 无人机/相机型号 | 从 KMZ 或 EXIF Model 读取 |
| 载荷类型 | 可见光 / 可见光+热红外 / 可见光+多光谱 |
| 航线类型 | 正射 / 倾斜（来自 KMZ） |
| 测区中心经度 | 十进制度 |
| 测区中心纬度 | 十进制度 |
| 测区覆盖面积(km2) | KMZ 多边形或 GPS 凸包面积 |
| 观测时长(min) | 首张到末张照片的时间差 |
| 影像数量(张/组) | 按拍摄组计数 |
| 平均航高(m) | GPS 高度平均值 |
| 航向重叠率(%) | KMZ 或 GPS 几何计算，归化至 5 的倍数 |
| 旁向重叠率(%) | KMZ 或 GPS 几何计算，归化至 5 的倍数 |
| 地面分辨率(cm) | GSD，从 EXIF 或相机参数计算 |

## 相机参数配置

`camera_params.yaml` 存储各相机型号的传感器物理参数，用于 GSD 和重叠率的回退计算。当 EXIF 中缺少焦距/传感器信息时（如部分热红外/多光谱相机），脚本会从此文件读取。

```yaml
# key 必须与 EXIF Model 标签值一致
M3E:
  focal_length_mm: 12.0
  sensor_width_mm: 17.3
  sensor_height_mm: 13.0
  image_width_px: 5280
  image_height_px: 3956

H30T:
  focal_length_mm: 12.0
  sensor_width_mm: 17.3
  sensor_height_mm: 13.0
  image_width_px: 4000
  image_height_px: 3000
```

五个字段均为必填。可通过 `--camera-params` 指定其他路径。

缺少此文件时脚本照常运行，GSD 和重叠率字段可能为空。

## 依赖

- Python 3.10+
- 包管理器：`uv`（依赖通过 `pyproject.toml` 管理，安装在 `.venv/` 中）
- 核心功能仅使用标准库
- 可选依赖（缺少时自动降级）：
  - `Pillow` — 更健壮的 EXIF 读取
  - `tqdm` — 进度条
  - `PyYAML` — 加载 `camera_params.yaml`

## 已支持的无人机型号

通过 KMZ `droneEnumValue` 识别：

| 型号 | 枚举值 |
|------|--------|
| DJI Matrice 400 | 103/0 |
| DJI Matrice 350 RTK | 89/0 |
| DJI Matrice 300 RTK | 60/0 |
| DJI Matrice 30 / 30T | 67/0, 67/1 |
| DJI Mavic 3E / 3T / 3TA | 77/0, 77/1, 77/3 |
| DJI Matrice 3D / 3TD | 91/0, 91/1 |
| DJI Matrice 4D / 4TD | 100/0, 100/1 |
| DJI Matrice 4E / 4T | 99/0, 99/1 |

其他型号通过 EXIF `Model` 标签识别，显示原始型号字符串（如 M3E、H30T）。

## License

MIT
