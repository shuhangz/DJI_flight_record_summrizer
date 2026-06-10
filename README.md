# DJI Flight Record Summarizer

Scans DJI drone flight data folders, extracts metadata from photo EXIF/XMP and KMZ mission files, and produces one summary row per flight folder as a CSV file.

## Features

- Automatically detects DJI photos (visible, thermal infrared, multispectral) in folders
- Extracts GPS, capture time, camera model, focal length, and other parameters from EXIF/XMP
- Reads drone model, route type, overlap rates, and survey polygon from KMZ mission files
- When no KMZ is available, computes heading/side overlap from GPS track geometry (adaptive flight-line segmentation)
- Supports `camera_params.yaml` for camera sensor specs, used as fallback for GSD and overlap calculations
- Multi-process parallel processing with progress bar support

## Quick Start

```bash
# Basic usage
uv run python summarize_uav_flight_info.py --data-root "F:/Photo data/flight_folders"

# Specify output path
uv run python summarize_uav_flight_info.py --data-root "F:/Photo data/flights" --output-csv result.csv

# Recursively scan subdirectories
uv run python summarize_uav_flight_info.py --data-root "F:/Photo data/flights" --recursive

# Control parallelism (default: min(cpu_count, 8))
uv run python summarize_uav_flight_info.py --data-root "F:/Photo data/flights" --workers 4
uv run python summarize_uav_flight_info.py --data-root "F:/Photo data/flights" --workers 1  # single-process, easier to debug
```

## Input Data Structure

```
DATA_ROOT/
  flight_folder_1/          <- each subfolder = one flight
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

Supported DJI photo suffixes:

| Type | Suffixes |
|------|----------|
| Visible (VIS) | `_V.JPG`, `_D.JPG`, `_W.JPG` |
| Thermal IR (TIR) | `_T.JPG` |
| Multispectral (MS) | `_MS_G.TIF`, `_MS_NIR.TIF`, `_MS_R.TIF`, `_MS_RE.TIF` |

## Output Fields

| Field | Description |
|-------|-------------|
| Data Folder | Flight folder name |
| Observation Date | YYYY-MM-DD |
| Observation Start Time | YYYYMMDD HHMMSS |
| Observation End Time | YYYYMMDD HHMMSS |
| Operator | Default XXX, editable in script |
| Flight Purpose | Default XXX, editable in script |
| Drone / Camera Model | From KMZ or EXIF Model tag |
| Payload Type | Visible / Visible+Thermal / Visible+Multispectral |
| Route Type | Orthographic / Oblique (from KMZ) |
| Survey Center Longitude | Decimal degrees |
| Survey Center Latitude | Decimal degrees |
| Survey Area (km2) | KMZ polygon or GPS convex hull area |
| Flight Duration (min) | Time difference between first and last photo |
| Image Count (shots/groups) | Counted by capture groups |
| Average Flight Altitude (m) | Mean GPS altitude |
| Heading Overlap (%) | From KMZ or GPS geometry, rounded to nearest 5% |
| Side Overlap (%) | From KMZ or GPS geometry, rounded to nearest 5% |
| Ground Sample Distance (cm) | GSD, from EXIF or camera parameter calculation |

## Camera Parameter Configuration

`camera_params.yaml` stores sensor physical parameters per camera model, used as fallback for GSD and overlap calculations. When EXIF lacks focal length or sensor info (common with some thermal/multispectral cameras), the script reads from this file.

```yaml
# Keys must match the EXIF Model tag value
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

All five fields are required per model. Use `--camera-params` to specify an alternative path.

The script runs normally without this file; GSD and overlap fields may be empty.

## Dependencies

- Python 3.10+
- Package manager: `uv` (dependencies managed via `pyproject.toml`, installed in `.venv/`)
- Core functionality uses only the standard library
- Optional dependencies (gracefully degrades without them):
  - `Pillow` — more robust EXIF reading
  - `tqdm` — progress bar
  - `PyYAML` — loads `camera_params.yaml`

## Supported Drone Models

Identified via KMZ `droneEnumValue`:

| Model | Enum Values |
|-------|-------------|
| DJI Matrice 400 | 103/0 |
| DJI Matrice 350 RTK | 89/0 |
| DJI Matrice 300 RTK | 60/0 |
| DJI Matrice 30 / 30T | 67/0, 67/1 |
| DJI Mavic 3E / 3T / 3TA | 77/0, 77/1, 77/3 |
| DJI Matrice 3D / 3TD | 91/0, 91/1 |
| DJI Matrice 4D / 4TD | 100/0, 100/1 |
| DJI Matrice 4E / 4T | 99/0, 99/1 |

Other models are identified via the EXIF `Model` tag, displaying the raw model string (e.g., M3E, H30T).

## License

MIT
