# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

A Python CLI tool that scans DJI drone flight data folders, extracts metadata from photos (EXIF/XMP) and DJI KMZ mission files, and produces a summary CSV — one row per flight folder.

## Environment

This project runs on **Windows**. The shell is Git Bash (bash syntax, forward slashes in paths). Use `uv` as the package manager and `uv run` to execute scripts within the virtual environment.

## Running

```bash
# Basic usage (default data root is D:\NewData)
uv run python summarize_uav_flight_info.py

# Custom input/output
uv run python summarize_uav_flight_info.py --data-root "F:/Photo data/flights" --output-csv result.csv

# Recursive scan of nested directories
uv run python summarize_uav_flight_info.py --data-root "F:/Photo data/flights" --recursive

# Control parallelism (default: min(cpu_count, 8))
uv run python summarize_uav_flight_info.py --workers 4
uv run python summarize_uav_flight_info.py --workers 1   # single-process, easier to debug
```

## Dependencies

- **Required**: Python 3.10+ (standard library only for core functionality)
- **Optional**: `Pillow` (for robust EXIF reading), `tqdm` (progress bar), `PyYAML` (for camera params) — gracefully degrades without them
- **Package manager**: `uv` — dependencies are managed via `pyproject.toml` and installed in `.venv/`

## Architecture

The script is a single file (`summarize_uav_flight_info.py`, ~1250 lines) organized as a pipeline:

1. **Folder discovery** (`list_candidate_folders`) — finds folders containing DJI photo files by suffix matching
2. **Per-folder summarization** (`summarize_folder`) — the core logic, processes one flight folder into a CSV row dict
3. **Multiprocessing orchestration** (`iter_with_progress`) — runs `summarize_folder_safe` in parallel via `ProcessPoolExecutor`
4. **CSV output** (`write_csv`) — writes UTF-8-BOM CSV with Chinese column headers

### Key data flows in `summarize_folder`

- **Photo metadata**: `extract_all_exif_data` extracts GPS, capture time, sensor model, and GSD parameters from each photo's EXIF/XMP in a single pass. GPS uses a custom byte-level TIFF parser (`parse_tiff_gps`) with Pillow fallback.
- **KMZ mission metadata**: `parse_kmz_metadata` reads `wpmz/template.kml` inside KMZ archives for drone model, route type (orthographic/oblique), overlap rates, and polygon area.
- **Area estimation**: Uses convex hull of GPS points as fallback when KMZ polygon data is unavailable.
- **GSD calculation**: `estimate_gsd_cm` computes ground sample distance from altitude, focal length, and sensor width. Falls back to `camera_params.yaml` when EXIF data is incomplete.
- **Overlap from GPS**: When KMZ data is unavailable, `estimate_overlap_from_gps` segments photos into flight lines (by time gaps and heading changes) and computes heading/side overlap from GPS distances and camera footprint dimensions. Overlap percentages are rounded to nearest 5%.

### DJI photo file conventions

Photos are classified by suffix into payload types:
- **Visible (VIS)**: `_V.JPG`, `_D.JPG`, `_W.JPG`
- **Thermal IR (TIR)**: `_T.JPG`
- **Multispectral (MS)**: `_MS_G.TIF`, `_MS_NIR.TIF`, `_MS_R.TIF`, `_MS_RE.TIF`

Image groups are identified by stripping these suffixes — files sharing the same stem belong to one capture group.

### DJI drone model identification

Drone model comes from two sources (in priority order):
1. KMZ `droneEnumValue`/`droneSubEnumValue` looked up in `DRONE_ENUM_MAP`
2. EXIF `CameraModelName`/`Model` tag as fallback

## Configuration

User-facing constants at the top of the file (lines 40-50):
- `DEFAULT_DATA_ROOT`, `DEFAULT_OUTPUT_CSV` — CLI defaults
- `OPERATOR_NAME`, `FLIGHT_PURPOSE` — placeholder values written to each CSV row

### Camera parameters (`camera_params.yaml`)

Optional YAML file (same directory as script, or via `--camera-params`) storing per-camera-model sensor specs. Keys must match the EXIF `Model` tag (e.g., `M3E`, `H30T`). Required fields per model: `focal_length_mm`, `sensor_width_mm`, `sensor_height_mm`, `image_width_px`, `image_height_px`.

Used as fallback for GSD calculation (when EXIF focal/sensor data is missing) and for GPS-based overlap estimation (when no KMZ file exists). Script proceeds without it if absent.
