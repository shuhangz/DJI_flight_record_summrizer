#!/usr/bin/env python3
"""
Build one simplified UAV flight summary CSV from raw DJI flight folders.

Input layout:
  DATA_ROOT/
    flight_folder_1/
    flight_folder_2/
    ...

Each flight folder may contain VIS, VIS+TIR, or VIS+MS photo groups and an
optional DJI KMZ mission file. One flight folder becomes one CSV row.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import re
import struct
import sys
import zipfile
from concurrent.futures import ProcessPoolExecutor, as_completed
from collections import Counter
from datetime import datetime
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence, Tuple
from xml.etree import ElementTree as ET

try:
    from PIL import ExifTags, Image
    from tqdm import tqdm
except Exception:
    ExifTags = None
    Image = None
    tqdm = None


# 用户配置区域。命令行参数可以覆盖这些默认值。
DEFAULT_DATA_ROOT = Path(r"D:\NewData")
DEFAULT_OUTPUT_CSV = Path("./uav_flight_info_summary.csv")

OPERATOR_NAME = "XXX"
FLIGHT_PURPOSE = "XXX"

VIS_SUFFIXES = ("_V.JPG", "_D.JPG", "_W.JPG")
TIR_SUFFIXES = ("_T.JPG",)
MS_SUFFIXES = ("_MS_G.TIF", "_MS_NIR.TIF", "_MS_R.TIF", "_MS_RE.TIF")
UAV_PHOTO_SUFFIXES = VIS_SUFFIXES + TIR_SUFFIXES + MS_SUFFIXES

HEADERS = [
    "数据文件夹",
    "观测日期",
    "观测开始时间",
    "观测结束时间",
    "操作员",
    "飞行目的",
    "无人机/相机型号",
    "载荷类型",
    "航线类型",
    "测区中心经度",
    "测区中心纬度",
    "测区覆盖面积(km2)",
    "观测时长(min)",
    "影像数量(张/组)",
    "平均航高(m)",
    "航向重叠率(%)",
    "旁向重叠率(%)",
    "地面分辨率(cm)",
]

# DJI Cloud API product support table: domain=0 aircraft entries.
# Source: https://developer.dji.com/doc/cloud-api-tutorial/cn/overview/product-support.html
DRONE_ENUM_MAP = {
    ("103", "0"): "DJI Matrice 400",
    ("89", "0"): "DJI Matrice 350 RTK",
    ("60", "0"): "DJI Matrice 300 RTK",
    ("67", "0"): "DJI Matrice 30",
    ("67", "1"): "DJI Matrice 30T",
    ("77", "0"): "DJI Mavic 3E",
    ("77", "1"): "DJI Mavic 3T",
    ("77", "3"): "DJI Mavic 3TA",
    ("91", "0"): "DJI Matrice 3D",
    ("91", "1"): "DJI Matrice 3TD",
    ("100", "0"): "DJI Matrice 4D",
    ("100", "1"): "DJI Matrice 4TD",
    ("99", "0"): "DJI Matrice 4E",
    ("99", "1"): "DJI Matrice 4T",
}

DRONE_TYPE_FALLBACK_MAP = {
    "103": "DJI Matrice 400",
    "89": "DJI Matrice 350 RTK",
    "60": "DJI Matrice 300 RTK",
    "67": "DJI Matrice 30",
    "77": "DJI Mavic 3E/T/TA",
    "91": "DJI Matrice 3D/3TD",
    "100": "DJI Matrice 4D/4TD",
    "99": "DJI Matrice 4E/4T",
}

WPML_NS = {"wpml": "http://www.dji.com/wpmz/1.0.6", "kml": "http://www.opengis.net/kml/2.2"}

# 预编译的正则表达式，避免每次调用时重复编译
_RE_XMP_DATETIME_PATTERNS = [
    re.compile(rb'DateTimeOriginal="([^"]+)"'),
    re.compile(rb'CreateDate="([^"]+)"'),
    re.compile(rb'<exif:DateTimeOriginal>([^<]+)</exif:DateTimeOriginal>'),
    re.compile(rb'<xmp:CreateDate>([^<]+)</xmp:CreateDate>'),
]

_RE_XMP_NUM_VALUE = re.compile(rb'([-+]?\d+(?:\.\d+)?)')

# 航带分割阈值
TIME_GAP_THRESHOLD_S = 10.0
HEADING_CHANGE_THRESHOLD_DEG = 45.0

_REQUIRED_CAM_KEYS = {"focal_length_mm", "sensor_width_mm", "sensor_height_mm",
                      "image_width_px", "image_height_px"}


def load_camera_params(path: Path) -> Dict[str, Dict[str, float]]:
    """Load per-camera-model parameters from YAML. Returns {} on any failure."""
    if not path.exists():
        print(f"[WARN] Camera params file not found: {path}. "
              "Overlap and GSD will rely solely on EXIF data.", file=sys.stderr)
        return {}
    try:
        import yaml
    except ImportError:
        print("[WARN] PyYAML not installed. Camera params file cannot be loaded.", file=sys.stderr)
        return {}
    try:
        with path.open("r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
    except Exception as e:
        print(f"[WARN] Failed to read camera params: {e}", file=sys.stderr)
        return {}
    if not isinstance(data, dict):
        print("[WARN] Camera params file must be a YAML dict.", file=sys.stderr)
        return {}

    result: Dict[str, Dict[str, float]] = {}
    for model, params in data.items():
        if not isinstance(params, dict):
            print(f"[WARN] Camera '{model}': expected dict, skipping.", file=sys.stderr)
            continue
        missing = _REQUIRED_CAM_KEYS - set(params.keys())
        if missing:
            print(f"[WARN] Camera '{model}': missing keys {missing}, skipping.", file=sys.stderr)
            continue
        try:
            validated = {k: float(params[k]) for k in _REQUIRED_CAM_KEYS}
            if any(v <= 0 for v in validated.values()):
                print(f"[WARN] Camera '{model}': all values must be positive, skipping.", file=sys.stderr)
                continue
            result[str(model)] = validated
        except (TypeError, ValueError):
            print(f"[WARN] Camera '{model}': non-numeric value, skipping.", file=sys.stderr)
    return result


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Summarize UAV flight folders into one CSV.")
    parser.add_argument(
        "--data-root",
        default=str(DEFAULT_DATA_ROOT),
        help="Main data folder A. Each direct child folder is treated as one flight folder B.",
    )
    parser.add_argument("--output-csv", default=str(DEFAULT_OUTPUT_CSV), help="Output CSV path.")
    parser.add_argument(
        "--recursive",
        action="store_true",
        help="Also scan nested directories as candidate flight folders. By default only direct children are summarized.",
    )
    parser.add_argument(
        "--workers",
        type=int,
        default=max(1, min((os.cpu_count() or 1), 8)),
        help="Number of parallel worker processes. Use 1 to disable multiprocessing. Default: min(CPU count, 8).",
    )
    parser.add_argument(
        "--no-progress",
        action="store_true",
        help="Disable tqdm progress bar.",
    )
    parser.add_argument(
        "--camera-params",
        default=str(Path(__file__).parent / "camera_params.yaml"),
        help="Path to camera_params.yaml with per-model sensor specifications.",
    )
    return parser.parse_args()


def suffix_match(path: Path, suffixes: Sequence[str]) -> bool:
    name = path.name.upper()
    return any(name.endswith(suffix.upper()) for suffix in suffixes)


def normalized_stem_for_suffix(path: Path, suffixes: Sequence[str]) -> Optional[str]:
    name = path.name
    upper = name.upper()
    for suffix in suffixes:
        if upper.endswith(suffix.upper()):
            return name[: -len(suffix)]
    return None


def iter_files(folder: Path) -> Iterable[Path]:
    for path in folder.rglob("*"):
        if path.is_file():
            yield path


def contains_uav_files(folder: Path) -> bool:
    return any(
        p.is_file() and suffix_match(p, UAV_PHOTO_SUFFIXES)
        for p in folder.rglob("*")
    )


def contains_direct_uav_files(folder: Path) -> bool:
    return any(
        p.is_file() and suffix_match(p, UAV_PHOTO_SUFFIXES)
        for p in folder.iterdir()
    )


def list_candidate_folders(root: Path, recursive: bool) -> List[Path]:
    if recursive:
        return sorted([p for p in root.rglob("*") if p.is_dir() and contains_direct_uav_files(p)])
    candidates = sorted([p for p in root.iterdir() if p.is_dir() and contains_uav_files(p)])
    if not candidates and contains_uav_files(root):
        return [root]
    return candidates


def classify_payload(files: Sequence[Path]) -> str:
    has_ms = any(suffix_match(p, MS_SUFFIXES) for p in files)
    if has_ms:
        return "可见光+多光谱"
    has_tir = any(suffix_match(p, TIR_SUFFIXES) for p in files)
    has_vis = any(suffix_match(p, VIS_SUFFIXES) for p in files)
    if has_tir and has_vis:
        return "可见光+热红外"
    return "可见光"


def primary_photo_files(files: Sequence[Path], payload_type: str) -> List[Path]:
    if payload_type == "可见光+多光谱":
        primary = [p for p in files if suffix_match(p, ("_D.JPG",))]
        if primary:
            return sorted(primary)
    if payload_type == "可见光+热红外":
        primary = [p for p in files if suffix_match(p, ("_V.JPG", "_W.JPG"))]
        if primary:
            return sorted(primary)
    primary = [p for p in files if suffix_match(p, VIS_SUFFIXES)]
    if primary:
        return sorted(primary)
    return sorted([p for p in files if p.suffix.upper() in (".JPG", ".JPEG")])


def count_image_groups(files: Sequence[Path], payload_type: str) -> int:
    if payload_type == "可见光+多光谱":
        suffixes = ("_D.JPG",) + MS_SUFFIXES
        groups = {normalized_stem_for_suffix(p, suffixes) for p in files}
        return len([g for g in groups if g])
    if payload_type == "可见光+热红外":
        suffixes = ("_V.JPG", "_W.JPG", "_T.JPG")
        groups = {normalized_stem_for_suffix(p, suffixes) for p in files}
        return len([g for g in groups if g])
    return len(primary_photo_files(files, payload_type))


def to_float_rational(value) -> Optional[float]:
    try:
        if hasattr(value, "numerator") and hasattr(value, "denominator"):
            den = float(value.denominator)
            return None if den == 0 else float(value.numerator) / den
        if isinstance(value, tuple) and len(value) == 2:
            den = float(value[1])
            return None if den == 0 else float(value[0]) / den
        return float(value)
    except Exception:
        return None


def dms_to_decimal(dms_values, ref: str) -> Optional[float]:
    try:
        d = to_float_rational(dms_values[0])
        m = to_float_rational(dms_values[1])
        s = to_float_rational(dms_values[2])
        if d is None or m is None or s is None:
            return None
        value = d + m / 60.0 + s / 3600.0
        if ref.upper() in ("S", "W"):
            value = -value
        return value
    except Exception:
        return None


def parse_capture_time_text(text: object) -> Optional[datetime]:
    if not text:
        return None
    for fmt in ("%Y:%m:%d %H:%M:%S", "%Y-%m-%d %H:%M:%S"):
        try:
            return datetime.strptime(str(text), fmt)
        except ValueError:
            pass
    return None


def parse_sensor_name_with_pillow(photo_path: Path, data: Optional[bytes] = None) -> Optional[str]:
    if Image is None or ExifTags is None:
        return None
    try:
        import io
        source = io.BytesIO(data) if data is not None else photo_path
        with Image.open(source) as img:
            raw_exif = img.getexif()
        if not raw_exif:
            return None
        named = {str(ExifTags.TAGS.get(k, k)): v for k, v in raw_exif.items()}
        values = [
            named.get("CameraModelName"),
            named.get("Model"),
            named.get("LensModel"),
            named.get("Make"),
        ]
        valid = [str(v).strip() for v in values if v and str(v).strip()]
        if valid:
            return " | ".join(valid[:2])
    except Exception:
        return None
    return None


def parse_exif_time_with_pillow(photo_path: Path, data: Optional[bytes] = None) -> Optional[datetime]:
    if Image is None or ExifTags is None:
        return None
    try:
        import io
        source = io.BytesIO(data) if data is not None else photo_path
        with Image.open(source) as img:
            raw_exif = img.getexif()
        if not raw_exif:
            return None
        named = {str(ExifTags.TAGS.get(k, k)): v for k, v in raw_exif.items()}
        for tag in ("DateTimeOriginal", "DateTimeDigitized", "DateTime"):
            parsed = parse_capture_time_text(named.get(tag))
            if parsed:
                return parsed
    except Exception:
        return None
    return None


def get_gps_with_pillow(photo_path: Path, data: Optional[bytes] = None) -> Optional[Tuple[float, float, Optional[float]]]:
    if Image is None or ExifTags is None:
        return None
    try:
        import io
        source = io.BytesIO(data) if data is not None else photo_path
        with Image.open(source) as img:
            raw_exif = img.getexif()
        if not raw_exif:
            return None
        # named = {str(ExifTags.TAGS.get(k, k)): v for k, v in raw_exif.items()}
        # gps_info = named.get("GPSInfo")
        gps_info = raw_exif.get_ifd(ExifTags.IFD.GPSInfo)
        if not gps_info or not isinstance(gps_info, dict):
            return None
        gps_named = {str(ExifTags.GPSTAGS.get(k, k)): v for k, v in gps_info.items()}
        lat = dms_to_decimal(gps_named.get("GPSLatitude"), str(gps_named.get("GPSLatitudeRef", "N")))
        lon = dms_to_decimal(gps_named.get("GPSLongitude"), str(gps_named.get("GPSLongitudeRef", "E")))
        if lat is None or lon is None:
            return None
        altitude = to_float_rational(gps_named.get("GPSAltitude"))
        alt_ref = gps_named.get("GPSAltitudeRef", 0)
        if altitude is not None:
            try:
                if int(alt_ref) == 1:
                    altitude = -altitude
            except Exception:
                pass
        return lon, lat, altitude
    except Exception:
        return None


def get_gps_from_exif_bytes(photo_path: Path, data: Optional[bytes] = None) -> Optional[Tuple[float, float, Optional[float]]]:
    if data is None:
        try:
            data = photo_path.read_bytes()
        except Exception:
            return None
    if len(data) < 4 or data[0:2] != b"\xFF\xD8":
        return None

    pos = 2
    while pos + 4 <= len(data):
        if data[pos] != 0xFF:
            break
        marker = struct.unpack(">H", data[pos : pos + 2])[0]
        if marker in (0xFFD8, 0xFFD9):
            pos += 2
            continue
        seg_len = struct.unpack(">H", data[pos + 2 : pos + 4])[0]
        if seg_len < 2 or pos + 2 + seg_len > len(data):
            break
        if marker == 0xFFE1:
            app1_data = data[pos + 4 : pos + 2 + seg_len]
            if app1_data[:6] == b"Exif\x00\x00":
                parsed = parse_tiff_gps(app1_data[6:])
                if parsed is not None:
                    return parsed
        pos += 2 + seg_len
    return None


def parse_tiff_gps(tiff_data: bytes) -> Optional[Tuple[float, float, Optional[float]]]:
    try:
        if tiff_data[:2] == b"II":
            endian = "<"
        elif tiff_data[:2] == b"MM":
            endian = ">"
        else:
            return None
        if len(tiff_data) < 8:
            return None
        ifd0_offset = struct.unpack(endian + "I", tiff_data[4:8])[0]

        def read_ifd(offset: int):
            if offset < 0 or offset + 2 > len(tiff_data):
                return None
            count = struct.unpack(endian + "H", tiff_data[offset : offset + 2])[0]
            entries = {}
            for i in range(count):
                entry_offset = offset + 2 + i * 12
                if entry_offset + 12 > len(tiff_data):
                    return None
                tag = struct.unpack(endian + "H", tiff_data[entry_offset : entry_offset + 2])[0]
                type_ = struct.unpack(endian + "H", tiff_data[entry_offset + 2 : entry_offset + 4])[0]
                count_ = struct.unpack(endian + "I", tiff_data[entry_offset + 4 : entry_offset + 8])[0]
                entries[tag] = (type_, count_, entry_offset + 8)
            return entries

        def read_raw(type_: int, count_: int, value_offset: int) -> Optional[bytes]:
            type_sizes = {1: 1, 2: 1, 3: 2, 4: 4, 5: 8, 6: 1, 7: 1, 8: 2, 9: 4, 10: 8, 11: 4, 12: 8}
            item_size = type_sizes.get(type_)
            if item_size is None:
                return None
            size = item_size * count_
            if size <= 4:
                return tiff_data[value_offset : value_offset + size]
            ptr = struct.unpack(endian + "I", tiff_data[value_offset : value_offset + 4])[0]
            if ptr < 0 or ptr + size > len(tiff_data):
                return None
            return tiff_data[ptr : ptr + size]

        def read_ascii(type_: int, count_: int, value_offset: int) -> str:
            raw = read_raw(type_, count_, value_offset)
            return "" if raw is None else raw.rstrip(b"\x00").decode("ascii", errors="ignore")

        def read_rationals(type_: int, count_: int, value_offset: int):
            raw = read_raw(type_, count_, value_offset)
            if raw is None:
                return None
            out = []
            for i in range(count_):
                chunk = raw[i * 8 : (i + 1) * 8]
                if len(chunk) < 8:
                    return None
                if type_ == 5:
                    out.append(struct.unpack(endian + "II", chunk))
                elif type_ == 10:
                    out.append(struct.unpack(endian + "ii", chunk))
                else:
                    return None
            return out

        def rational_to_float(pair) -> Optional[float]:
            if not pair:
                return None
            num, den = pair
            return None if den == 0 else float(num) / float(den)

        def rationals_to_deg(rats) -> Optional[float]:
            if not rats or len(rats) < 3:
                return None
            d = rational_to_float(rats[0])
            m = rational_to_float(rats[1])
            s = rational_to_float(rats[2])
            if d is None or m is None or s is None:
                return None
            return d + m / 60.0 + s / 3600.0

        ifd0 = read_ifd(ifd0_offset)
        if not ifd0 or 0x8825 not in ifd0:
            return None
        gps_offset_raw = read_raw(*ifd0[0x8825])
        if gps_offset_raw is None or len(gps_offset_raw) < 4:
            return None
        gps_ifd = read_ifd(struct.unpack(endian + "I", gps_offset_raw[:4])[0])
        if not gps_ifd or 2 not in gps_ifd or 4 not in gps_ifd:
            return None

        lat = rationals_to_deg(read_rationals(*gps_ifd[2]))
        lon = rationals_to_deg(read_rationals(*gps_ifd[4]))
        if lat is None or lon is None:
            return None
        if 1 in gps_ifd and read_ascii(*gps_ifd[1])[:1].upper() == "S":
            lat = -lat
        if 3 in gps_ifd and read_ascii(*gps_ifd[3])[:1].upper() == "W":
            lon = -lon

        altitude = None
        if 6 in gps_ifd:
            alt_rats = read_rationals(*gps_ifd[6])
            if alt_rats:
                altitude = rational_to_float(alt_rats[0])
            if altitude is not None and 5 in gps_ifd:
                alt_ref = read_raw(*gps_ifd[5])
                if alt_ref and alt_ref[0] == 1:
                    altitude = -altitude
        return lon, lat, altitude
    except Exception:
        return None


def parse_xmp_datetime(photo_path: Path, data: Optional[bytes] = None) -> Optional[datetime]:
    if data is None:
        try:
            data = photo_path.read_bytes()
        except Exception:
            return None
    for pattern in _RE_XMP_DATETIME_PATTERNS:
        match = pattern.search(data)
        if not match:
            continue
        text = match.group(1).decode("utf-8", errors="ignore").replace("T", " ").replace("Z", "")
        text = re.sub(r"([+-]\d\d:\d\d)$", "", text)
        parsed = parse_capture_time_text(text[:19])
        if parsed:
            return parsed
    return None


def read_string_xmp_value(data: bytes, names: Sequence[str]) -> Optional[str]:
    """Read a string attribute or element value from XMP data."""
    for name in names:
        escaped = re.escape(name.encode("ascii"))
        # attribute form: Name="value"
        attr_pattern = rb'(?:drone-dji:|tiff:|exif:)?' + escaped + rb'="([^"]+)"'
        match = re.search(attr_pattern, data)
        if match:
            return match.group(1).decode("utf-8", errors="ignore").strip()
        # element form: <Name>value</Name>
        elem_pattern = rb'<(?:drone-dji:|tiff:|exif:)?' + escaped + rb'>([^<]+)</'
        match = re.search(elem_pattern, data)
        if match:
            return match.group(1).decode("utf-8", errors="ignore").strip()
    return None


def parse_xmp_all(data: bytes) -> Dict[str, str]:
    """Single-pass XMP extraction. Returns {short_key: value} dict."""
    result: Dict[str, str] = {}
    xmp_start = data.find(b'<x:xmpmeta')
    if xmp_start < 0:
        return result
    xmp_end = data.find(b'</x:xmpmeta>', xmp_start)
    if xmp_end < 0:
        return result
    xmp_bytes = data[xmp_start:xmp_end + 13]
    # Attribute form: key="value"
    for m in re.finditer(rb'([\w:-]+)\s*=\s*"([^"]*)"', xmp_bytes):
        key = m.group(1).decode("utf-8", errors="ignore")
        # Strip namespace prefix: "drone-dji:Model" -> "Model"
        if ":" in key:
            key = key.rsplit(":", 1)[-1]
        result[key] = m.group(2).decode("utf-8", errors="ignore")
    # Element form: <key>value</key>
    for m in re.finditer(rb'<([\w:-]+)>([^<]+)</', xmp_bytes):
        key = m.group(1).decode("utf-8", errors="ignore")
        if ":" in key:
            key = key.rsplit(":", 1)[-1]
        if key not in result:
            result[key] = m.group(2).decode("utf-8", errors="ignore")
    return result


def read_numeric_xmp_value(data: bytes, names: Sequence[str]) -> Optional[float]:
    for name in names:
        escaped = re.escape(name.encode("ascii"))

        attr_pattern = rb'(?:drone-dji:|exif:|tiff:)?' + escaped + rb'="' + _RE_XMP_NUM_VALUE.pattern + rb'"'
        match = re.search(attr_pattern, data)
        if match:
            return float(match.group(1))

        elem_pattern = (
            rb'<(?:drone-dji:|exif:|tiff:)?'
            + escaped
            + rb'>'
            + _RE_XMP_NUM_VALUE.pattern
            + rb'</(?:drone-dji:|exif:|tiff:)?'
            + escaped
            + rb'>'
        )
        match = re.search(elem_pattern, data)
        if match:
            return float(match.group(1))

    return None


def jpeg_width(photo_path: Path, data: Optional[bytes] = None) -> Optional[int]:
    if data is None:
        try:
            data = photo_path.read_bytes()
        except Exception:
            return None
    if len(data) < 4 or data[:2] != b"\xFF\xD8":
        return None
    pos = 2
    while pos + 9 <= len(data):
        if data[pos] != 0xFF:
            pos += 1
            continue
        marker = data[pos + 1]
        pos += 2
        if marker in (0xD8, 0xD9):
            continue
        if pos + 2 > len(data):
            return None
        seg_len = struct.unpack(">H", data[pos : pos + 2])[0]
        if seg_len < 2 or pos + seg_len > len(data):
            return None
        if marker in (0xC0, 0xC1, 0xC2, 0xC3, 0xC5, 0xC6, 0xC7, 0xC9, 0xCA, 0xCB, 0xCD, 0xCE, 0xCF):
            if pos + 7 <= len(data):
                return struct.unpack(">H", data[pos + 5 : pos + 7])[0]
            return None
        pos += seg_len
    return None


def parse_gsd_exif_inputs(photo_path: Optional[Path], data: Optional[bytes] = None) -> Optional[Tuple[float, float, int]]:
    if photo_path is None:
        return None

    focal_mm = None
    focal_35mm = None
    image_width = None

    if Image is not None and ExifTags is not None:
        try:
            import io
            source = io.BytesIO(data) if data is not None else photo_path
            with Image.open(source) as img:
                image_width = int(img.width) if img.width else None
                raw_exif = img.getexif()

            named = {}
            if raw_exif:
                # 1. 获取基础 IFD0 标签
                named = {str(ExifTags.TAGS.get(k, k)): v for k, v in raw_exif.items()}
                # 2. 深入获取 Exif IFD 标签（焦距等信息存在这里）
                try:
                    exif_ifd = raw_exif.get_ifd(ExifTags.IFD.Exif)
                    for k, v in exif_ifd.items():
                        named[str(ExifTags.TAGS.get(k, k))] = v
                except Exception:
                    pass

            focal_mm = to_float_rational(named.get("FocalLength"))
            focal_35mm = to_float_rational(named.get("FocalLengthIn35mmFilm"))
            exif_width = named.get("ExifImageWidth") or named.get("ImageWidth")
            if image_width is None and exif_width is not None:
                image_width = int(to_float_rational(exif_width) or 0) or None
        except Exception:
            pass

    if data is None:
        try:
            data = photo_path.read_bytes()
        except Exception:
            data = b""

    if focal_mm is None and data:
        focal_mm = read_numeric_xmp_value(data, ("FocalLength", "CalibratedFocalLength"))
    if focal_35mm is None and data:
        focal_35mm = read_numeric_xmp_value(data, ("FocalLengthIn35mmFilm", "FocalLengthIn35mmFormat"))
    if image_width is None and data:
        width = read_numeric_xmp_value(data, ("ExifImageWidth", "ImageWidth", "PixelXDimension"))
        image_width = int(width) if width else jpeg_width(photo_path, data)

    if not focal_mm or not focal_35mm or not image_width:
        return None
    return float(focal_mm), float(focal_35mm), int(image_width)


def estimate_gsd_cm(
    avg_altitude_m: Optional[float],
    focal_mm: Optional[float],
    focal_35mm: Optional[float],
    image_width_px: Optional[int],
) -> Optional[float]:
    if not avg_altitude_m or not focal_mm or not focal_35mm or not image_width_px:
        return None
    sensor_width_mm = (focal_mm / focal_35mm) * 36.0
    return ((avg_altitude_m * sensor_width_mm) / (focal_mm * image_width_px)) * 100.0


def read_file_header(photo_path: Path, max_bytes: int = 65536) -> Optional[bytes]:
    """只读取文件前max_bytes字节（EXIF头部在文件开头）"""
    try:
        with open(photo_path, "rb") as f:
            return f.read(max_bytes)
    except Exception:
        return None


def extract_all_exif_data(photo_path: Path, data: Optional[bytes] = None) -> Dict[str, object]:
    """一次性提取所有EXIF数据，避免多次Image.open()"""
    result = {
        "data": data,
        "gps": None,
        "capture_time": None,
        "sensor": None,
        "focal_mm": None,
        "focal_35mm": None,
        "image_width": None,
    }

    if data is None:
        # 只读取文件前128KB，EXIF数据在文件头部
        data = read_file_header(photo_path, 131072)
        result["data"] = data

    # GPS: 先用字节解析，再用Pillow
    result["gps"] = get_gps_from_exif_bytes(photo_path, data) or get_gps_with_pillow(photo_path, data)

    # 时间: 先用Pillow，再用XMP
    result["capture_time"] = parse_exif_time_with_pillow(photo_path, data) or parse_xmp_datetime(photo_path, data)

    # 一次性用Pillow提取所有EXIF标签
    if Image is not None and ExifTags is not None:
        try:
            import io
            source = io.BytesIO(data) if data is not None else photo_path
            with Image.open(source) as img:
                result["image_width"] = int(img.width) if img.width else None
                raw_exif = img.getexif()

            named = {}
            if raw_exif:
                named = {str(ExifTags.TAGS.get(k, k)): v for k, v in raw_exif.items()}
                try:
                    exif_ifd = raw_exif.get_ifd(ExifTags.IFD.Exif)
                    for k, v in exif_ifd.items():
                        named[str(ExifTags.TAGS.get(k, k))] = v
                except Exception:
                    pass

            # 相机型号 - 优先使用Model标签
            model = named.get("CameraModelName") or named.get("Model")
            if model:
                result["sensor"] = str(model).strip()

            # GSD相关参数
            result["focal_mm"] = to_float_rational(named.get("FocalLength"))
            result["focal_35mm"] = to_float_rational(named.get("FocalLengthIn35mmFilm"))
            exif_width = named.get("ExifImageWidth") or named.get("ImageWidth")
            if result["image_width"] is None and exif_width is not None:
                result["image_width"] = int(to_float_rational(exif_width) or 0) or None
        except Exception as e:
            import sys
            print(f"[DEBUG] EXIF extraction error: {e}", file=sys.stderr)
            import traceback
            traceback.print_exc(file=sys.stderr)

    # XMP补充: 单次遍历提取所有XMP字段
    if data:
        xmp = parse_xmp_all(data)
        if result["sensor"] is None:
            result["sensor"] = xmp.get("Model") or xmp.get("DroneModel") or xmp.get("CameraModelName") or None
        if result["focal_mm"] is None:
            val = xmp.get("FocalLength") or xmp.get("CalibratedFocalLength")
            result["focal_mm"] = float(val) if val else None
        if result["focal_35mm"] is None:
            val = xmp.get("FocalLengthIn35mmFilm") or xmp.get("FocalLengthIn35mmFormat")
            result["focal_35mm"] = float(val) if val else None
        if result["image_width"] is None:
            val = xmp.get("ExifImageWidth") or xmp.get("ImageWidth") or xmp.get("PixelXDimension")
            result["image_width"] = int(float(val)) if val else jpeg_width(photo_path, data)

    return result


def parse_photo(photo_path: Path, read_sensor: bool = False) -> Dict[str, object]:
    # 一次性提取所有EXIF数据
    exif_data = extract_all_exif_data(photo_path)

    gps = exif_data["gps"]
    lon, lat, altitude = gps if gps else (None, None, None)

    return {
        "photo": photo_path,
        "capture_time": exif_data["capture_time"],
        "longitude": lon,
        "latitude": lat,
        "gps_altitude": altitude,
        "sensor": exif_data["sensor"] if read_sensor else None,
        "focal_mm": exif_data["focal_mm"],
        "focal_35mm": exif_data["focal_35mm"],
        "image_width": exif_data["image_width"],
    }


def decimal_to_dms(value: Optional[float], is_lon: bool) -> str:
    if value is None:
        return ""
    direction = "E" if is_lon and value >= 0 else "W" if is_lon else "N" if value >= 0 else "S"
    abs_value = abs(value)
    deg = int(abs_value)
    minutes_float = (abs_value - deg) * 60.0
    minute = int(minutes_float)
    second = (minutes_float - minute) * 60.0
    return f"{deg}°{minute:02d}′{second:05.2f}″{direction}"


def average(values: Sequence[float]) -> Optional[float]:
    return sum(values) / len(values) if values else None


def round_text(value: Optional[float], digits: int) -> str:
    if value is None:
        return ""
    return str(round(value, digits))


def polygon_area_km2(coords: Sequence[Tuple[float, float]]) -> Optional[float]:
    if len(coords) < 3:
        return None
    radius = 6371.0088
    mean_lat = math.radians(sum(lat for _, lat in coords) / len(coords))
    xy = []
    for lon, lat in coords:
        x = radius * math.radians(lon) * math.cos(mean_lat)
        y = radius * math.radians(lat)
        xy.append((x, y))
    area = 0.0
    for idx, (x1, y1) in enumerate(xy):
        x2, y2 = xy[(idx + 1) % len(xy)]
        area += x1 * y2 - x2 * y1
    return abs(area) / 2.0


def convex_hull(points: Sequence[Tuple[float, float]]) -> List[Tuple[float, float]]:
    unique = sorted(set(points))
    if len(unique) <= 1:
        return list(unique)

    def cross(origin, a, b):
        return (a[0] - origin[0]) * (b[1] - origin[1]) - (a[1] - origin[1]) * (b[0] - origin[0])

    lower = []
    for point in unique:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0:
            lower.pop()
        lower.append(point)

    upper = []
    for point in reversed(unique):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0:
            upper.pop()
        upper.append(point)

    return lower[:-1] + upper[:-1]


def estimate_area_from_points_km2(points: Sequence[Tuple[float, float]]) -> Optional[float]:
    valid = [(lon, lat) for lon, lat in points if lon is not None and lat is not None]
    if len(valid) < 3:
        return None
    hull = convex_hull(valid)
    if len(hull) < 3:
        return None
    return polygon_area_km2(hull)


def haversine_m(lon1: float, lat1: float, lon2: float, lat2: float) -> float:
    """Haversine distance in meters between two GPS points."""
    R = 6371000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def bearing_rad(lon1: float, lat1: float, lon2: float, lat2: float) -> float:
    """Bearing from point 1 to point 2 in radians [0, 2*pi)."""
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dlam = math.radians(lon2 - lon1)
    x = math.sin(dlam) * math.cos(phi2)
    y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlam)
    return math.atan2(x, y) % (2 * math.pi)


def heading_change_deg(h1: float, h2: float) -> float:
    """Absolute heading change in degrees, handling 360-degree wraparound."""
    diff = abs(h2 - h1)
    return min(diff, 2 * math.pi - diff) * 180.0 / math.pi


def camera_footprint_m(altitude_m: float, sensor_mm: float, focal_mm: float) -> float:
    """Ground coverage in meters for one axis: (altitude * sensor_size) / focal_length."""
    return (altitude_m * sensor_mm) / focal_mm


def local_xy(lon: float, lat: float, ref_lon: float, ref_lat: float) -> Tuple[float, float]:
    """Convert GPS lon/lat to local metric X/Y (meters) relative to a reference point."""
    R = 6371000.0
    x = R * math.radians(lon - ref_lon) * math.cos(math.radians(ref_lat))
    y = R * math.radians(lat - ref_lat)
    return x, y


def segment_flight_lines(records_with_gps: List[Dict]) -> List[List[int]]:
    """Segment photos into flight lines by time gaps and heading changes.

    Returns list of segments, each segment is a list of indices into records_with_gps.
    """
    if len(records_with_gps) < 2:
        return [list(range(len(records_with_gps)))] if records_with_gps else []

    segments: List[List[int]] = [[0]]
    for i in range(1, len(records_with_gps)):
        prev = records_with_gps[i - 1]
        curr = records_with_gps[i]

        time_gap = (curr["capture_time"] - prev["capture_time"]).total_seconds()
        if time_gap > TIME_GAP_THRESHOLD_S:
            segments.append([i])
            continue

        if len(segments[-1]) >= 2:
            first = records_with_gps[segments[-1][0]]
            last = records_with_gps[segments[-1][-1]]
            prev_bearing = bearing_rad(first["longitude"], first["latitude"],
                                       last["longitude"], last["latitude"])
            curr_bearing = bearing_rad(prev["longitude"], prev["latitude"],
                                       curr["longitude"], curr["latitude"])
            if heading_change_deg(prev_bearing, curr_bearing) > HEADING_CHANGE_THRESHOLD_DEG:
                segments.append([i])
                continue

        segments[-1].append(i)

    return segments


def filter_flight_segments(segments: List[List[int]]) -> List[List[int]]:
    """Keep only the 'real' flight line segments using maximum-gap method on segment sizes.

    Sorts segment sizes, finds the largest gap between consecutive sizes,
    and keeps segments above that gap. If all segments are similar size, keeps all.
    """
    if len(segments) <= 1:
        return segments

    sizes_with_idx = sorted((len(s), i) for i, s in enumerate(segments))
    sizes = [s for s, _ in sizes_with_idx]

    # Find the largest gap between consecutive sorted sizes
    max_gap = 0
    split_pos = -1
    for i in range(len(sizes) - 1):
        gap = sizes[i + 1] - sizes[i]
        if gap > max_gap:
            max_gap = gap
            split_pos = i

    # If the largest gap is small relative to the sizes, keep all segments
    if max_gap <= 1:
        return segments

    threshold = sizes[split_pos] + 1
    return [s for s in segments if len(s) >= threshold]


def compute_heading_overlap(along_track_dists: List[float],
                            footprint_along_m: float) -> Optional[float]:
    """Compute heading overlap from along-track distances between consecutive photos."""
    if not along_track_dists or footprint_along_m <= 0:
        return None
    overlaps = [max(0.0, min(1.0, 1.0 - d / footprint_along_m)) for d in along_track_dists]
    median_overlap = sorted(overlaps)[len(overlaps) // 2]
    return round(median_overlap * 100 / 5) * 5


def compute_side_overlap(segments: List[List[int]],
                         records: List[Dict],
                         footprint_cross_m: float,
                         ref_lon: float, ref_lat: float) -> Optional[float]:
    """Compute side overlap from cross-track distances between adjacent flight lines."""
    if len(segments) < 2 or footprint_cross_m <= 0:
        return None

    seg_info: List[Tuple[float, float, float, float]] = []
    for seg in segments:
        lons = [records[i]["longitude"] for i in seg]
        lats = [records[i]["latitude"] for i in seg]
        cx, cy = local_xy(sum(lons) / len(lons), sum(lats) / len(lats), ref_lon, ref_lat)
        x0, y0 = local_xy(lons[0], lats[0], ref_lon, ref_lat)
        x1, y1 = local_xy(lons[-1], lats[-1], ref_lon, ref_lat)
        dx, dy = x1 - x0, y1 - y0
        length = math.hypot(dx, dy)
        if length > 0:
            dx, dy = dx / length, dy / length
        seg_info.append((cx, cy, dx, dy))

    cross_dists: List[float] = []
    for i in range(len(seg_info) - 1):
        cx0, cy0, dx0, dy0 = seg_info[i]
        cx1, cy1, _, _ = seg_info[i + 1]
        # Perpendicular (cross-track) unit vector
        nx, ny = -dy0, dx0
        dist = abs((cx1 - cx0) * nx + (cy1 - cy0) * ny)
        cross_dists.append(dist)

    if not cross_dists:
        return None
    median_dist = sorted(cross_dists)[len(cross_dists) // 2]
    overlap = max(0.0, min(1.0, 1.0 - median_dist / footprint_cross_m))
    return round(overlap * 100 / 5) * 5


def estimate_overlap_from_gps(
    photo_records: List[Dict],
    cam_params: Dict[str, float],
    avg_altitude_m: float,
) -> Tuple[Optional[float], Optional[float]]:
    """Estimate heading and side overlap from GPS positions and camera parameters."""
    valid = [r for r in photo_records
             if r.get("longitude") is not None
             and r.get("latitude") is not None
             and isinstance(r.get("capture_time"), datetime)]
    if len(valid) < 3:
        return None, None

    focal = cam_params["focal_length_mm"]
    # sensor_height (短边) -> along-track footprint, sensor_width (长边) -> cross-track footprint
    footprint_along = camera_footprint_m(avg_altitude_m, cam_params["sensor_height_mm"], focal)
    footprint_cross = camera_footprint_m(avg_altitude_m, cam_params["sensor_width_mm"], focal)

    segments = segment_flight_lines(valid)

    # Heading overlap: along-track distances within segments
    along_track_dists: List[float] = []
    for seg in segments:
        for j in range(len(seg) - 1):
            a, b = valid[seg[j]], valid[seg[j + 1]]
            d = haversine_m(a["longitude"], a["latitude"],
                            b["longitude"], b["latitude"])
            along_track_dists.append(d)
    heading_overlap = compute_heading_overlap(along_track_dists, footprint_along)

    # Side overlap: filter out turn segments (too few photos), then compute
    flight_segments = filter_flight_segments(segments)
    ref_lon = valid[0]["longitude"]
    ref_lat = valid[0]["latitude"]
    side_overlap = compute_side_overlap(flight_segments, valid, footprint_cross, ref_lon, ref_lat)

    return heading_overlap, side_overlap


def estimate_gsd_from_camera_params(
    avg_altitude_m: Optional[float],
    cam_params: Dict[str, float],
) -> Optional[float]:
    """Compute GSD in cm using YAML camera parameters (no EXIF focal_35mm needed)."""
    if not avg_altitude_m or not cam_params:
        return None
    focal = cam_params["focal_length_mm"]
    sw = cam_params["sensor_width_mm"]
    iw = cam_params["image_width_px"]
    return ((avg_altitude_m * sw) / (focal * iw)) * 100.0


def kmz_text(kmz_path: Path, inner_name: str) -> Optional[str]:
    try:
        with zipfile.ZipFile(kmz_path) as zf:
            return zf.read(inner_name).decode("utf-8")
    except Exception:
        return None


def find_first_text(root: ET.Element, names: Sequence[str]) -> Optional[str]:
    for name in names:
        value = root.findtext(f".//wpml:{name}", namespaces=WPML_NS)
        if value:
            return value
    return None


def parse_kmz_metadata(folder: Path, files: Optional[List[Path]] = None) -> Dict[str, object]:
    metadata = {
        "drone_model": None,
        "route_type": None,
        "heading_overlap_pct": None,
        "side_overlap_pct": None,
        "area_km2": None,
    }
    # 如果已提供文件列表，从中查找KMZ文件，避免重复rglob
    if files is not None:
        kmz_files = [f for f in files if f.suffix.lower() == ".kmz"]
        kmz_path = kmz_files[0] if kmz_files else None
    else:
        kmz_path = next(iter(sorted(folder.rglob("*.kmz"))), None)
    if kmz_path is None:
        return metadata
    template_xml = kmz_text(kmz_path, "wpmz/template.kml")
    if not template_xml:
        return metadata
    try:
        root = ET.fromstring(template_xml)
    except ET.ParseError:
        return metadata

    smart_oblique = find_first_text(root, ("smartObliqueEnable",))
    if smart_oblique == "1":
        metadata["route_type"] = "倾斜"
    elif smart_oblique == "0":
        metadata["route_type"] = "正射"

    heading = find_first_text(root, ("orthoCameraOverlapH",))
    side = find_first_text(root, ("orthoCameraOverlapW",))
    def safe_float(text: Optional[str]) -> Optional[float]:
        if text is None:
            return None
        try:
            return float(str(text).strip())
        except ValueError:
            return None
    metadata["heading_overlap_pct"] = safe_float(heading)
    metadata["side_overlap_pct"] = safe_float(side)

    drone_enum = find_first_text(root, ("droneEnumValue",))
    drone_sub_enum = find_first_text(root, ("droneSubEnumValue", "droneSubType", "droneSubEnum"))
    if drone_enum:
        if drone_sub_enum is not None:
            metadata["drone_model"] = DRONE_ENUM_MAP.get(
                (drone_enum, drone_sub_enum),
                f"DJI droneEnumValue={drone_enum}, droneSubEnumValue={drone_sub_enum}",
            )
        else:
            metadata["drone_model"] = DRONE_TYPE_FALLBACK_MAP.get(
                drone_enum,
                f"DJI droneEnumValue={drone_enum}",
            )

    coords_text = root.findtext(".//kml:Polygon//kml:coordinates", namespaces=WPML_NS)
    if coords_text:
        coords = []
        for item in coords_text.strip().split():
            parts = item.split(",")
            if len(parts) >= 2:
                try:
                    coords.append((float(parts[0]), float(parts[1])))
                except ValueError:
                    pass
        if coords and coords[0] == coords[-1]:
            coords = coords[:-1]
        metadata["area_km2"] = polygon_area_km2(coords)
    return metadata


def fallback_date_from_folder(folder_name: str) -> str:
    match = re.search(r"(20\d{6})", folder_name)
    if not match:
        return ""
    try:
        return datetime.strptime(match.group(1), "%Y%m%d").strftime("%Y-%m-%d")
    except ValueError:
        return ""


def summarize_folder(folder: Path, camera_params: Optional[Dict] = None) -> Dict[str, str]:
    files = sorted(list(iter_files(folder)))
    payload_type = classify_payload(files)
    photos = primary_photo_files(files, payload_type)

    photo_records = [parse_photo(p, read_sensor=(idx == 0)) for idx, p in enumerate(photos)]
    kmz_meta = parse_kmz_metadata(folder, files)

    times = [r["capture_time"] for r in photo_records if isinstance(r["capture_time"], datetime)]
    longitudes = [float(r["longitude"]) for r in photo_records if r["longitude"] is not None]
    latitudes = [float(r["latitude"]) for r in photo_records if r["latitude"] is not None]
    altitudes = [float(r["gps_altitude"]) for r in photo_records if r["gps_altitude"] is not None]
    sensors = [str(r["sensor"]) for r in photo_records if r.get("sensor")]

    start_time = min(times) if times else None
    end_time = max(times) if times else None
    duration_min = None
    if start_time and end_time:
        duration_min = (end_time - start_time).total_seconds() / 60.0

    avg_altitude = average(altitudes)
    focal_mm = None
    focal_35mm = None
    image_width_px = None
    for record in photo_records:
        fm = record.get("focal_mm")
        f35 = record.get("focal_35mm")
        iw = record.get("image_width")
        if fm and f35 and iw:
            focal_mm = fm
            focal_35mm = f35
            image_width_px = iw
            break

    sensor_model = Counter(sensors).most_common(1)[0][0] if sensors else ""
    drone_model = str(kmz_meta.get("drone_model") or sensor_model)
    point_area_km2 = estimate_area_from_points_km2(list(zip(longitudes, latitudes)))
    area_km2 = kmz_meta.get("area_km2") if kmz_meta.get("area_km2") is not None else point_area_km2

    # --- GSD: EXIF first, then YAML fallback ---
    gsd_cm = estimate_gsd_cm(avg_altitude, focal_mm, focal_35mm, image_width_px)
    cam_params = None
    if camera_params and sensor_model in camera_params:
        cam_params = camera_params[sensor_model]
    if gsd_cm is None and cam_params is not None:
        gsd_cm = estimate_gsd_from_camera_params(avg_altitude, cam_params)

    # --- Overlap: KMZ first, then GPS-based fallback ---
    heading_overlap = kmz_meta.get("heading_overlap_pct")
    side_overlap = kmz_meta.get("side_overlap_pct")
    if heading_overlap is None and side_overlap is None and cam_params is not None and avg_altitude:
        gps_heading, gps_side = estimate_overlap_from_gps(photo_records, cam_params, avg_altitude)
        if heading_overlap is None:
            heading_overlap = gps_heading
        if side_overlap is None:
            side_overlap = gps_side

    return {
        "数据文件夹": folder.name,
        "观测日期": start_time.strftime("%Y-%m-%d") if start_time else fallback_date_from_folder(folder.name),
        "观测开始时间": start_time.strftime("%Y%m%d %H%M%S") if start_time else "",
        "观测结束时间": end_time.strftime("%Y%m%d %H%M%S") if end_time else "",
        "操作员": OPERATOR_NAME,
        "飞行目的": FLIGHT_PURPOSE,
        "无人机/相机型号": drone_model,
        "载荷类型": payload_type,
        "航线类型": str(kmz_meta.get("route_type") or ""),
        "测区中心经度": round_text(average(longitudes), 6),
        "测区中心纬度": round_text(average(latitudes), 6),
        "测区覆盖面积(km2)": round_text(area_km2, 4),
        "观测时长(min)": round_text(duration_min, 2),
        "影像数量(张/组)": str(count_image_groups(files, payload_type)),
        "平均航高(m)": round_text(avg_altitude, 2),
        "航向重叠率(%)": round_text(heading_overlap, 0),
        "旁向重叠率(%)": round_text(side_overlap, 0),
        "地面分辨率(cm)": round_text(gsd_cm, 2),
    }


def summarize_folder_safe(
    folder: Path,
    camera_params: Optional[Dict] = None,
) -> Tuple[str, Optional[Dict[str, str]], Optional[str]]:
    """Worker wrapper used by multiprocessing. Keeps one bad folder from stopping all results."""
    try:
        return folder.name, summarize_folder(folder, camera_params), None
    except Exception as exc:
        return folder.name, None, f"{type(exc).__name__}: {exc}"


def iter_with_progress(
    folders: Sequence[Path],
    workers: int,
    use_progress: bool,
    camera_params: Optional[Dict] = None,
):
    """Yield summary rows. Progress advances when a folder finishes, not per image."""
    total = len(folders)
    progress = tqdm(total=total, desc="Summarizing UAV folders", unit="folder") if use_progress and tqdm is not None else None

    def update(folder_name: str) -> None:
        if progress is not None:
            progress.set_postfix_str(folder_name[:40])
            progress.update(1)

    try:
        if workers <= 1 or total <= 1:
            for folder in folders:
                name, row, error = summarize_folder_safe(folder, camera_params)
                update(name)
                yield name, row, error
            return

        with ProcessPoolExecutor(max_workers=workers) as executor:
            future_to_folder = {
                executor.submit(summarize_folder_safe, folder, camera_params): folder
                for folder in folders
            }
            for future in as_completed(future_to_folder):
                folder = future_to_folder[future]
                try:
                    name, row, error = future.result()
                except Exception as exc:
                    name, row, error = folder.name, None, f"{type(exc).__name__}: {exc}"
                update(name)
                yield name, row, error
    finally:
        if progress is not None:
            progress.close()


def write_csv(rows: Sequence[Dict[str, str]], output_csv: Path) -> None:
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    with output_csv.open("w", newline="", encoding="utf-8-sig") as handle:
        writer = csv.DictWriter(handle, fieldnames=HEADERS)
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    args = parse_args()
    data_root = Path(args.data_root)
    output_csv = Path(args.output_csv)
    if not data_root.exists() or not data_root.is_dir():
        raise FileNotFoundError(f"data-root not found or not a folder: {data_root}")

    valid_folders = list_candidate_folders(data_root, args.recursive)
    camera_params = load_camera_params(Path(args.camera_params))
    print(f"Found {len(valid_folders)} candidate flight folders.")
    print(f"Workers: {args.workers}")
    if camera_params:
        print(f"Camera models loaded: {', '.join(sorted(camera_params.keys()))}")

    rows = []
    errors = []
    for name, row, error in iter_with_progress(
        valid_folders,
        workers=max(1, args.workers),
        use_progress=not args.no_progress,
        camera_params=camera_params,
    ):
        if row is not None:
            rows.append(row)
        else:
            errors.append((name, error or "unknown error"))

    rows.sort(key=lambda row: row["数据文件夹"])
    write_csv(rows, output_csv)
    print(f"Wrote {len(rows)} rows to {output_csv}")
    if errors:
        print(f"Skipped {len(errors)} folders due to errors:")
        for name, error in errors[:20]:
            print(f"  - {name}: {error}")
        if len(errors) > 20:
            print(f"  ... and {len(errors) - 20} more")


if __name__ == "__main__":
    main()
