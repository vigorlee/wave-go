#!/usr/bin/env python3
"""Import a ZIP containing one LAZ as the project's RGB + intensity PCD.

The archive is never modified. This script expects numpy, laspy and a LAZ
backend such as lazrs (``python3 -m pip install numpy laspy lazrs``).

Convert to a new path::

    python3 scripts/import_laz_rgb_intensity.py scan.zip --output scan.pcd

Verify every point in an existing PCD without writing to it::

    python3 scripts/import_laz_rgb_intensity.py scan.zip --verify-pcd scan.pcd

The packed RGB value uses the high byte of each LAS 16-bit color channel, as
in the existing Wavego import. XYZ is converted to float32, intensity stays
uint16, and the PCD body follows imported_923_io.PCD_DTYPE exactly.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import tempfile
import zipfile

try:
    import laspy
    import numpy as np
except ImportError as exc:
    raise SystemExit(f"Missing dependency {exc.name!r}; install numpy, laspy and lazrs") from exc

from imported_923_io import PCD_DTYPE


PCD_FIELDS = ("x", "y", "z", "rgb", "intensity")
PCD_SIZES = (4, 4, 4, 4, 2)
PCD_TYPES = ("F", "F", "F", "U", "U")


def select_member(archive: zipfile.ZipFile) -> zipfile.ZipInfo:
    members = [m for m in archive.infolist() if not m.is_dir() and m.filename.lower().endswith(".laz")]
    if len(members) != 1:
        raise ValueError(f"Expected exactly one LAZ file in ZIP; found {len(members)}")
    return members[0]


def copy_laz_and_check_crc(zip_path: Path, destination: Path) -> dict:
    zip_hash = hashlib.sha256()
    with zip_path.open("rb") as stream:
        for block in iter(lambda: stream.read(8 * 1024 * 1024), b""):
            zip_hash.update(block)
    with zipfile.ZipFile(zip_path) as archive:
        member = select_member(archive)
        member_hash = hashlib.sha256()
        copied = 0
        # ZipExtFile checks the member's CRC when the last byte is read.
        with archive.open(member, "r") as source, destination.open("wb") as target:
            for block in iter(lambda: source.read(8 * 1024 * 1024), b""):
                copied += len(block)
                member_hash.update(block)
                target.write(block)
        if copied != member.file_size:
            raise ValueError(f"LAZ member size mismatch: {copied} != {member.file_size}")
        return {
            "source_zip": str(zip_path),
            "source_zip_sha256": zip_hash.hexdigest(),
            "laz_member": member.filename,
            "laz_member_sha256": member_hash.hexdigest(),
            "laz_member_crc32": f"{member.CRC:08x}",
            "zip_crc_checked": True,
        }


def header_bytes(count: int) -> bytes:
    return (
        "# .PCD v0.7 - Point Cloud Data file format\n"
        "VERSION .7\n"
        "FIELDS x y z rgb intensity\n"
        "SIZE 4 4 4 4 2\n"
        "TYPE F F F U U\n"
        "COUNT 1 1 1 1 1\n"
        f"WIDTH {count}\n"
        "HEIGHT 1\n"
        "VIEWPOINT 0 0 0 1 0 0 0\n"
        f"POINTS {count}\n"
        "DATA binary\n"
    ).encode("ascii")


def existing_pcd_payload(path: Path, expected_count: int):
    stream = path.open("rb")
    try:
        lines = {}
        while True:
            line = stream.readline()
            if not line or len(line) > 512:
                raise ValueError("PCD has no valid DATA binary header")
            words = line.decode("ascii").split()
            if words:
                lines[words[0]] = words[1:]
            if words[:2] == ["DATA", "binary"]:
                break
        if tuple(lines.get("FIELDS", ())) != PCD_FIELDS or tuple(map(int, lines.get("SIZE", ()))) != PCD_SIZES:
            raise ValueError("PCD fields or field sizes differ from Wavego RGB/intensity layout")
        if tuple(map(int, lines.get("COUNT", ()))) != (1, 1, 1, 1, 1):
            raise ValueError("PCD field counts differ from Wavego RGB/intensity layout")
        types = tuple(lines.get("TYPE", ()))
        if types[:3] != PCD_TYPES[:3] or types[3] not in ("F", "U") or types[4] != "U":
            raise ValueError("PCD field types differ from Wavego RGB/intensity layout")
        count = int(lines["POINTS"][0])
        width = int(lines["WIDTH"][0])
        if count != expected_count or width != expected_count:
            raise ValueError(f"PCD count {count} differs from LAZ count {expected_count}")
        payload_offset = stream.tell()
        expected_bytes = payload_offset + count * PCD_DTYPE.itemsize
        if path.stat().st_size != expected_bytes:
            raise ValueError(f"PCD size {path.stat().st_size} differs from expected {expected_bytes}")
        return stream
    except Exception:
        stream.close()
        raise


def points_as_pcd(chunk) -> np.ndarray:
    record = np.empty(len(chunk), dtype=PCD_DTYPE)
    for field in "xyz":
        values = np.asarray(getattr(chunk, field))
        if not np.all(np.isfinite(values)):
            raise ValueError(f"LAZ has non-finite {field} coordinates")
        record[field] = values
    red = np.asarray(chunk.red, dtype=np.uint32) >> 8
    green = np.asarray(chunk.green, dtype=np.uint32) >> 8
    blue = np.asarray(chunk.blue, dtype=np.uint32) >> 8
    record["rgb"] = (red << 16) | (green << 8) | blue
    record["intensity"] = np.asarray(chunk.intensity, dtype=np.uint16)
    return record


def import_or_verify(zip_path: Path, output: Path | None, verify_pcd: Path | None, chunk_size: int) -> dict:
    if not zip_path.is_file():
        raise FileNotFoundError(zip_path)
    if output is not None and output.exists():
        raise FileExistsError(f"Output exists; refusing to overwrite: {output}")
    if verify_pcd is not None and not verify_pcd.is_file():
        raise FileNotFoundError(verify_pcd)
    if PCD_DTYPE.itemsize != 18 or PCD_DTYPE.names != PCD_FIELDS:
        raise RuntimeError("Existing imported_923_io.PCD_DTYPE has changed; review PCD layout")

    with tempfile.TemporaryDirectory(prefix="wavego_laz_import_") as temporary:
        laz_path = Path(temporary) / "source.laz"
        result = copy_laz_and_check_crc(zip_path, laz_path)
        with laspy.open(laz_path) as reader:
            las_header = reader.header
            dimensions = set(las_header.point_format.dimension_names)
            if not {"red", "green", "blue", "intensity"}.issubset(dimensions):
                raise ValueError("LAZ lacks RGB or intensity fields")
            result.update({
                "las_version": str(las_header.version),
                "las_point_format": las_header.point_format.id,
                "points": las_header.point_count,
                "min_xyz": las_header.mins.tolist(),
                "max_xyz": las_header.maxs.tolist(),
                "scale_xyz": las_header.scales.tolist(),
                "pcd_fields": list(PCD_FIELDS),
                "pcd_point_stride_bytes": PCD_DTYPE.itemsize,
                "coordinate_transform_applied": False,
                "rgb_mapping": "red, green, blue high 8 bits packed into uint32",
            })
            partial_path = None
            if output is not None:
                output.parent.mkdir(parents=True, exist_ok=True)
                fd, name = tempfile.mkstemp(prefix=output.name + ".", suffix=".partial", dir=output.parent)
                partial_path = Path(name)
                stream = os.fdopen(fd, "wb")
            else:
                stream = existing_pcd_payload(verify_pcd, las_header.point_count)
            payload_hash = hashlib.sha256()
            processed = 0
            completed = False
            try:
                if output is not None:
                    stream.write(header_bytes(las_header.point_count))
                for chunk in reader.chunk_iterator(chunk_size):
                    payload = points_as_pcd(chunk).tobytes()
                    payload_hash.update(payload)
                    if output is not None:
                        stream.write(payload)
                    else:
                        current = stream.read(len(payload))
                        if current != payload:
                            # The PCD body has a fixed 18-byte stride.
                            mismatch = next((i for i, (a, b) in enumerate(zip(current, payload)) if a != b), min(len(current), len(payload)))
                            raise ValueError(f"PCD data differs at point {processed + mismatch // PCD_DTYPE.itemsize}")
                    processed += len(chunk)
                if processed != las_header.point_count:
                    raise ValueError(f"LAZ yielded {processed} points; header declares {las_header.point_count}")
                if output is not None:
                    stream.flush()
                    os.fsync(stream.fileno())
                completed = True
            finally:
                stream.close()
                if not completed and partial_path is not None:
                    partial_path.unlink(missing_ok=True)
            if output is not None:
                try:
                    expected_size = len(header_bytes(processed)) + processed * PCD_DTYPE.itemsize
                    if partial_path.stat().st_size != expected_size:
                        raise ValueError("Incomplete PCD output")
                    # Hard-linking creates the final name atomically and never replaces it.
                    os.link(partial_path, output)
                finally:
                    partial_path.unlink(missing_ok=True)
                result["output_pcd"] = str(output)
            else:
                result["verified_pcd"] = str(verify_pcd)
                result["full_payload_match"] = True
            result["pcd_payload_sha256"] = payload_hash.hexdigest()
            return result


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("zip", type=Path, help="ZIP archive containing exactly one LAZ member")
    target = parser.add_mutually_exclusive_group(required=True)
    target.add_argument("--output", type=Path, help="write a new RGB/intensity PCD; never overwrites")
    target.add_argument("--verify-pcd", type=Path, help="compare all points to an existing PCD, read-only")
    parser.add_argument("--chunk-size", type=int, default=1_000_000)
    args = parser.parse_args()
    if args.chunk_size < 1:
        parser.error("--chunk-size must be positive")
    try:
        result = import_or_verify(args.zip.resolve(), args.output.resolve() if args.output else None,
                                  args.verify_pcd.resolve() if args.verify_pcd else None, args.chunk_size)
    except (ValueError, FileNotFoundError, FileExistsError, zipfile.BadZipFile) as exc:
        parser.exit(2, f"Import failed: {exc}\n")
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
