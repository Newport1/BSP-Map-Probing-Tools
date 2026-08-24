#!/usr/bin/env python3
"""
tf2_bsp_triage.py

Small, reusable Source/TF2 BSP geometry triage script inspired by SeamshotCalculator.

It is intended for map-author QA and responsible bug-fix investigation:
- parses core VBSP geometry lumps locally
- classifies simple vs complex solid brushes
- finds classic seamshot-style simple/complex brush-edge contacts
- finds tiny vertical under-gaps between world brushes
- writes JSON, CSV, TF2 drawline CFG, HTML report, and a ZIP bundle
- optionally asks a local Ollama model for a short human-readable triage note

No required third-party dependencies. Optional:
- pandas: nicer CSV writing
- matplotlib: top-down PNG plot with --plot
- local Ollama server: --ollama-model llama3.2 or similar

Usage:
  python3 tf2_bsp_triage.py /path/to/koth_example_map.bsp --out reports/koth_example_map --plot
  python3 tf2_bsp_triage.py /path/to/map.bsp --ollama-model llama3.2

Limitations:
- This is a geometry triage tool, not proof of an in-game exploit.
- It does not emulate TF2 weapon traces/projectiles/splash.
- It does not fully resolve every entity/tool material/prop collision case.
"""

from __future__ import annotations

import argparse
import csv
import html
import itertools
import json
import lzma
import math
import os
import re
import struct
import sys
import textwrap
import urllib.error
import urllib.request
import zipfile
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Set, Tuple

# Source BSP lump IDs used here.
LUMP_ENTITIES = 0
LUMP_PLANES = 1
LUMP_NODES = 5
LUMP_LEAFS = 10
LUMP_MODELS = 14
LUMP_LEAFBRUSHES = 17
LUMP_BRUSHES = 18
LUMP_BRUSHSIDES = 19

CONTENTS_SOLID = 0x1

EPS = 1e-4
POINT_EPS = 1e-3
AXIS_EPS = 1e-4

Vec3 = Tuple[float, float, float]
Bounds = Tuple[float, float, float, float, float, float]


@dataclass
class Lump:
    offset: int
    length: int
    version: int
    fourcc: bytes


@dataclass
class Plane:
    normal: Vec3
    dist: float
    type: int


@dataclass
class BrushSide:
    planenum: int
    texinfo: int
    dispinfo: int
    bevel: int


@dataclass
class Brush:
    index: int
    firstside: int
    numsides: int
    contents: int
    sides: List[int]
    planes: List[Plane]
    vertices: List[Vec3]
    edges: List[Tuple[Vec3, Vec3, Tuple[int, int]]]
    bounds: Optional[Bounds]
    is_solid: bool
    is_world: bool
    model_indices: List[int]
    is_axis_aligned_box: bool
    is_complex: bool
    leafs: List[int]


@dataclass
class Node:
    planenum: int
    children: Tuple[int, int]
    mins: Tuple[int, int, int]
    maxs: Tuple[int, int, int]
    firstface: int
    numfaces: int
    area: int


@dataclass
class Leaf:
    contents: int
    cluster: int
    area_flags: int
    mins: Tuple[int, int, int]
    maxs: Tuple[int, int, int]
    firstleafface: int
    numleaffaces: int
    firstleafbrush: int
    numleafbrushes: int
    leafwaterdataid: int


@dataclass
class Model:
    index: int
    mins: Vec3
    maxs: Vec3
    origin: Vec3
    headnode: int
    firstface: int
    numfaces: int


@dataclass
class SeamCandidate:
    id: str
    type: str
    brush: int
    other_brush: int
    edge: List[List[float]]
    midpoint: List[float]
    length_units: float
    brush_complex: bool
    other_complex: bool
    severity: str
    reason: str


@dataclass
class GapCandidate:
    id: str
    brush: int
    support_brush: int
    gap_units: float
    gap_z_min: float
    gap_z_max: float
    xy_bounds: List[float]
    brush_bounds: List[float]
    severity: str
    reason: str


@dataclass
class RawContact:
    brush: int
    other_brush: int
    type: str
    midpoint: List[float]
    length_units: float


class BspParseError(RuntimeError):
    pass


def f3(x: float) -> float:
    """Stable compact float for JSON/CSV."""
    if abs(x) < 0.0005:
        x = 0.0
    return round(float(x), 3)


def dot(a: Vec3, b: Vec3) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def sub(a: Vec3, b: Vec3) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def length(a: Vec3) -> float:
    return math.sqrt(dot(a, a))


def midpoint(a: Vec3, b: Vec3) -> Vec3:
    return ((a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0, (a[2] + b[2]) / 2.0)


def determinant3(m: Sequence[Sequence[float]]) -> float:
    return (
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
        - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
        + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    )


def solve3(normals: Sequence[Vec3], dists: Sequence[float]) -> Optional[Vec3]:
    """Solve normals * x = dists by Cramer's rule."""
    a = [[float(v) for v in row] for row in normals]
    det = determinant3(a)
    if abs(det) < 1e-7:
        return None
    mats = []
    for col in range(3):
        m = [row[:] for row in a]
        for r in range(3):
            m[r][col] = float(dists[r])
        mats.append(m)
    return (determinant3(mats[0]) / det, determinant3(mats[1]) / det, determinant3(mats[2]) / det)


def quantize_point(p: Vec3, eps: float = POINT_EPS) -> Tuple[int, int, int]:
    return (round(p[0] / eps), round(p[1] / eps), round(p[2] / eps))


def dedupe_points(points: Iterable[Vec3]) -> List[Vec3]:
    out: Dict[Tuple[int, int, int], Vec3] = {}
    for p in points:
        out.setdefault(quantize_point(p), (f3(p[0]), f3(p[1]), f3(p[2])))
    return list(out.values())


def bbox(points: Sequence[Vec3]) -> Optional[Bounds]:
    if not points:
        return None
    xs = [p[0] for p in points]
    ys = [p[1] for p in points]
    zs = [p[2] for p in points]
    return (min(xs), min(ys), min(zs), max(xs), max(ys), max(zs))


def point_in_brush(p: Vec3, planes: Sequence[Plane], eps: float = 0.01) -> bool:
    # Source brush side plane normals point outward. For valid brush points, n dot p <= dist.
    return all(dot(pl.normal, p) <= pl.dist + eps for pl in planes)


def point_on_plane(p: Vec3, pl: Plane, eps: float = 0.03) -> bool:
    return abs(dot(pl.normal, p) - pl.dist) <= eps


def is_axis_aligned_normal(n: Vec3) -> bool:
    ax = [abs(v) for v in n]
    return max(ax) > 1.0 - AXIS_EPS and sum(1 for v in ax if v > AXIS_EPS) == 1


def is_axis_aligned_box(side_planes: Sequence[Plane], vertices: Sequence[Vec3]) -> bool:
    if len(side_planes) != 6 or not vertices:
        return False
    if not all(is_axis_aligned_normal(pl.normal) for pl in side_planes):
        return False
    b = bbox(vertices)
    if b is None:
        return False
    # A rectangular box should have 8 unique corners.
    return len(vertices) == 8


def xy_overlap(a: Bounds, b: Bounds, eps: float = 0.01) -> Optional[Tuple[float, float, float, float]]:
    xmin = max(a[0], b[0])
    ymin = max(a[1], b[1])
    xmax = min(a[3], b[3])
    ymax = min(a[4], b[4])
    if xmax - xmin > eps and ymax - ymin > eps:
        return (xmin, ymin, xmax, ymax)
    return None


def read_lumps(data: bytes) -> Tuple[str, int, List[Lump], int]:
    if len(data) < 8 + 64 * 16 + 4:
        raise BspParseError("File is too small to be a VBSP file.")
    ident = data[:4].decode("ascii", errors="replace")
    if ident != "VBSP":
        raise BspParseError(f"Not a VBSP file; header ident={ident!r}.")
    version = struct.unpack_from("<i", data, 4)[0]
    lumps: List[Lump] = []
    off = 8
    for _ in range(64):
        lo, ln, lv, fourcc = struct.unpack_from("<iii4s", data, off)
        lumps.append(Lump(lo, ln, lv, fourcc))
        off += 16
    map_revision = struct.unpack_from("<i", data, off)[0]
    return ident, version, lumps, map_revision


def _decompress_source_lzma(buf: bytes) -> bytes:
    """Decompress a Source-engine LZMA lump (17-byte custom header + raw LZMA1)."""
    if len(buf) < 17:
        raise BspParseError("Compressed lump too short for LZMA header.")
    header_id, actual_size, lzma_size = struct.unpack_from("<III", buf, 0)
    if header_id != 0x414D5A4C:  # little-endian "LZMA"
        raise BspParseError(f"Unexpected LZMA id {header_id:#x}")
    props = buf[12:17]
    if len(buf) < 17 + lzma_size:
        raise BspParseError(
            f"Compressed lump declares {lzma_size} bytes of LZMA data, "
            f"but only {len(buf) - 17} are present."
        )
    compressed = buf[17 : 17 + lzma_size]
    # Decode lc/lp/pb and dict_size from the 5-byte properties (Valve style).
    d = props[0]
    if d >= 9 * 5 * 5:
        raise BspParseError("Invalid LZMA properties byte.")
    lc = d % 9
    d //= 9
    pb = d // 5
    lp = d % 5
    dict_size = struct.unpack_from("<I", props, 1)[0]
    filters = [{"id": lzma.FILTER_LZMA1, "dict_size": dict_size, "lc": lc, "lp": lp, "pb": pb}]
    decompressor = lzma.LZMADecompressor(format=lzma.FORMAT_RAW, filters=filters)
    out = decompressor.decompress(compressed, max_length=actual_size)
    if len(out) != actual_size:
        raise BspParseError(
            f"LZMA lump declares an uncompressed size of {actual_size} bytes, "
            f"but decompression produced {len(out)} bytes."
        )
    return out


def lump_bytes(data: bytes, lumps: Sequence[Lump], idx: int) -> bytes:
    lump = lumps[idx]
    raw = data[lump.offset : lump.offset + lump.length]
    # Non-zero fourcc usually means the lump is LZMA-compressed (fourcc holds uncompressed size).
    if lump.fourcc != b"\x00\x00\x00\x00" and len(raw) >= 17 and raw[:4] == b"LZMA":
        return _decompress_source_lzma(raw)
    return raw


def parse_planes(buf: bytes) -> List[Plane]:
    if len(buf) % 20 != 0:
        raise BspParseError(f"Planes lump length {len(buf)} is not divisible by 20.")
    out = []
    for i in range(0, len(buf), 20):
        nx, ny, nz, dist, typ = struct.unpack_from("<ffffi", buf, i)
        out.append(Plane((nx, ny, nz), dist, typ))
    return out


def parse_brushsides(buf: bytes) -> List[BrushSide]:
    if len(buf) % 8 != 0:
        raise BspParseError(f"BrushSides lump length {len(buf)} is not divisible by 8.")
    out = []
    for i in range(0, len(buf), 8):
        planenum, texinfo, dispinfo, bevel = struct.unpack_from("<Hhhh", buf, i)
        out.append(BrushSide(planenum, texinfo, dispinfo, bevel))
    return out


def parse_brush_records(buf: bytes) -> List[Tuple[int, int, int]]:
    if len(buf) % 12 != 0:
        raise BspParseError(f"Brushes lump length {len(buf)} is not divisible by 12.")
    out = []
    for i in range(0, len(buf), 12):
        out.append(struct.unpack_from("<iii", buf, i))
    return out


def parse_nodes(buf: bytes) -> List[Node]:
    if len(buf) % 32 != 0:
        raise BspParseError(f"Nodes lump length {len(buf)} is not divisible by 32.")
    out: List[Node] = []
    for i in range(0, len(buf), 32):
        vals = struct.unpack_from("<iii hhh hhh HHhh", buf, i)
        out.append(
            Node(
                planenum=vals[0],
                children=(vals[1], vals[2]),
                mins=(vals[3], vals[4], vals[5]),
                maxs=(vals[6], vals[7], vals[8]),
                firstface=vals[9],
                numfaces=vals[10],
                area=vals[11],
            )
        )
    return out


def parse_leafs(buf: bytes) -> List[Leaf]:
    # Source 2004 dleaf_t is 32 bytes. This covers TF2 VBSP v20 maps.
    if len(buf) % 32 != 0:
        raise BspParseError(f"Leafs lump length {len(buf)} is not divisible by 32; this parser targets Source/TF2 leaf v0.")
    out: List[Leaf] = []
    for i in range(0, len(buf), 32):
        vals = struct.unpack_from("<ihh hhh hhh HHHHh", buf, i)
        out.append(
            Leaf(
                contents=vals[0],
                cluster=vals[1],
                area_flags=vals[2],
                mins=(vals[3], vals[4], vals[5]),
                maxs=(vals[6], vals[7], vals[8]),
                firstleafface=vals[9],
                numleaffaces=vals[10],
                firstleafbrush=vals[11],
                numleafbrushes=vals[12],
                leafwaterdataid=vals[13],
            )
        )
    return out


def parse_leafbrushes(buf: bytes) -> List[int]:
    if len(buf) % 2 != 0:
        raise BspParseError(f"LeafBrushes lump length {len(buf)} is not divisible by 2.")
    return list(struct.unpack_from("<" + "H" * (len(buf) // 2), buf, 0)) if buf else []


def parse_models(buf: bytes) -> List[Model]:
    if len(buf) % 48 != 0:
        raise BspParseError(f"Models lump length {len(buf)} is not divisible by 48.")
    out: List[Model] = []
    for idx, i in enumerate(range(0, len(buf), 48)):
        vals = struct.unpack_from("<fff fff fff i ii", buf, i)
        out.append(
            Model(
                index=idx,
                mins=(vals[0], vals[1], vals[2]),
                maxs=(vals[3], vals[4], vals[5]),
                origin=(vals[6], vals[7], vals[8]),
                headnode=vals[9],
                firstface=vals[10],
                numfaces=vals[11],
            )
        )
    return out


def parse_entities(buf: bytes) -> List[Dict[str, str]]:
    text = buf.split(b"\x00", 1)[0].decode("latin-1", errors="replace")
    entities: List[Dict[str, str]] = []
    for block in re.findall(r"\{([^{}]*)\}", text, flags=re.S):
        ent: Dict[str, str] = {}
        for key, val in re.findall(r'"([^"]*)"\s*"([^"]*)"', block):
            ent[key] = val
        if ent:
            entities.append(ent)
    return entities


def entity_class_counts(entities: Sequence[Dict[str, str]]) -> Dict[str, int]:
    out: Dict[str, int] = {}
    for ent in entities:
        cls = ent.get("classname", "<unknown>")
        out[cls] = out.get(cls, 0) + 1
    return dict(sorted(out.items()))


def collect_model_leafs(model: Model, nodes: Sequence[Node], leafs: Sequence[Leaf]) -> Set[int]:
    seen_nodes: Set[int] = set()
    out: Set[int] = set()

    def walk(child: int) -> None:
        if child < 0:
            li = -1 - child
            if 0 <= li < len(leafs):
                out.add(li)
            return
        if child in seen_nodes or not (0 <= child < len(nodes)):
            return
        seen_nodes.add(child)
        node = nodes[child]
        walk(node.children[0])
        walk(node.children[1])

    walk(model.headnode)
    return out


def leaf_brush_set(leaf_indices: Iterable[int], leafs: Sequence[Leaf], leafbrushes: Sequence[int]) -> Set[int]:
    out: Set[int] = set()
    for li in leaf_indices:
        if not (0 <= li < len(leafs)):
            continue
        leaf = leafs[li]
        for j in range(leaf.firstleafbrush, leaf.firstleafbrush + leaf.numleafbrushes):
            if 0 <= j < len(leafbrushes):
                out.add(int(leafbrushes[j]))
    return out


def reconstruct_brush_vertices(side_planes: Sequence[Plane]) -> List[Vec3]:
    points: List[Vec3] = []
    for a, b, c in itertools.combinations(range(len(side_planes)), 3):
        planes3 = [side_planes[a], side_planes[b], side_planes[c]]
        p = solve3([pl.normal for pl in planes3], [pl.dist for pl in planes3])
        if p is None:
            continue
        if point_in_brush(p, side_planes):
            points.append(p)
    return dedupe_points(points)


def reconstruct_brush_edges(vertices: Sequence[Vec3], side_planes: Sequence[Plane]) -> List[Tuple[Vec3, Vec3, Tuple[int, int]]]:
    edges: Dict[Tuple[Tuple[int, int, int], Tuple[int, int, int]], Tuple[Vec3, Vec3, Tuple[int, int]]] = {}
    if not vertices:
        return []
    for i, j in itertools.combinations(range(len(side_planes)), 2):
        pts = [p for p in vertices if point_on_plane(p, side_planes[i]) and point_on_plane(p, side_planes[j])]
        pts = dedupe_points(pts)
        if len(pts) == 2 and length(sub(pts[0], pts[1])) > 0.05:
            k0, k1 = quantize_point(pts[0]), quantize_point(pts[1])
            key = (k0, k1) if k0 <= k1 else (k1, k0)
            edges[key] = (pts[0], pts[1], (i, j))
        elif len(pts) > 2:
            # A real edge has two endpoints. If numeric noise leaves collinear extras, take the farthest pair.
            best: Optional[Tuple[float, Vec3, Vec3]] = None
            for p, q in itertools.combinations(pts, 2):
                d = length(sub(p, q))
                if best is None or d > best[0]:
                    best = (d, p, q)
            if best and best[0] > 0.05:
                k0, k1 = quantize_point(best[1]), quantize_point(best[2])
                key = (k0, k1) if k0 <= k1 else (k1, k0)
                edges[key] = (best[1], best[2], (i, j))
    return list(edges.values())


def build_brushes(
    brush_records: Sequence[Tuple[int, int, int]],
    brushsides: Sequence[BrushSide],
    planes: Sequence[Plane],
    world_brush_indices: Set[int],
    brush_model_map: Dict[int, Set[int]],
    brush_leaf_map: Dict[int, Set[int]],
) -> List[Brush]:
    brushes: List[Brush] = []
    for idx, (firstside, numsides, contents) in enumerate(brush_records):
        side_indices = list(range(firstside, firstside + numsides))
        side_indices = [s for s in side_indices if 0 <= s < len(brushsides)]
        side_planes = [planes[brushsides[s].planenum] for s in side_indices if 0 <= brushsides[s].planenum < len(planes)]
        vertices = reconstruct_brush_vertices(side_planes)
        edges = reconstruct_brush_edges(vertices, side_planes)
        b = bbox(vertices)
        axis_box = is_axis_aligned_box(side_planes, vertices)
        is_solid = bool(contents & CONTENTS_SOLID)
        model_indices = sorted(brush_model_map.get(idx, set()))
        leaf_indices = sorted(brush_leaf_map.get(idx, set()))
        brushes.append(
            Brush(
                index=idx,
                firstside=firstside,
                numsides=numsides,
                contents=contents,
                sides=side_indices,
                planes=side_planes,
                vertices=vertices,
                edges=edges,
                bounds=b,
                is_solid=is_solid,
                is_world=idx in world_brush_indices,
                model_indices=model_indices,
                is_axis_aligned_box=axis_box,
                is_complex=not axis_box,
                leafs=leaf_indices,
            )
        )
    return brushes


def seam_type(a: Brush, b: Brush) -> Optional[str]:
    if a.is_axis_aligned_box and b.is_axis_aligned_box:
        return None
    if a.is_axis_aligned_box != b.is_axis_aligned_box:
        return "simple_complex"
    if a.is_complex and b.is_complex:
        # Complex-complex contacts are much noisier. Prefer ones with disjoint leaf usage.
        if set(a.leafs).isdisjoint(b.leafs):
            return "complex_complex_disjoint_leafs"
        return "complex_complex_same_leafs"
    return None


def find_seam_candidates(brushes: Sequence[Brush], include_entities: bool = False, max_candidates: int = 5000) -> Tuple[List[SeamCandidate], List[RawContact]]:
    candidates: List[SeamCandidate] = []
    raw_contacts: List[RawContact] = []
    considered = [b for b in brushes if b.is_solid and b.bounds is not None and (include_entities or b.is_world)]

    seen: Set[Tuple[int, int, Tuple[int, int, int], Tuple[int, int, int]]] = set()
    raw_seen: Set[Tuple[int, int, Tuple[int, int, int], Tuple[int, int, int]]] = set()

    for a in considered:
        for e0, e1, _edge_planes in a.edges:
            if length(sub(e0, e1)) < 1.0:
                continue
            for b in considered:
                if a.index == b.index or b.bounds is None:
                    continue
                # Fast reject by bbox proximity.
                eb = bbox([e0, e1])
                assert eb is not None
                if (
                    eb[3] < b.bounds[0] - 0.5
                    or eb[0] > b.bounds[3] + 0.5
                    or eb[4] < b.bounds[1] - 0.5
                    or eb[1] > b.bounds[4] + 0.5
                    or eb[5] < b.bounds[2] - 0.5
                    or eb[2] > b.bounds[5] + 0.5
                ):
                    continue
                face_planes = b.planes if b.planes else bbox_face_planes(b.bounds)
                on_face = any(point_on_plane(e0, pl, 0.03) and point_on_plane(e1, pl, 0.03) for pl in face_planes)
                if not on_face:
                    continue
                key_edge = (quantize_point(e0), quantize_point(e1))
                key_edge = key_edge if key_edge[0] <= key_edge[1] else (key_edge[1], key_edge[0])
                key = (min(a.index, b.index), max(a.index, b.index), key_edge[0], key_edge[1])
                stype = seam_type(a, b)
                m = midpoint(e0, e1)
                dist = f3(length(sub(e0, e1)))
                if stype is None:
                    if key not in raw_seen:
                        raw_seen.add(key)
                        raw_contacts.append(
                            RawContact(
                                brush=a.index,
                                other_brush=b.index,
                                type="simple_simple_excluded",
                                midpoint=[f3(v) for v in m],
                                length_units=dist,
                            )
                        )
                    continue
                if key in seen:
                    continue
                seen.add(key)
                sev = "high" if stype == "simple_complex" else "medium"
                candidates.append(
                    SeamCandidate(
                        id=f"seam_{len(candidates)+1:03d}",
                        type=stype,
                        brush=a.index,
                        other_brush=b.index,
                        edge=[[f3(v) for v in e0], [f3(v) for v in e1]],
                        midpoint=[f3(v) for v in m],
                        length_units=dist,
                        brush_complex=a.is_complex,
                        other_complex=b.is_complex,
                        severity=sev,
                        reason=(
                            "Brush edge lies on another brush face; includes complex geometry, so this is a classic seamshot-style candidate."
                            if stype == "simple_complex"
                            else "Complex-complex brush contact. Lower confidence; validate in-game."
                        ),
                    )
                )
                if len(candidates) >= max_candidates:
                    return candidates, raw_contacts
    return candidates, raw_contacts


def bbox_face_planes(b: Bounds) -> List[Plane]:
    xmin, ymin, zmin, xmax, ymax, zmax = b
    return [
        Plane((-1.0, 0.0, 0.0), -xmin, 0),
        Plane((1.0, 0.0, 0.0), xmax, 0),
        Plane((0.0, -1.0, 0.0), -ymin, 1),
        Plane((0.0, 1.0, 0.0), ymax, 1),
        Plane((0.0, 0.0, -1.0), -zmin, 2),
        Plane((0.0, 0.0, 1.0), zmax, 2),
    ]


def find_gap_candidates(brushes: Sequence[Brush], max_gap: float, include_entities: bool = False) -> List[GapCandidate]:
    world = [b for b in brushes if b.is_solid and b.bounds is not None and (include_entities or b.is_world)]
    out: List[GapCandidate] = []
    for a in world:
        assert a.bounds is not None
        best: Optional[Tuple[float, Brush, Tuple[float, float, float, float]]] = None
        for b in world:
            if a.index == b.index or b.bounds is None:
                continue
            overlap = xy_overlap(a.bounds, b.bounds)
            if not overlap:
                continue
            gap = a.bounds[2] - b.bounds[5]
            if EPS < gap <= max_gap:
                if best is None or gap < best[0]:
                    best = (gap, b, overlap)
        if best:
            gap, support, overlap = best
            sev = "medium" if gap <= 1.0 else "low"
            out.append(
                GapCandidate(
                    id=f"gap_{len(out)+1:03d}",
                    brush=a.index,
                    support_brush=support.index,
                    gap_units=f3(gap),
                    gap_z_min=f3(support.bounds[5] if support.bounds else 0.0),
                    gap_z_max=f3(a.bounds[2]),
                    xy_bounds=[f3(v) for v in overlap],
                    brush_bounds=[f3(v) for v in a.bounds],
                    severity=sev,
                    reason=(
                        f"Structural world brush bottom is {f3(gap)}u above the supporting floor; "
                        "possible under-gap/splash LOS artifact, not a classic seamshot."
                    ),
                )
            )
    return out


def call_ollama(model: str, analysis_brief: Dict[str, Any], timeout: int = 45) -> Optional[str]:
    prompt = textwrap.dedent(
        f"""
        You are assisting responsible Team Fortress 2 map QA. Summarize this local BSP geometry triage in 3-6 concise bullets.
        Emphasize that candidates require local in-game validation and that the goal is submitting fixes, not abuse.

        Analysis JSON summary:
        {json.dumps(analysis_brief, indent=2)[:12000]}
        """
    ).strip()
    payload = json.dumps({"model": model, "prompt": prompt, "stream": False}).encode("utf-8")
    req = urllib.request.Request("http://127.0.0.1:11434/api/generate", data=payload, headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as resp:
            obj = json.loads(resp.read().decode("utf-8", errors="replace"))
            return str(obj.get("response", "")).strip() or None
    except (urllib.error.URLError, TimeoutError, json.JSONDecodeError, OSError) as exc:
        return f"Ollama note unavailable: {exc}"


def write_json(path: Path, obj: Dict[str, Any]) -> None:
    path.write_text(json.dumps(obj, indent=2), encoding="utf-8")


def write_gap_csv(path: Path, gaps: Sequence[GapCandidate]) -> None:
    rows = []
    for g in gaps:
        d = asdict(g)
        d["xy_bounds"] = " ".join(str(v) for v in g.xy_bounds)
        d["brush_bounds"] = " ".join(str(v) for v in g.brush_bounds)
        rows.append(d)

    try:
        import pandas as pd  # type: ignore

        pd.DataFrame(rows).to_csv(path, index=False)
    except Exception:
        with path.open("w", newline="", encoding="utf-8") as f:
            fieldnames = ["id", "brush", "support_brush", "gap_units", "gap_z_min", "gap_z_max", "xy_bounds", "brush_bounds", "severity", "reason"]
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            for row in rows:
                writer.writerow(row)


def write_cfg(path: Path, stem: str, seams: Sequence[SeamCandidate], gaps: Sequence[GapCandidate]) -> None:
    lines: List[str] = []
    lines.append(f"// {stem} seam/gap triage cfg")
    lines.append("// Generated by tf2_bsp_triage.py. Use locally for responsible map QA.")
    lines.append(f"// Classic seamshot candidates: {len(seams)}. Under-gap candidates: {len(gaps)}.")
    lines.append(f"// Run locally with: sv_cheats 1; developer 1; exec {path.name}")
    lines.append("")

    for s in seams:
        p0, p1 = s.edge
        color = (255, 0, 0) if s.type == "simple_complex" else (255, 96, 0)
        lines.append(f"// {s.id}: {s.type}; brush {s.brush} against brush {s.other_brush}; length {s.length_units}u")
        lines.append(
            "drawline "
            f"{p0[0]:.3f} {p0[1]:.3f} {p0[2]:.3f} {p1[0]:.3f} {p1[1]:.3f} {p1[2]:.3f} "
            f"{color[0]} {color[1]} {color[2]}"
        )
        lines.append("")

    for g in gaps:
        x0, y0, x1, y1 = g.xy_bounds
        z = (g.gap_z_min + g.gap_z_max) / 2.0
        lines.append(f"// {g.id}: brush {g.brush} floats {g.gap_units}u above brush {g.support_brush}; z {g.gap_z_min}..{g.gap_z_max}")
        lines.append(f"drawline {x0:.3f} {y0:.3f} {z:.3f} {x1:.3f} {y0:.3f} {z:.3f} 255 192 0")
        lines.append(f"drawline {x1:.3f} {y0:.3f} {z:.3f} {x1:.3f} {y1:.3f} {z:.3f} 255 192 0")
        lines.append(f"drawline {x1:.3f} {y1:.3f} {z:.3f} {x0:.3f} {y1:.3f} {z:.3f} 255 192 0")
        lines.append(f"drawline {x0:.3f} {y1:.3f} {z:.3f} {x0:.3f} {y0:.3f} {z:.3f} 255 192 0")
        lines.append("")

    path.write_text("\n".join(lines), encoding="utf-8")


def make_topdown_plot(path: Path, brushes: Sequence[Brush], gaps: Sequence[GapCandidate], seams: Sequence[SeamCandidate]) -> Optional[str]:
    try:
        import matplotlib.pyplot as plt  # type: ignore
        from matplotlib.patches import Rectangle  # type: ignore
    except Exception:
        return None

    fig, ax = plt.subplots(figsize=(9, 9))
    plotted = 0
    for b in brushes:
        if not b.is_solid or not b.is_world or b.bounds is None:
            continue
        x0, y0, _z0, x1, y1, _z1 = b.bounds
        if x1 <= x0 or y1 <= y0:
            continue
        rect = Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, linewidth=0.5, alpha=0.35)
        ax.add_patch(rect)
        plotted += 1
    for g in gaps:
        x0, y0, x1, y1 = g.xy_bounds
        rect = Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, linewidth=2.0)
        ax.add_patch(rect)
    for s in seams:
        p0, p1 = s.edge
        ax.plot([p0[0], p1[0]], [p0[1], p1[1]], linewidth=2.0)
    if plotted == 0:
        plt.close(fig)
        return None
    ax.set_title("TF2 BSP triage top-down view")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_aspect("equal", adjustable="box")
    ax.autoscale_view()
    fig.tight_layout()
    fig.savefig(path, dpi=160)
    plt.close(fig)
    return str(path)


def write_html_report(path: Path, analysis: Dict[str, Any], plot_name: Optional[str] = None) -> None:
    counts = analysis["counts"]
    gaps = analysis.get("thin_air_gap_candidates", [])
    seams = analysis.get("classic_seamshot_candidates", [])
    raw = analysis.get("raw_simple_simple_contacts_excluded", [])
    ollama_note = analysis.get("ollama_note")

    def esc(x: Any) -> str:
        return html.escape(str(x))

    gap_rows = "".join(
        f"<tr><td>{esc(g['id'])}</td><td>{g['brush']}</td><td>{g['support_brush']}</td><td>{g['gap_units']}</td>"
        f"<td>{esc(g['xy_bounds'])}</td><td>{esc(g['severity'])}</td><td>{esc(g['reason'])}</td></tr>"
        for g in gaps
    ) or "<tr><td colspan='7'>No under-gap candidates found.</td></tr>"

    seam_rows = "".join(
        f"<tr><td>{esc(s['id'])}</td><td>{esc(s['type'])}</td><td>{s['brush']}</td><td>{s['other_brush']}</td>"
        f"<td>{s['length_units']}</td><td>{esc(s['midpoint'])}</td><td>{esc(s['reason'])}</td></tr>"
        for s in seams
    ) or "<tr><td colspan='7'>No classic seamshot-style candidates found.</td></tr>"

    raw_rows = "".join(
        f"<tr><td>{r['brush']}</td><td>{r['other_brush']}</td><td>{r['length_units']}</td><td>{esc(r['midpoint'])}</td></tr>"
        for r in raw[:100]
    ) or "<tr><td colspan='4'>No excluded simple-simple contacts recorded.</td></tr>"

    plot_html = f"<figure><img src='{esc(plot_name)}' alt='Top-down plot'><figcaption>Optional top-down brush/gap plot.</figcaption></figure>" if plot_name else ""
    note_html = f"<section><h2>Ollama triage note</h2><pre>{esc(ollama_note)}</pre></section>" if ollama_note else ""

    body = f"""<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>{esc(analysis['file'])} BSP seam/gap triage</title>
<style>
  :root {{ color-scheme: dark; --bg:#111; --panel:#181818; --ink:#e8e0cf; --muted:#aaa; --line:#333; --accent:#d2a84a; --danger:#ff5a4f; }}
  * {{ box-sizing:border-box; }}
  body {{ margin:0; font-family: ui-monospace, SFMono-Regular, Menlo, Consolas, monospace; background:var(--bg); color:var(--ink); line-height:1.45; }}
  main {{ max-width:1180px; margin:0 auto; padding:28px; }}
  h1,h2 {{ font-weight:700; letter-spacing:-0.03em; }}
  h1 {{ font-size:clamp(28px,5vw,52px); margin:0 0 8px; }}
  h2 {{ margin-top:34px; border-top:1px solid var(--line); padding-top:22px; }}
  .muted {{ color:var(--muted); }}
  .grid {{ display:grid; grid-template-columns:repeat(auto-fit,minmax(180px,1fr)); gap:12px; margin:22px 0; }}
  .card {{ background:var(--panel); border:1px solid var(--line); padding:14px; }}
  .label {{ color:var(--muted); font-size:12px; text-transform:uppercase; letter-spacing:0.08em; }}
  .value {{ font-size:24px; color:var(--accent); }}
  table {{ width:100%; border-collapse:collapse; background:var(--panel); border:1px solid var(--line); margin:12px 0 24px; }}
  th,td {{ border:1px solid var(--line); padding:8px; vertical-align:top; font-size:13px; }}
  th {{ text-align:left; color:var(--accent); }}
  pre {{ white-space:pre-wrap; background:#0b0b0b; border:1px solid var(--line); padding:14px; }}
  img {{ max-width:100%; border:1px solid var(--line); background:#000; }}
  code {{ color:var(--accent); }}
</style>
</head>
<body><main>
<h1>BSP seam/gap triage</h1>
<p class="muted">File: <code>{esc(analysis['file'])}</code>. This is a geometry triage report for local map-author QA, not proof of an in-game exploit.</p>
<div class="grid">
  <div class="card"><div class="label">VBSP version</div><div class="value">{counts['bsp_version']}</div></div>
  <div class="card"><div class="label">Map revision</div><div class="value">{counts['map_revision']}</div></div>
  <div class="card"><div class="label">Brushes total</div><div class="value">{counts['brushes_total']}</div></div>
  <div class="card"><div class="label">World brushes</div><div class="value">{counts['world_brushes']}</div></div>
  <div class="card"><div class="label">Complex brushes</div><div class="value">{counts['complex_brushes']}</div></div>
  <div class="card"><div class="label">Classic candidates</div><div class="value">{len(seams)}</div></div>
  <div class="card"><div class="label">Under-gap candidates</div><div class="value">{len(gaps)}</div></div>
</div>
{plot_html}
{note_html}
<section>
<h2>Classic seamshot-style candidates</h2>
<table><thead><tr><th>ID</th><th>Type</th><th>Brush</th><th>Other</th><th>Length</th><th>Midpoint</th><th>Reason</th></tr></thead><tbody>{seam_rows}</tbody></table>
</section>
<section>
<h2>Thin-air / under-gap candidates</h2>
<p class="muted">Small vertical gaps between a brush bottom and a supporting brush top can be relevant to splash/trace oddities even when they are not classic seamshots.</p>
<table><thead><tr><th>ID</th><th>Brush</th><th>Support</th><th>Gap</th><th>XY bounds</th><th>Severity</th><th>Reason</th></tr></thead><tbody>{gap_rows}</tbody></table>
</section>
<section>
<h2>Excluded simple-simple contacts</h2>
<p class="muted">These are brush-edge contacts intentionally excluded from classic seamshot scoring because both brushes are simple axis-aligned boxes. First 100 shown.</p>
<table><thead><tr><th>Brush</th><th>Other</th><th>Length</th><th>Midpoint</th></tr></thead><tbody>{raw_rows}</tbody></table>
</section>
<section>
<h2>Suggested local validation</h2>
<pre>sv_cheats 1
developer 1
map {esc(Path(analysis['file']).stem)}
exec {esc(Path(analysis['outputs']['cfg']).name)}</pre>
</section>
</main></body></html>
"""
    path.write_text(body, encoding="utf-8")


def zip_outputs(zip_path: Path, files: Sequence[Path]) -> None:
    with zipfile.ZipFile(zip_path, "w", compression=zipfile.ZIP_DEFLATED) as zf:
        for f in files:
            if f.exists():
                zf.write(f, arcname=f.name)


def analyze_bsp(args: argparse.Namespace) -> Dict[str, Any]:
    bsp_path = Path(args.bsp).resolve()
    if not bsp_path.exists():
        raise SystemExit(f"BSP not found: {bsp_path}")
    data = bsp_path.read_bytes()
    ident, version, lumps, map_revision = read_lumps(data)

    entities = parse_entities(lump_bytes(data, lumps, LUMP_ENTITIES))
    planes = parse_planes(lump_bytes(data, lumps, LUMP_PLANES))
    nodes = parse_nodes(lump_bytes(data, lumps, LUMP_NODES))
    leafs = parse_leafs(lump_bytes(data, lumps, LUMP_LEAFS))
    models = parse_models(lump_bytes(data, lumps, LUMP_MODELS))
    leafbrushes = parse_leafbrushes(lump_bytes(data, lumps, LUMP_LEAFBRUSHES))
    brushsides = parse_brushsides(lump_bytes(data, lumps, LUMP_BRUSHSIDES))
    brush_records = parse_brush_records(lump_bytes(data, lumps, LUMP_BRUSHES))

    model_leafs: List[Set[int]] = [collect_model_leafs(m, nodes, leafs) for m in models]
    model_brushes: List[Set[int]] = [leaf_brush_set(ls, leafs, leafbrushes) for ls in model_leafs]
    world_brush_indices: Set[int] = set(model_brushes[0]) if model_brushes else set()
    brush_model_map: Dict[int, Set[int]] = {}
    brush_leaf_map: Dict[int, Set[int]] = {}
    for mi, brs in enumerate(model_brushes):
        for bi in brs:
            brush_model_map.setdefault(bi, set()).add(mi)
    for li, leaf in enumerate(leafs):
        for j in range(leaf.firstleafbrush, leaf.firstleafbrush + leaf.numleafbrushes):
            if 0 <= j < len(leafbrushes):
                brush_leaf_map.setdefault(int(leafbrushes[j]), set()).add(li)

    brushes = build_brushes(brush_records, brushsides, planes, world_brush_indices, brush_model_map, brush_leaf_map)
    seams, raw_contacts = find_seam_candidates(brushes, include_entities=args.include_entity_brushes, max_candidates=args.max_seams)
    gaps = find_gap_candidates(brushes, max_gap=args.max_gap, include_entities=args.include_entity_brushes)

    out_dir = Path(args.out).resolve() if args.out else Path.cwd() / f"{bsp_path.stem}_analysis"
    out_dir.mkdir(parents=True, exist_ok=True)
    json_path = out_dir / f"{bsp_path.stem}_analysis.json"
    csv_path = out_dir / f"{bsp_path.stem}_gap_candidates.csv"
    cfg_path = out_dir / f"{bsp_path.stem}_gap_triage.cfg"
    html_path = out_dir / f"{bsp_path.stem}_report.html"
    plot_path = out_dir / f"{bsp_path.stem}_topdown.png"
    zip_path = out_dir.parent / f"{bsp_path.stem}_analysis_bundle.zip"

    counts = {
        "bsp_ident": ident,
        "bsp_version": version,
        "map_revision": map_revision,
        "entities": len(entities),
        "planes": len(planes),
        "nodes": len(nodes),
        "leaves": len(leafs),
        "leafbrushes": len(leafbrushes),
        "brushes_total": len(brushes),
        "brushsides": len(brushsides),
        "models": len(models),
        "world_brushes": sum(1 for b in brushes if b.is_world),
        "brush_entity_hulls": sum(1 for b in brushes if not b.is_world),
        "solid_brushes": sum(1 for b in brushes if b.is_solid),
        "simple_axis_aligned_brushes": sum(1 for b in brushes if b.is_axis_aligned_box),
        "complex_brushes": sum(1 for b in brushes if b.is_complex),
        "classic_seamshot_candidates": len(seams),
        "raw_simple_simple_contacts_excluded": len(raw_contacts),
        "thin_air_gap_candidates": len(gaps),
    }

    brief = {
        "file": bsp_path.name,
        "counts": counts,
        "top_gap_candidates": [asdict(g) for g in gaps[:20]],
        "top_classic_candidates": [asdict(s) for s in seams[:20]],
    }
    ollama_note = None
    if args.ollama_model:
        ollama_note = call_ollama(args.ollama_model, brief)

    analysis: Dict[str, Any] = {
        "file": bsp_path.name,
        "file_size_bytes": len(data),
        "counts": counts,
        "worldspawn": next((e for e in entities if e.get("classname") == "worldspawn"), {}),
        "entity_class_counts": entity_class_counts(entities),
        "classic_seamshot_assessment": {
            "summary": (
                "No classic SeamshotCalculator-style candidates were found."
                if not seams
                else f"Found {len(seams)} classic seamshot-style candidates requiring in-game validation."
            ),
            "method": "Brush edge/face contact triage, prioritizing simple-complex and complex-complex contacts.",
            "caveat": "This script does not emulate TF2 traces/projectiles/splash and cannot prove exploitability by itself.",
        },
        "classic_seamshot_candidates": [asdict(s) for s in seams],
        "thin_air_gap_candidates": [asdict(g) for g in gaps],
        "raw_simple_simple_contacts_excluded": [asdict(r) for r in raw_contacts],
        "brushes": [
            {
                "index": b.index,
                "contents": b.contents,
                "firstside": b.firstside,
                "numsides": b.numsides,
                "is_solid": b.is_solid,
                "is_world": b.is_world,
                "model_indices": b.model_indices,
                "leaf_count": len(b.leafs),
                "bounds": [f3(v) for v in b.bounds] if b.bounds else None,
                "vertex_count": len(b.vertices),
                "edge_count": len(b.edges),
                "is_axis_aligned_box": b.is_axis_aligned_box,
                "is_complex": b.is_complex,
            }
            for b in brushes
        ],
        "outputs": {
            "json": str(json_path),
            "csv": str(csv_path),
            "cfg": str(cfg_path),
            "html": str(html_path),
            "zip": str(zip_path),
        },
    }
    if ollama_note:
        analysis["ollama_note"] = ollama_note

    plot_name = None
    if args.plot:
        plot_made = make_topdown_plot(plot_path, brushes, gaps, seams)
        if plot_made:
            analysis["outputs"]["plot"] = str(plot_path)
            plot_name = plot_path.name

    write_json(json_path, analysis)
    write_gap_csv(csv_path, gaps)
    write_cfg(cfg_path, bsp_path.stem, seams, gaps)
    write_html_report(html_path, analysis, plot_name=plot_name)

    bundle_files = [json_path, csv_path, cfg_path, html_path]
    if args.plot and plot_path.exists():
        bundle_files.append(plot_path)
    if args.bundle:
        zip_outputs(zip_path, bundle_files)

    return analysis


def main(argv: Optional[Sequence[str]] = None) -> int:
    parser = argparse.ArgumentParser(description="TF2/Source BSP seamshot-style and under-gap triage.")
    parser.add_argument("bsp", help="Path to a .bsp file")
    parser.add_argument("--out", help="Output directory. Default: ./<map>_analysis")
    parser.add_argument("--max-gap", type=float, default=1.0, help="Maximum vertical brush under-gap to report, in Hammer units. Default: 1.0")
    parser.add_argument("--include-entity-brushes", action="store_true", help="Include non-world brush hulls/triggers in seam/gap triage.")
    parser.add_argument("--max-seams", type=int, default=5000, help="Safety cap for seam candidates. Default: 5000")
    parser.add_argument("--plot", action="store_true", help="Also write a matplotlib top-down PNG if matplotlib is installed.")
    parser.add_argument("--no-bundle", dest="bundle", action="store_false", help="Do not write the ZIP bundle.")
    parser.set_defaults(bundle=True)
    parser.add_argument("--ollama-model", help="Optional local Ollama model name, e.g. llama3.2, qwen2.5-coder:7b, mistral.")
    args = parser.parse_args(argv)

    try:
        analysis = analyze_bsp(args)
    except BspParseError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    counts = analysis["counts"]
    print(f"Analyzed {analysis['file']}")
    print(f"  VBSP version: {counts['bsp_version']} revision {counts['map_revision']}")
    print(f"  Brushes: {counts['brushes_total']} total / {counts['world_brushes']} world / {counts['complex_brushes']} complex")
    print(f"  Classic seamshot-style candidates: {counts['classic_seamshot_candidates']}")
    print(f"  Thin-air under-gap candidates: {counts['thin_air_gap_candidates']}")
    print("Outputs:")
    for key, val in analysis["outputs"].items():
        print(f"  {key}: {val}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
