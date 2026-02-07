import os, re
from collections import namedtuple
import math
import numpy as np
from scipy import ndimage
from scipy.spatial import ConvexHull, Delaunay
from typing import Iterable, List, Sequence, Tuple, Dict, Any, Optional
try:
    import cv2
except:
    cv2 = None
    pass


# ----------------- GCODE PARSING ----------------------------------------------------------------------

GCODE_EPS = 1e-4

RE_PARAMS = re.compile(r"([A-Z])([-+]?(?:\d*\.\d+|\d+))")
RE_Z = re.compile(r"Z([-+]?(?:\d*\.\d+|\d+))")
Point = namedtuple('Point', 'x y z e')


def _next_pos(cur: Point, params: Dict[str, float], abs_pos: bool, abs_ext: bool) -> Point:
    """
    cur=Point, params=dict, abs_pos=bool, abs_ext=bool  →  Point
    Computes next position from params and current state.
    """
    nx = (params['X'] if abs_pos else cur.x + params['X']) if 'X' in params else cur.x
    ny = (params['Y'] if abs_pos else cur.y + params['Y']) if 'Y' in params else cur.y
    nz = (params['Z'] if abs_pos else cur.z + params['Z']) if 'Z' in params else cur.z

    ne = cur.e
    if 'E' in params:
        if abs_ext:
            ne = params['E']
        else:
            ne += params['E']

    return Point(nx, ny, nz, ne)


def _sweep_delta(a0: float, a1: float, cw: bool) -> float:
    """
    a0=float, a1=float, cw=bool  →  float
    Returns positive sweep angle from a0 to a1 in cw/ccw direction.
    """
    if cw:
        d = a0 - a1
        if d <= 0:
            d += 2.0 * math.pi
    else:
        d = a1 - a0
        if d <= 0:
            d += 2.0 * math.pi
    return d


def _arc_center_from_r(cur: Point, target: Point, r: float, cw: bool) -> Optional[Tuple[float, float]]:
    """
    cur=Point, target=Point, r=float, cw=bool  →  (float,float)|None
    Resolves center from R.
    """
    dx = target.x - cur.x
    dy = target.y - cur.y
    chord = math.hypot(dx, dy)
    r_abs = abs(r)
    if chord <= 1e-9 or chord > 2.0 * r_abs + 1e-9:
        return None

    mx = (cur.x + target.x) * 0.5
    my = (cur.y + target.y) * 0.5
    h = math.sqrt(max(r_abs * r_abs - (chord * 0.5) ** 2, 0.0))
    px = -dy / chord
    py = dx / chord
    c1 = (mx + px * h, my + py * h)
    c2 = (mx - px * h, my - py * h)

    def _sweep(center):
        a0 = math.atan2(cur.y - center[1], cur.x - center[0])
        a1 = math.atan2(target.y - center[1], target.x - center[0])
        return _sweep_delta(a0, a1, cw)

    d1 = _sweep(c1)
    d2 = _sweep(c2)
    want_large = (r < 0.0)
    if (d1 > math.pi) == want_large and (d2 > math.pi) != want_large:
        return c1
    if (d2 > math.pi) == want_large and (d1 > math.pi) != want_large:
        return c2
    return c1 if d1 >= d2 else c2


def _arc_center(cur: Point, target: Point, params: Dict[str, float], cw: bool) -> Optional[Tuple[float, float]]:
    """
    cur=Point, target=Point, params=dict, cw=bool  →  (float,float)|None
    Resolves arc center from I/J or R.
    """
    if 'I' in params or 'J' in params:
        return (cur.x + params.get('I', 0.0), cur.y + params.get('J', 0.0))
    if 'R' in params:
        return _arc_center_from_r(cur, target, float(params['R']), cw)
    return None


def _iter_arc_points(cur: Point,
                     target: Point,
                     center: Optional[Tuple[float, float]],
                     cw: bool,
                     max_seg_len: float = 1.0,
                     max_angle_deg: float = 10.0) -> Iterable[Point]:
    """
    cur=Point, target=Point, center=(float,float)|None, cw=bool,
    max_seg_len=float, max_angle_deg=float  →  iterator(Point)
    Yields Points along an arc from cur to target (inclusive).
    """
    if center is None:
        yield target
        return

    cx, cy = center
    r = math.hypot(cur.x - cx, cur.y - cy)
    if r < 1e-9:
        yield target
        return

    a0 = math.atan2(cur.y - cy, cur.x - cx)
    a1 = math.atan2(target.y - cy, target.x - cx)
    delta = _sweep_delta(a0, a1, cw)
    direction = -1.0 if cw else 1.0

    arc_len = abs(delta) * r
    max_angle = math.radians(max_angle_deg)
    n_len = int(math.ceil(arc_len / max_seg_len)) if arc_len > 0 else 1
    n_ang = int(math.ceil(abs(delta) / max_angle)) if abs(delta) > 0 else 1
    n = max(1, n_len, n_ang)

    for i in range(1, n + 1):
        frac = i / n
        ang = a0 + direction * delta * frac
        x = cx + r * math.cos(ang)
        y = cy + r * math.sin(ang)
        z = cur.z + (target.z - cur.z) * frac
        e = cur.e + (target.e - cur.e) * frac
        yield Point(x, y, z, e)


def _ensure_path(path):
    """path=str  →  path=str|None
    Expands ~ and returns path only if it exists, else None.
    """
    p = os.path.expanduser(path)
    return p if os.path.exists(p) else None


def _parse_params(line):
    """line=str  →  dict{'X','Y','Z','E',...: float}
    Extracts G-code parameters before any comment.
    """
    return {k: float(v) for k, v in RE_PARAMS.findall(line.split(';', 1)[0])}


def _get_z_at_offset(f, offset):
    """f=file, offset=int(bytes)  →  float|None
    Returns first Z value found after the given byte offset.
    """
    f.seek(offset)
    f.readline()

    chunk = f.read(4096)
    if not chunk:
        return None

    match = RE_Z.search(chunk)
    if match:
        return float(match.group(1))
    return None


def _bisect_z_offset(path, target_z):
    """path=str, target_z=float  →  int(byte_offset_before_layer)
    Binary-search-ish scan to find an approximate offset before target_z.
    """
    size = os.path.getsize(path)
    low, high = 0, size
    best_offset = 0

    for _ in range(20):
        mid = (low + high) // 2
        if mid == low:
            break

        with open(path, 'r', errors='ignore') as f:
            z_val = _get_z_at_offset(f, mid)

        if z_val is None:
            high = mid
            continue

        if z_val < target_z:
            best_offset = mid
            low = mid
        else:
            high = mid

    # Move a bit earlier to ensure we don't skip the layer start
    return max(0, best_offset - 16384)


def _gcode_state(path, start_offset=0):
    """
    path=str, start_offset=int  →  iterator of (Point, Point)
    Streams consecutive motion states (last, curr) across the file.
    """
    path = _ensure_path(path)
    if not path:
        return
    cur = Point(0.0, 0.0, 0.0, 0.0)
    abs_pos, abs_ext = True, True

    with open(path, 'r', errors='ignore') as f:
        if start_offset > 0:
            f.seek(start_offset)
            f.readline()

        for line in f:
            line = line.lstrip()
            if len(line) < 2 or line.startswith(';'):
                continue

            char = line[0]
            if char == 'G':
                c2 = line[1]
                if c2 == '0' or c2 == '1':
                    p = _parse_params(line)
                    nxt = _next_pos(cur, p, abs_pos, abs_ext)
                    yield cur, nxt
                    cur = nxt

                elif c2 == '2' or c2 == '3':
                    # Arc move (G2/G3). Approximate with linear segments.
                    p = _parse_params(line)
                    cw = (c2 == '2')
                    tgt = _next_pos(cur, p, abs_pos, abs_ext)
                    center = _arc_center(cur, tgt, p, cw)
                    for pt in _iter_arc_points(cur, tgt, center, cw):
                        yield cur, pt
                        cur = pt

                elif c2 == '9':
                    c3 = line[2] if len(line) > 2 else ''
                    if c3 == '0':
                        abs_pos = True
                    elif c3 == '1':
                        abs_pos = False
                    elif c3 == '2':
                        p = _parse_params(line)
                        cur = Point(
                            p.get('X', cur.x),
                            p.get('Y', cur.y),
                            p.get('Z', cur.z),
                            p.get('E', cur.e)
                        )

            elif char == 'M':
                if len(line) > 2 and line[1] == '8':
                    c3 = line[2]
                    if c3 == '2':
                        abs_ext = True
                    elif c3 == '3':
                        abs_ext = False


def find_first_layer_z(path, threshold=1.0, z_min=0.005, scan_lim=0.6):
    """
    path=str, threshold=float(mm_extruded), z_min=float, scan_lim=float  →  float|None
    Scans up to scan_lim (while extruding) and returns the first Z with ≥ threshold extrusion.
    """
    z_totals = {}
    valid_layers = set()

    for last, curr in _gcode_state(path):
        if curr.z >= z_min:
            de = curr.e - last.e
            if de > 0 and (abs(curr.x - last.x) > GCODE_EPS or abs(curr.y - last.y) > GCODE_EPS):
                if curr.z > scan_lim:
                    break
                zr = round(curr.z, 3)
                new_tot = z_totals.get(zr, 0.0) + de
                z_totals[zr] = new_tot

                if new_tot >= threshold:
                    valid_layers.add(zr)
                    if len(valid_layers) >= 3:
                        break

    if not valid_layers:
        return None
    return sorted(valid_layers)[0]


def parse_gcode_data(path, z_req, tol=GCODE_EPS, max_layer_height=1.0):
    """
    path=str, z_req=float, tol=float, max_layer_height=float
      →  [ [ {'x','y','z','e'}, ... ], ... ]  (list of segments)
    Collects contiguous extruding segments around z_req within tol.
    """
    z_min, z_max = z_req - tol, z_req + tol
    z_exit = z_req + max_layer_height

    segments, cur_seg = [], []
    seen_layer = False

    start_offset = 0
    if z_req > 2.0:
        start_offset = _bisect_z_offset(path, z_req)

    for last, curr in _gcode_state(path, start_offset=start_offset):
        if z_min <= curr.z <= z_max:
            seen_layer = True
            if (curr.e - last.e) > 0:
                if not cur_seg:
                    cur_seg.append(last)
                cur_seg.append(curr)
            else:
                if cur_seg:
                    segments.append(cur_seg)
                    cur_seg = []
        else:
            if seen_layer and curr.z > z_exit:
                break
            if cur_seg:
                segments.append(cur_seg)
                cur_seg = []

    if cur_seg:
        segments.append(cur_seg)

    return [[pt._asdict() for pt in seg] for seg in segments]


# ----------------- BASIC POLYGON / POLYLINE UTILITIES -------------------------------------------------

EPS = 1e-9

Edge = Tuple[int, int]
Loop = List[int]


def ensure_open_np(poly):
    """poly_like  →  np.ndarray shape(N,2) (open)
    Returns an open polyline (last point != first point).
    """
    pts = np.array(poly, dtype=float)
    if len(pts) > 1 and np.allclose(pts[0], pts[-1]):
        return pts[:-1]
    return pts


def ensure_closed_np(poly):
    """poly_like  →  np.ndarray shape(N+1,2) (closed)
    Returns a closed polygon (last point == first point).
    """
    pts = np.array(poly, dtype=float)
    if len(pts) > 1 and not np.allclose(pts[0], pts[-1]):
        return np.vstack((pts, pts[0]))
    return pts


def ensure_open(poly):
    """poly_like  →  [[x,y], ...] (open)
    Python list wrapper around ensure_open_np.
    """
    return ensure_open_np(poly).tolist()


def ensure_closed(poly):
    """poly_like  →  [[x,y], ...] (closed)
    Python list wrapper around ensure_closed_np.
    """
    return ensure_closed_np(poly).tolist()


def fill_raster_holes(raster):
    """raster=np_like(bool|0/1)  →  np.ndarray(bool)
    Fills interior holes in a 2D mask.
    """
    if raster is None:
        return None
    raster = np.asarray(raster)
    if raster.size == 0:
        return raster
    return ndimage.binary_fill_holes(raster)


def point_in_polygon(poly, point):
    """poly=[[x,y],...], point=[x,y]  →  bool
    Ray-casting point-in-polygon test on the boundary of poly.
    """
    poly = np.asarray(poly, dtype=float)
    x, y = point
    p1 = poly
    p2 = np.roll(poly, -1, axis=0)
    mask = ((p1[:, 1] > y) != (p2[:, 1] > y)) & \
           (x < (p2[:, 0] - p1[:, 0]) * (y - p1[:, 1]) / (p2[:, 1] - p1[:, 1] + EPS) + p1[:, 0])
    return np.sum(mask) % 2 == 1


def get_polygon_properties(poly):
    """poly=[[x,y], ...]  →  {'area','perimeter','center':[x,y],'orientation':±1.0}
    Computes basic polygon properties; orientation >0 means CCW.
    """
    pts = ensure_open_np(poly)
    if len(pts) < 3:
        return {'area': 0.0, 'perimeter': 0.0, 'center': [0.0, 0.0], 'orientation': 1.0}

    x, y = pts[:, 0], pts[:, 1]
    signed_area = 0.5 * np.sum(x * np.roll(y, -1) - y * np.roll(x, -1))

    perim = np.sum(np.linalg.norm(pts - np.roll(pts, -1, axis=0), axis=1))
    cent = np.mean(pts, axis=0)

    return {
        'area': float(abs(signed_area)),
        'perimeter': float(perim),
        'center': cent.tolist(),
        'orientation': float(1.0 if signed_area >= 0 else -1.0),
    }


def get_bounding_box(poly, padding=0.0):
    """poly=[[x,y],...], padding=float  →  {'min_x','min_y','max_x','max_y'}
    Axis-aligned bounding box with optional padding.
    """
    pts = np.array(poly, dtype=float)
    if len(pts) == 0:
        return {}
    min_x, min_y = np.min(pts, axis=0) - padding
    max_x, max_y = np.max(pts, axis=0) + padding
    return {
        'min_x': float(min_x),
        'min_y': float(min_y),
        'max_x': float(max_x),
        'max_y': float(max_y),
    }


def get_closest_point_on_perimeter(poly, target_point):
    """
    poly=[[x,y],...], target_point=[x,y]
      →  {'point':[x,y], 'distance':float, 's':float, 'segment_index':int}
    Projects target_point onto the polygon perimeter, with arc-length s.
    """
    pts = np.array(poly, dtype=float)
    if len(pts) < 2:
        return None

    pts_closed = np.vstack((pts, pts[0]))
    vecs = np.diff(pts_closed, axis=0)

    p = np.array(target_point, dtype=float)
    ap = p - pts_closed[:-1]
    ab_sq = np.sum(vecs**2, axis=1)
    ab_sq[ab_sq < EPS] = 1.0

    t = np.sum(ap * vecs, axis=1) / ab_sq
    t = np.clip(t, 0.0, 1.0)

    projections = pts_closed[:-1] + vecs * t[:, np.newaxis]
    dists_sq = np.sum((projections - p)**2, axis=1)

    best_idx = int(np.argmin(dists_sq))
    seg_lens = np.sqrt(ab_sq)
    current_dist = float(np.sum(seg_lens[:best_idx]))
    s = current_dist + float(t[best_idx]) * float(seg_lens[best_idx])

    return {
        'point': projections[best_idx].tolist(),
        'distance': float(np.sqrt(dists_sq[best_idx])),
        's': float(s),
        'segment_index': best_idx,
    }


def get_polygon_distance_to_point(poly, point):
    """poly=[[x,y],...], point=[x,y]  →  float|None
    Returns shortest distance from point to polygon perimeter.
    """
    res = get_closest_point_on_perimeter(poly, point)
    return res['distance'] if res else None


# ----------------- POLYLINE SAMPLING, SIMPLIFICATION, SMOOTHING ---------------------------------------

def segment_poly(poly, resolution=0.5):
    """poly=[[x,y],...], resolution=float(mm)  →  [[x,y],...]
    Resamples edges so consecutive points are spaced by ≈ resolution.
    """
    if len(poly) < 2:
        return poly

    pts = np.array(poly, dtype=float)
    diffs = np.diff(pts, axis=0)
    dists = np.linalg.norm(diffs, axis=1)

    counts = np.ceil(dists / resolution).astype(int)
    counts = np.maximum(counts, 1)

    out = []
    for i in range(len(counts)):
        seg = np.linspace(pts[i], pts[i + 1], counts[i] + 1)
        if i < len(counts) - 1:
            out.append(seg[:-1])
        else:
            out.append(seg)
    return np.vstack(out).tolist()


def simplify_polygon(poly, tolerance=0.1):
    """poly=[[x,y],...], tolerance=float(mm)  →  [[x,y],...]
    Ramer–Douglas–Peucker-style simplification for open/closed polylines.
    """
    if len(poly) < 3:
        return poly

    pts = np.array(poly, dtype=float)
    keep = np.zeros(len(pts), dtype=bool)
    keep[0] = True
    keep[-1] = True
    stack = [(0, len(pts) - 1)]

    while stack:
        start, end = stack.pop()
        pt_start, pt_end = pts[start], pts[end]

        line_vec = pt_end - pt_start
        line_len_sq = float(np.dot(line_vec, line_vec))

        intermediates = pts[start + 1: end]
        if len(intermediates) == 0:
            continue

        w = intermediates - pt_start

        if line_len_sq < 1e-12:
            dists = np.linalg.norm(w, axis=1)
        else:
            cross = w[:, 0] * line_vec[1] - w[:, 1] * line_vec[0]
            dists = np.abs(cross) / math.sqrt(line_len_sq)

        dmax = float(np.max(dists))
        index_local = int(np.argmax(dists))
        index = int(start + 1 + index_local)

        if dmax > tolerance:
            keep[index] = True
            stack.append((start, index))
            stack.append((index, end))

    simplified_pts = pts[keep]
    return simplified_pts.tolist()


def chaikin_smooth(poly, iterations=1):
    """poly=[[x,y],...], iterations=int  →  [[x,y],...]
    Chaikin corner-cutting to smooth a polyline, then returns closed polygon.
    """
    if len(poly) < 3:
        return poly
    pts = ensure_open_np(poly)
    for _ in range(int(iterations)):
        p_next = np.roll(pts, -1, axis=0)
        Q = 0.75 * pts + 0.25 * p_next
        R = 0.25 * pts + 0.75 * p_next
        new_pts = np.empty((len(pts) * 2, 2))
        new_pts[0::2] = Q
        new_pts[1::2] = R
        pts = new_pts
    return ensure_closed(pts)


def rotate_polygon(poly, angle_deg, point=None):
    """poly=[[x,y],...], angle_deg=float, point=[x,y]|None  →  [[x,y],...]
    Rotates polygon around given point (or centroid if None).
    """
    pts = ensure_open_np(poly)
    if point is None:
        point = np.mean(pts, axis=0)

    theta = np.radians(angle_deg)
    c, s = np.cos(theta), np.sin(theta)
    R = np.array(((c, -s), (s, c)))

    centered = pts - point
    rotated = np.dot(centered, R.T) + point
    return ensure_closed_np(rotated).tolist()


# ----------------- CONVEX HULL & GEOMETRIC OFFSETS ----------------------------------------------------

def get_convex_hull(polygons_list):
    """polygons=[[[x,y],...], ...]  →  [[x,y],...]
    Flattens all points and returns closed convex hull polygon.
    """
    all_points = []
    try:
        arr = np.array(polygons_list, dtype=object)
        if arr.ndim == 2 and isinstance(arr[0][0], (int, float)):
            all_points = arr
        else:
            for p in polygons_list:
                all_points.extend(p)
    except Exception:
        return []

    pts = np.unique(np.array(all_points, dtype=float), axis=0)
    if len(pts) < 3:
        return ensure_closed(pts)
    try:
        hull = ConvexHull(pts)
        return ensure_closed(pts[hull.vertices])
    except Exception:
        return ensure_closed(pts)
    
    
def _edge_key(u: int, v: int) -> Edge:
    return (u, v) if u < v else (v, u)


def _boundary_loops_from_edges(
    points: np.ndarray,
    edges: Sequence[Edge],
    *,
    eps: float = 1e-12,
) -> List[Loop]:
    """points=np.ndarray shape(N,2), edges=[(i,j), ...]  →  [ [i0,i1,...,i0], ... ]
    Builds closed index loops from undirected boundary edges.
    """
    if not edges:
        return []

    # adjacency (small degree on boundary, so list is fine)
    adj: Dict[int, List[int]] = {}
    for a, b in edges:
        adj.setdefault(a, []).append(b)
        adj.setdefault(b, []).append(a)

    used: set[Edge] = set()
    loops: List[Loop] = []

    for a, b in edges:
        if _edge_key(a, b) in used:
            continue

        loop: Loop = [a, b]
        used.add(_edge_key(a, b))

        prev, curr = a, b

        while True:
            neigh = adj.get(curr)
            if not neigh:
                break

            # Prefer continuing forward: skip going back to prev unless it's the only option.
            cand = [
                n for n in neigh
                if not (_edge_key(curr, n) in used)
                and not (n == prev and len(neigh) > 1)
            ]

            if not cand:
                # if we can close, close
                if loop[0] in neigh:
                    loop.append(loop[0])
                break

            if len(cand) == 1 or prev is None:
                nxt = cand[0]
            else:
                # Choose candidate with maximum dot(v_prev, v_next) => smallest turn / closest to straight
                v_prev = points[curr] - points[prev]
                v_prev_len = float(np.linalg.norm(v_prev))
                if v_prev_len <= eps:
                    nxt = cand[0]
                else:
                    v_prev /= v_prev_len
                    vecs = points[np.asarray(cand, dtype=int)] - points[curr]
                    lens = np.linalg.norm(vecs, axis=1)

                    # Guard against degenerate zero-length segments.
                    valid = lens > eps
                    if not np.any(valid):
                        nxt = cand[0]
                    else:
                        vecs[valid] /= lens[valid, None]
                        dots = vecs @ v_prev
                        # invalid candidates get -inf so they never win
                        dots[~valid] = -np.inf
                        nxt = cand[int(np.argmax(dots))]

            used.add(_edge_key(curr, nxt))
            loop.append(nxt)

            prev, curr = curr, nxt
            if curr == loop[0]:
                break

        if len(loop) >= 4 and loop[0] == loop[-1]:
            loops.append(loop)

    return loops


def concave_hull(points: Sequence[Sequence[float]], alpha: float, *, eps: float = 1e-12):
    """points=[[x,y],...], alpha=float  →  [[x,y],...] (closed)
    Alpha-shape concave hull using Delaunay triangulation.
    Smaller alpha yields tighter (more concave) hulls.
    """
    pts = np.asarray(points, dtype=float)
    if pts.size == 0:
        return []

    pts = np.unique(pts, axis=0)
    if len(pts) < 4:
        return ensure_closed(pts)

    # Delaunay can fail on degenerate inputs (collinear, duplicate, etc).
    # If it fails, fall back to a minimal safe result.
    try:
        tri = Delaunay(pts)
    except (ValueError, RuntimeError):
        return ensure_closed(pts)

    simplices = tri.simplices
    if simplices is None or len(simplices) == 0:
        return ensure_closed(pts)

    pa = pts[simplices[:, 0]]
    pb = pts[simplices[:, 1]]
    pc = pts[simplices[:, 2]]

    a = np.linalg.norm(pb - pc, axis=1)
    b = np.linalg.norm(pa - pc, axis=1)
    c = np.linalg.norm(pa - pb, axis=1)

    s = 0.5 * (a + b + c)
    area_sq = np.maximum(s * (s - a) * (s - b) * (s - c), 0.0)
    area = np.sqrt(area_sq)

    # circumradius R = abc / (4A); handle A≈0 safely
    radius = (a * b * c) / (4.0 * area + eps)

    keep = radius <= float(alpha)
    if not np.any(keep):
        return get_convex_hull(pts)

    # Count triangle edges; boundary edges occur exactly once.
    counts = Counter()
    for t in simplices[keep]:
        u0, u1, u2 = (int(t[0]), int(t[1]), int(t[2]))
        counts[_edge_key(u0, u1)] += 1
        counts[_edge_key(u1, u2)] += 1
        counts[_edge_key(u2, u0)] += 1

    boundary_edges = [e for e, n in counts.items() if n == 1]
    if not boundary_edges:
        return get_convex_hull(pts)

    loops = _boundary_loops_from_edges(pts, boundary_edges, eps=eps)
    if not loops:
        return get_convex_hull(pts)

    def loop_area(idx_loop: Loop) -> float:
        poly = ensure_closed(pts[np.asarray(idx_loop, dtype=int)])
        props = get_polygon_properties(poly)
        return float(props.get("area", 0.0))

    best_loop = max(loops, key=loop_area)
    return ensure_closed(pts[np.asarray(best_loop, dtype=int)])


def offset_polygon_convex_hull(poly, distance, miter_limit=2.0):
    """poly=[[x,y],...], distance=float, miter_limit=float  →  [[x,y],...]
    Offsets the convex hull of the input polygon (always convex result).
    """
    hull = get_convex_hull(poly)
    pts = np.array(hull, dtype=float)
    if len(pts) < 4:
        return hull

    x, y = pts[:, 0], pts[:, 1]
    if (0.5 * np.sum(x * np.roll(y, -1) - y * np.roll(x, -1))) < 0:
        pts = pts[::-1]

    vecs = np.diff(pts, axis=0)
    normals = np.stack([vecs[:, 1], -vecs[:, 0]], axis=1)

    norm_lens = np.linalg.norm(normals, axis=1, keepdims=True)
    norm_lens[norm_lens < EPS] = 1.0
    normals /= norm_lens

    n2 = normals
    n1 = np.roll(normals, 1, axis=0)
    miter = n1 + n2

    denom = 1.0 + np.sum(n1 * n2, axis=1, keepdims=True)
    denom[denom < EPS] = EPS

    scale = 2.0 / denom
    limit_sq = miter_limit * miter_limit
    scale[scale > limit_sq] = limit_sq

    miter_lens_sq = np.sum(miter**2, axis=1, keepdims=True)
    offset_vecs = miter * (scale * distance / (miter_lens_sq + EPS))

    new_pts = pts[:-1] + offset_vecs
    return ensure_closed(new_pts)


def _intersect_segments(p1, p2, p3, p4):
    """p1,p2,p3,p4=[x,y]  →  [x,y]|None
    Returns intersection point of segments (p1-p2) and (p3-p4), or None.
    """
    x1, y1 = p1
    x2, y2 = p2
    x3, y3 = p3
    x4, y4 = p4

    denom = (y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1)
    if abs(denom) < EPS:
        return None  # Parallel

    ua = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / denom
    ub = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / denom

    if 0.0 < ua < 1.0 and 0.0 < ub < 1.0:
        return [x1 + ua * (x2 - x1), y1 + ua * (y2 - y1)]
    return None


def prune_self_intersections(poly):
    """poly=[[x,y],...]  →  [[x,y],...]
    Recursively clips off self-intersecting loops from a polygon.
    """
    pts = ensure_open(poly)
    n = len(pts)
    if n < 4:
        return ensure_closed(pts)

    for i in range(n - 2):
        p1, p2 = pts[i], pts[i + 1]

        for j in range(i + 2, n):
            p3 = pts[j]
            p4 = pts[(j + 1) % n]

            hit = _intersect_segments(p1, p2, p3, p4)
            if hit:
                new_poly = pts[:i + 1] + [hit] + pts[j + 1:]
                return prune_self_intersections(new_poly)

    return ensure_closed(pts)


def offset_polygon_geometric(poly, distance, miter_limit=2.0):
    """poly=[[x,y],...], distance=float, miter_limit=float  →  [[x,y],...]
    True geometric offset with miter limiting + self-intersection pruning.
    """
    pts = ensure_closed_np(poly)
    if len(pts) < 4:
        return poly

    is_ccw = get_polygon_properties(pts)['orientation'] > 0
    if not is_ccw:
        pts = pts[::-1]

    vecs = np.diff(pts, axis=0)
    normals = np.stack([vecs[:, 1], -vecs[:, 0]], axis=1)
    norm_lens = np.linalg.norm(normals, axis=1, keepdims=True)
    normals /= (norm_lens + EPS)

    miter = np.roll(normals, 1, axis=0) + normals
    denom = 1.0 + np.sum(np.roll(normals, 1, axis=0) * normals, axis=1, keepdims=True)
    scale = 2.0 / (denom + EPS)

    limit_sq = miter_limit * miter_limit
    scale[scale > limit_sq] = limit_sq

    offset_vecs = miter * (scale * distance / (np.sum(miter**2, axis=1, keepdims=True) + EPS))
    new_pts = pts[:-1] + offset_vecs

    if not is_ccw:
        new_pts = new_pts[::-1]
    return prune_self_intersections(new_pts)


# ----------------- RASTERIZATION (LINES → GRID) -------------------------------------------------------

def rasterize_lines(lines, resolution=0.5, padding=5.0, line_width=None):
    """lines=[[[x,y],...],...], resolution=float, padding=float, line_width=float|None
        →  (grid=np.ndarray(bool), origin=(float,float))
    Rasterizes open polylines into a boolean occupancy grid in world coords.
    """
    valid_lines = [l for l in lines if len(l) > 1]
    if not valid_lines:
        return None, (0.0, 0.0)

    all_pts = np.vstack(valid_lines)
    min_x, min_y = np.min(all_pts, axis=0) - padding
    max_x, max_y = np.max(all_pts, axis=0) + padding

    w = int(np.ceil((max_x - min_x) / resolution)) + 1
    h = int(np.ceil((max_y - min_y) / resolution)) + 1

    grid = np.zeros((h, w), dtype=np.int8)

    for line in valid_lines:
        seg_pts = np.asarray(segment_poly(line, resolution=resolution * 0.5), float)
        idx_x = ((seg_pts[:, 0] - min_x) / resolution).astype(int)
        idx_y = ((seg_pts[:, 1] - min_y) / resolution).astype(int)
        idx_x = np.clip(idx_x, 0, w - 1)
        idx_y = np.clip(idx_y, 0, h - 1)
        grid[idx_y, idx_x] = 1

    if line_width is None or line_width < 0:
        line_width = resolution
    px_r = int(math.ceil((line_width / 2.0) / resolution))

    if px_r > 0:
        rng = np.arange(-px_r, px_r + 1)
        yy, xx = np.meshgrid(rng, rng)
        kernel = ((xx**2 + yy**2) <= (px_r**2 + 0.5)).astype(np.int8)

        thick = ndimage.binary_dilation(grid, structure=kernel).astype(float)
        smooth = ndimage.gaussian_filter(thick, sigma=0.8)
        grid = (smooth > 0.5).astype(bool)
    else:
        grid = grid.astype(bool)

    return grid, (float(min_x), float(min_y))


def offset_polygon(poly, distance, resolution=0.25, padding=2.0, min_area=0.5):
    """poly=[[x,y],...], distance=float(mm)
         resolution=float(mm), padding=float, min_area=float
         →  [ [[x,y],...], [[x,y],...], ... ]  (list of polygons)
    SDF-based offset: rasterize → signed distance → contour extraction.
    """
    poly = ensure_closed(poly)
    expand_pad = padding + max(0, distance)
    grid, origin = rasterize_lines([poly], resolution=resolution, padding=expand_pad)

    if grid is None or grid.max() == 0:
        return []

    struct = np.ones((3, 3), dtype=int)
    filled = ndimage.binary_fill_holes(grid, structure=struct)

    dist_in = ndimage.distance_transform_edt(filled)
    dist_out = ndimage.distance_transform_edt(~filled)
    sdf = dist_in - dist_out

    threshold = -(distance / resolution)
    offset_grid = (sdf > threshold)

    return raster_outline(
        offset_grid,
        resolution,
        origin,
        filter_holes=True,
        simplify_tol=resolution / 2,
        min_area=min_area,
    )


# ----------------- CONTOURING (GRID → POLYGONS via SDF + MARCHING SQUARES) ----------------------------

_MS_EDGE_LUT = [
    [],                 # 0: 0000
    [(3, 0)],           # 1
    [(0, 1)],           # 2
    [(3, 1)],           # 3
    [(1, 2)],           # 4
    [(3, 0), (1, 2)],   # 5 (ambiguous, 2 disjoint segments)
    [(0, 2)],           # 6
    [(3, 2)],           # 7
    [(2, 3)],           # 8
    [(0, 2)],           # 9
    [(1, 3), (0, 2)],   # 10
    [(1, 2)],           # 11
    [(3, 1)],           # 12
    [(0, 1)],           # 13
    [(3, 0)],           # 14
    []                  # 15
]


def _get_edge_point(edge_idx, vals, r, c, threshold=0.0):
    """edge_idx=int, vals=(tl,tr,br,bl), r=int, c=int, threshold=float  →  (x,y)
    Linear interpolation of zero-crossing on a cell edge (marching squares).
    """
    v_tl, v_tr, v_br, v_bl = vals
    if edge_idx == 0:   # Bottom (BL -> BR)
        t = (threshold - v_bl) / (v_br - v_bl + EPS)
        return (c + t, r + 1.0)
    elif edge_idx == 1: # Right (TR -> BR)
        t = (threshold - v_tr) / (v_br - v_tr + EPS)
        return (c + 1.0, r + t)
    elif edge_idx == 2: # Top (TL -> TR)
        t = (threshold - v_tl) / (v_tr - v_tl + EPS)
        return (c + t, r + 0.0)
    elif edge_idx == 3: # Left (TL -> BL)
        t = (threshold - v_tl) / (v_bl - v_tl + EPS)
        return (c + 0.0, r + t)
    return (float(c), float(r))


def raster_outline(grid, resolution, origin,
                   filter_holes=True,
                   simplify_tol=None,
                   smoothing_iters=1,
                   min_area=0.0):
    """
    grid=np.ndarray(bool|float), resolution=float, origin=(ox,oy),
    filter_holes=bool, simplify_tol=float|None, smoothing_iters=int, min_area=float
      →  [ [[x,y],...], ... ]  (polygons, largest area first)
    Extracts smoothed, simplified contours from a binary/signed-distance grid.
    """
    if simplify_tol is None:
        simplify_tol = resolution
    if grid is None:
        return []

    if filter_holes:
        grid = ndimage.binary_fill_holes(grid)

    dist_in = ndimage.distance_transform_edt(grid)
    dist_out = ndimage.distance_transform_edt(~grid)
    sdf = ndimage.gaussian_filter((dist_in - dist_out).astype(float), sigma=1.0)

    h, w = sdf.shape
    padded = np.full((h + 2, w + 2), -100.0)
    padded[1:-1, 1:-1] = sdf

    bin_pad = (padded > 0.0).astype(int)
    indices = (bin_pad[:-1, :-1] * 8) + (bin_pad[:-1, 1:] * 4) \
              + (bin_pad[1:, 1:] * 2) + (bin_pad[1:, :-1] * 1)

    segments = []
    active_rows, active_cols = np.where((indices > 0) & (indices < 15))

    for r, c in zip(active_rows, active_cols):
        idx = indices[r, c]
        vals = [padded[r, c], padded[r, c + 1], padded[r + 1, c + 1], padded[r + 1, c]]
        for (e_start, e_end) in _MS_EDGE_LUT[idx]:
            segments.append(
                (_get_edge_point(e_start, vals, r, c),
                 _get_edge_point(e_end, vals, r, c))
            )

    adj = {}
    for u, v in segments:
        adj.setdefault(u, []).append(v)
        adj.setdefault(v, []).append(u)

    polygons = []
    visited = set()
    for start in list(adj.keys()):
        if start in visited:
            continue
        loop = [start]
        curr = start
        visited.add(curr)
        prev = None
        while True:
            nxt = next((n for n in adj[curr] if n != prev), None)
            if not nxt or nxt in visited:
                break
            prev, curr = curr, nxt
            visited.add(curr)
            loop.append(curr)
        if len(loop) > 2:
            polygons.append(loop)

    results = []
    ox, oy = origin

    for poly in polygons:
        pts = (np.array(poly) - 1.0) * resolution
        pts[:, 0] += ox
        pts[:, 1] += oy
        pts_list = pts.tolist()

        props = get_polygon_properties(pts_list)
        if props['area'] < min_area:
            continue

        if simplify_tol > 0:
            pts_list = simplify_polygon(pts_list, tolerance=simplify_tol)

        if smoothing_iters > 0:
            pts_list = chaikin_smooth(pts_list, iterations=smoothing_iters)

        results.append({'pts': pts_list, 'area': props['area']})

    results.sort(key=lambda x: x['area'], reverse=True)
    return [r['pts'] for r in results]


# ----------------- PATH EXTRACTION ON PERIMETERS ------------------------------------------------------

def _get_point_at_s(poly, s, perimeter=None):
    """poly=[[x,y],...], s=float, perimeter=float|None  →  ([x,y], seg_index:int, remaining:float)
    Returns point at arc-length s along closed perimeter (wraps around).
    """
    pts = ensure_closed_np(poly)
    if perimeter is None:
        perimeter = np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1))

    s = s % perimeter
    if s < 0:
        s += perimeter

    cur_s = 0.0
    for i in range(len(pts) - 1):
        p0, p1 = pts[i], pts[i + 1]
        seg_len = float(np.linalg.norm(p1 - p0))

        if cur_s + seg_len >= s:
            t = (s - cur_s) / (seg_len + 1e-9)
            pt = p0 + (p1 - p0) * t
            return pt.tolist(), i, seg_len - (s - cur_s)

        cur_s += seg_len

    return pts[-1].tolist(), len(pts) - 2, 0.0


def get_polyline_segment(poly, target, length, mode='center'):
    """poly=[[x,y],...], target=[x,y], length=float, mode in {'center','cw','ccw'}
         →  [[x,y],...]  (open polyline)
    Extracts an arc-length≈length segment along the polygon perimeter
    anchored around the closest point to target.
    """
    if not poly or length <= 0:
        return []

    pts = ensure_closed_np(poly)
    if pts.shape[0] < 3:
        return []

    segs = np.diff(pts, axis=0)
    dists = np.linalg.norm(segs, axis=1)
    perimeter = float(np.sum(dists))
    if perimeter <= EPS:
        return []

    proj = get_closest_point_on_perimeter(poly, target)
    if not proj:
        return []

    s_anchor = float(proj['s'])
    L = float(length)

    mode = str(mode).strip()
    if mode == 'center':
        s_start = s_anchor - 0.5 * L
    elif mode == 'cw':
        s_start = s_anchor - L
    elif mode == 'ccw':
        s_start = s_anchor
    else:
        raise RuntimeError(
            "get_polyline_segment: mode must be 'center', 'cw' or 'ccw' (got %r)" % mode
        )

    # Choose sampling density: ~40 points across requested length, min 2
    n_samples = max(2, int(math.ceil(L / max(L / 40.0, 1e-6))))
    step = L / (n_samples - 1) if n_samples > 1 else L

    line = []
    for i in range(n_samples):
        s = s_start + i * step
        pt, _, _ = _get_point_at_s(poly, s, perimeter=perimeter)
        line.append(pt)

    return line



# ----------------------- opencv

def rasterize_polygon_fill_cv(poly, resolution=0.25, padding=2.0):
    """
    poly=[[x,y],...], resolution=float, padding=float
      →  (grid=np.ndarray(bool), origin=(float,float))
    Rasterizes a closed polygon into a solid boolean grid using OpenCV.
    This prevents 'bridging' of narrow gaps that occurs with line-thickening methods.
    """
    if cv2 is None:
        raise RuntimeError
    pts = np.array(poly, dtype=float)
    if len(pts) < 3:
        return None, (0.0, 0.0)

    min_x, min_y = np.min(pts, axis=0) - padding
    max_x, max_y = np.max(pts, axis=0) + padding

    w = int(np.ceil((max_x - min_x) / resolution)) + 1
    h = int(np.ceil((max_y - min_y) / resolution)) + 1

    grid = np.zeros((h, w), dtype=np.uint8)

    grid_pts = np.round((pts - [min_x, min_y]) / resolution).astype(np.int32)
    cv2.fillPoly(grid, [grid_pts], 1)

    return grid.astype(bool), (float(min_x), float(min_y))
