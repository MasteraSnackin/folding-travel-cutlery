"""Minimal geometric repairs for pribor-cutlery-v7.3mf.

Uses millimetres. Never writes the source file. The two fork slot roots contain
self-touching internal pockets; small circular reliefs open those pockets into
the existing slots. The nut is shortened by cutting its tip without scaling the
thread, so surviving thread pitch and radial profile are unchanged.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from zipfile import ZipFile
import xml.etree.ElementTree as ET

import manifold3d as mf
import numpy as np
import trimesh

SOURCE = Path(__file__).resolve().parent / 'source' / 'pribor-cutlery-v7.3mf'
NS = {'m': 'http://schemas.microsoft.com/3dmanufacturing/core/2015/02'}
OBJECT_FILES = {'fork': 'object_5.model', 'nut': 'object_3.model'}
NUT_BEARING_Z = 4.19999981


def load_part(name: str, source: Path = SOURCE) -> trimesh.Trimesh:
    with ZipFile(source) as archive:
        root = ET.fromstring(archive.read('3D/Objects/' + OBJECT_FILES[name]))
    vertices = np.array([[float(e.get(a)) for a in ('x', 'y', 'z')]
                         for e in root.findall('.//m:vertex', NS)])
    faces = np.array([[int(e.get(a)) for a in ('v1', 'v2', 'v3')]
                     for e in root.findall('.//m:triangle', NS)])
    return trimesh.Trimesh(vertices, faces, process=False)


def as_manifold(mesh: trimesh.Trimesh) -> mf.Manifold:
    solid = mf.Manifold(mf.Mesh64(np.asarray(mesh.vertices, dtype=np.float64),
                                 np.asarray(mesh.faces, dtype=np.uint64)))
    if solid.status() != mf.Error.NoError:
        raise ValueError(f'Manifold import failed: {solid.status()}')
    return solid


def as_trimesh(solid: mf.Manifold) -> trimesh.Trimesh:
    if solid.status() != mf.Error.NoError or solid.is_empty():
        raise ValueError(f'Invalid result: {solid.status()}')
    raw = solid.to_mesh64()
    mesh = trimesh.Trimesh(np.asarray(raw.vert_properties)[:, :3],
                           np.asarray(raw.tri_verts), process=True)
    check_mesh(mesh)
    return mesh


def bad_edges(mesh: trimesh.Trimesh) -> np.ndarray:
    edges, counts = np.unique(mesh.edges_sorted, axis=0, return_counts=True)
    return edges[counts != 2]


def check_mesh(mesh: trimesh.Trimesh) -> None:
    assert mesh.is_watertight, 'Mesh is not watertight after coordinate merging'
    assert mesh.is_winding_consistent, 'Mesh winding is inconsistent'
    assert mesh.volume > 0, 'Mesh has non-positive volume'
    assert len(bad_edges(mesh)) == 0, 'Mesh has invalid edge incidence'
    assert len(as_manifold(mesh).decompose()) == 1, 'Mesh contains detached solids'


def metrics(mesh: trimesh.Trimesh) -> dict:
    return {'vertices': len(mesh.vertices), 'triangles': len(mesh.faces),
            'bounds_mm': mesh.bounds.tolist(), 'extents_mm': mesh.extents.tolist(),
            'volume_mm3': float(mesh.volume), 'watertight': bool(mesh.is_watertight),
            'consistent_winding': bool(mesh.is_winding_consistent),
            'euler_number': int(mesh.euler_number),
            'invalid_edge_count': len(bad_edges(mesh))}


def repair_fork(mesh: trimesh.Trimesh, relief_radius: float = 0.8,
                relief_depth: float = 0.0075) -> tuple[trimesh.Trimesh, dict]:
    """Open the two defective pockets into existing slots with tiny round cuts.

    relief_depth is below the self-touching edge, rather than below the lowest
    point of the tiny pocket. The original pocket extends about 0.0021 mm below
    that edge. Circular radius 0.8 mm creates a rounded transition, with most of
    the cutter in empty space. Only a small area at each root is removed.
    """
    invalid = bad_edges(mesh)
    if len(invalid) != 2:
        raise ValueError(f'Expected two fork defects, found {len(invalid)}')
    if relief_radius <= 0 or relief_depth <= 0:
        raise ValueError('Relief dimensions must be positive')
    root_points = mesh.vertices[invalid].mean(axis=1)
    if not np.allclose(np.abs(root_points[:, 0]), 5.31472015, atol=1e-4):
        raise ValueError('Unexpected defect positions; inspect source version')
    zmin, zmax = mesh.bounds[:, 2]
    cutters = [mf.Manifold.cylinder(float(zmax-zmin+4), relief_radius,
                                   circular_segments=512).translate(
                (float(p[0]), float(p[1] + relief_radius - relief_depth),
                 float(zmin-2))) for p in root_points]
    cutter = mf.Manifold.batch_boolean(cutters, mf.OpType.Add)
    original = as_manifold(mesh)
    removed = original ^ cutter
    repaired = as_trimesh(original - cutter)
    if not np.allclose(mesh.bounds, repaired.bounds, atol=1e-8):
        raise ValueError('Fork overall bounds unexpectedly changed')
    evidence = {'method': 'Subtract two circular through-thickness reliefs at self-touching slot roots',
                'relief_radius_mm': relief_radius,
                'relief_depth_below_touching_edge_mm': relief_depth,
                'original_defect_edges_mm': mesh.vertices[invalid].tolist(),
                'removed_volume_mm3': float(removed.volume()),
                'removed_volume_percent': float(removed.volume()/mesh.volume*100),
                'removed_bounds_mm': list(removed.bounding_box()),
                'remaining_solid_count': len((original-cutter).decompose()),
                'before': metrics(mesh), 'after': metrics(repaired)}
    return repaired, evidence


def shorten_nut(mesh: trimesh.Trimesh, projection: float = 12.4,
                bearing_z: float = NUT_BEARING_Z) -> tuple[trimesh.Trimesh, dict]:
    """Cut the nut tip to projection millimetres from its bearing surface.

    No axial or radial scaling is used. Thread engagement and smooth entry must
    still be checked in a physical print. This preserves surviving thread shape.
    """
    original_projection = bearing_z - float(mesh.bounds[0, 2])
    if not 5.0 < projection < original_projection:
        raise ValueError('Projection must shorten the tip while retaining usable engagement')
    original = as_manifold(mesh)
    cut_z = bearing_z - projection
    result = original.trim_by_plane((0., 0., 1.), cut_z)
    shortened = as_trimesh(result)
    if not np.allclose(mesh.bounds[:, :2], shortened.bounds[:, :2], atol=1e-8):
        raise ValueError('Nut lateral bounds unexpectedly changed')
    removed = original-result
    evidence = {'method': 'Planar cut perpendicular to thread axis; no thread scaling',
                'bearing_surface_z_mm': bearing_z,
                'original_projection_mm': original_projection,
                'new_projection_mm': projection,
                'cut_plane_z_mm': cut_z,
                'axial_tip_removed_mm': original_projection-projection,
                'removed_volume_mm3': float(removed.volume()),
                'removed_bounds_mm': list(removed.bounding_box()),
                'nominal_13mm_stack_remaining_clamping_travel_mm': 13.-projection,
                'surviving_thread_profile_preserved': True,
                'integration_note': 'Optional intermediate only. If the revised stack exceeds the original nut projection, retain the original nut for longer sleeve guidance.',
                'remaining_solid_count': len(result.decompose()),
                'before': metrics(mesh), 'after': metrics(shortened)}
    return shortened, evidence


def export_checked(mesh: trimesh.Trimesh, path: Path) -> dict:
    path.parent.mkdir(parents=True, exist_ok=True)
    mesh.export(path)
    # STL reload merges coincident vertices. This catches the original defect,
    # even if a mesh kernel masks it by splitting its vertices internally.
    reloaded = trimesh.load_mesh(path, process=True)
    check_mesh(reloaded)
    if not np.allclose(mesh.bounds, reloaded.bounds, atol=2e-5):
        raise ValueError('STL bounds changed beyond float32 rounding')
    try:
        recorded_path = path.resolve().relative_to(Path(__file__).resolve().parent)
    except ValueError:
        recorded_path = Path(path.name)
    return {'path': recorded_path.as_posix(),
            'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
            'reload': metrics(reloaded)}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--source', type=Path, default=SOURCE)
    parser.add_argument('--output', type=Path, default=Path(__file__).parent/'intermediate')
    parser.add_argument('--nut-projection', type=float, default=12.4)
    args = parser.parse_args()
    source_hash = hashlib.sha256(args.source.read_bytes()).hexdigest()
    fork, fork_info = repair_fork(load_part('fork', args.source))
    nut, nut_info = shorten_nut(load_part('nut', args.source), args.nut_projection)
    outputs = {'fork': export_checked(fork, args.output/'fork-repaired.stl'),
               'nut': export_checked(nut, args.output/'nut-shortened.stl')}
    report = {'source': str(args.source), 'source_sha256': source_hash,
              'units': 'mm', 'fork': fork_info, 'nut': nut_info,
              'outputs': outputs,
              'physical_validation_remaining': [
                  'Slicer inspection of fork slot roots and nut thread entry',
                  'Printed thread engagement and free pivot rotation',
                  'Clamping of assembled stack before any bottoming contact',
                  'Resistance to rotation under expected eating loads',
                  'Durability and cleaning performance']}
    assert source_hash == hashlib.sha256(args.source.read_bytes()).hexdigest()
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output/'repair-evidence.json').write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({'fork':fork_info['after'], 'nut':nut_info['after'],
                      'fork_removed_mm3':fork_info['removed_volume_mm3'],
                      'nut_projection_mm':args.nut_projection,
                      'outputs': list(outputs)}, indent=2))


if __name__ == '__main__':
    main()
