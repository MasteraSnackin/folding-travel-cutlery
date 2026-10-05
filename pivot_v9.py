"""V9 fixed barrel and mechanically captive, independently rotating cap.

Native coordinates are assembly coordinates in mm: lower utensil starts Z0.
The source main-thread profile/pitch is retained. The separate coarse LEFT-HAND
keeper is removed CLOCKWISE with the printed cover-key driver. Its shoulder
seats on the fixed barrel floor, and its recessed conical head limits cap rise.
No retention claim depends on friction or a flexible printed snap.
"""
from __future__ import annotations

import json
from pathlib import Path
from zipfile import ZipFile
import xml.etree.ElementTree as ET
import numpy as np
import trimesh
import manifold3d as mf
from repair_parts import as_manifold, as_trimesh, export_checked

ROOT = Path(__file__).resolve().parent
MAIN_PITCH = 2.5
MAIN_PHASE = 172.5
STACK_HEIGHT = 16.2
BARREL_TOP = 21.0
BARREL_RADIUS = 11.8
CAP_ROOF_UNDERSIDE = 21.6
CAP_TOP = 25.6
CAP_RISE_LIMIT = 4.6
KEEPER_PITCH = 2.0
KEEPER_ROOT_RADIUS = 3.35
KEEPER_MAJOR_RADIUS = 4.0
KEEPER_CLEARANCE = .28
KEEPER_HEAD_BOTTOM = 21.61
KEEPER_HEAD_TOP = 25.6
CONTACT_DEPTH_TOL = 2e-5


def cylinder(z0, z1, r, r1=None, segments=192):
    return mf.Manifold.cylinder(z1-z0, r, r if r1 is None else r1,
                                circular_segments=segments).translate((0, 0, z0))


def slab(z0, z1, radius=30.):
    return cylinder(z0, z1, radius)


def load_source(name, source=None):
    source = Path(source) if source else ROOT/'source'/'pribor-cutlery-v7.3mf'
    ids = {'nut': 3, 'screw': 6}
    ns = {'m': 'http://schemas.microsoft.com/3dmanufacturing/core/2015/02'}
    with ZipFile(source) as z:
        root = ET.fromstring(z.read(f'3D/Objects/object_{ids[name]}.model'))
    vertices = np.array([[float(v.get(k)) for k in ('x','y','z')]
                         for v in root.findall('.//m:vertex', ns)])
    faces = np.array([[int(v.get(k)) for k in ('v1','v2','v3')]
                      for v in root.findall('.//m:triangle', ns)])
    return as_manifold(trimesh.Trimesh(vertices, faces, process=False))


def keeper_thread(z0, z1, radial_allowance=0.):
    """Trapezoidal single-start LEFT-HAND thread; explicit matching cutter.

    The cutter uses the same phase and pitch, enlarged radially by 0.28 mm.
    This is a prototype clearance, to be checked with the supplied test set.
    """
    angles = np.linspace(0., 2*np.pi, 128, endpoint=False)
    levels = np.linspace(z0, z1, int(np.ceil((z1-z0)/.04))+1)
    verts = []
    for z in levels:
        phase = np.mod(z/KEEPER_PITCH + angles/(2*np.pi), 1.)
        tooth = np.clip((.45-np.abs(phase-.5))/.30, 0., 1.)
        radius = KEEPER_ROOT_RADIUS + (KEEPER_MAJOR_RADIUS-KEEPER_ROOT_RADIUS)*tooth + radial_allowance
        verts.extend(np.column_stack((radius*np.cos(angles), radius*np.sin(angles),
                                      np.full(len(angles), z))).tolist())
    n = len(angles)
    faces = []
    for j in range(len(levels)-1):
        for i in range(n):
            a=j*n+i; b=j*n+(i+1)%n; c=(j+1)*n+i; d=(j+1)*n+(i+1)%n
            faces.extend(((a,b,d),(a,d,c)))
    bottom=len(verts); verts.append([0,0,z0])
    top=len(verts); verts.append([0,0,z1])
    for i in range(n):
        faces.append((bottom,(i+1)%n,i))
        faces.append((top,(len(levels)-1)*n+i,(len(levels)-1)*n+(i+1)%n))
    return as_manifold(trimesh.Trimesh(verts, faces, process=True))


def build_pivot(source=None, attachment_ears=None, attachment_cutter=None):
    """Return {barrel, cap, keeper} as Trimesh in native assembled poses.

    The optional cover ear/cutter arguments are Manifold solids. Passing them
    does not transfer ownership of cover geometry to this module.
    """
    # Flip the original female part: original flange becomes stationary base.
    original_female = load_source('nut', source).rotate((180,0,0)).translate((0,0,4.19999981))
    # Original flange is retained above Z-5, with a 1 mm bottom extension.
    base = original_female ^ slab(-5., 0.)
    extension = (original_female ^ slab(-4., -3.)).translate((0,0,-2.))
    base = base + extension
    # Work from the female VOID, preserving its original radial thread shape.
    # Moving the original complete cavity by three pitches gives enough
    # length without duplicating near-coincident triangulated thread faces.
    raised_female = original_female.translate((0,0,7.5))
    thread_void = slab(2.9,20.9,10.5) - raised_female
    thread_void = thread_void + cylinder(20.2,21.1,9.5,10.4)
    barrel = base + cylinder(0., BARREL_TOP, BARREL_RADIUS)
    barrel = barrel - (thread_void ^ slab(3., BARREL_TOP+.05))
    # Internal floor with through keeper thread. No blind cleaning cavity.
    barrel = barrel + cylinder(-6., 3., BARREL_RADIUS)
    keeper_cutter = keeper_thread(-6.1, 3.1, KEEPER_CLEARANCE)
    # Flared thread entry, at the upper face of the internal floor.
    keeper_cutter = keeper_cutter + cylinder(2.5, 3.05, 3.95, 4.5)
    barrel = barrel - keeper_cutter
    if attachment_ears is not None:
        barrel = barrel + attachment_ears
    if attachment_cutter is not None:
        barrel = barrel - attachment_cutter

    # Transform is derived from the previously matched v8 thread poses.
    male = load_source('screw', source).rotate((180,0,0)).rotate((0,0,MAIN_PHASE)).translate((0,0,14.75000010))
    male = male ^ slab(3.4,20.9,10.0)
    male = male + (cylinder(20.,CAP_ROOF_UNDERSIDE+.1,8.8)-cylinder(19.9,CAP_ROOF_UNDERSIDE+.2,5.5))
    # Existing hollow bore leaves the keeper shoulder free to rotate past.
    # Lead-in chamfer on the male thread tip does not change working pitch.
    male = male ^ (cylinder(3.4,4.3,8.75,9.9)+cylinder(4.3,22.,10.))
    ring = cylinder(STACK_HEIGHT, CAP_TOP, 19.) - cylinder(STACK_HEIGHT-.1, CAP_ROOF_UNDERSIDE, 12.2)
    cap = male + ring
    # Local central annulus creates the captive stop. Tapered well is printable
    # head-down without an unsupported horizontal internal annular ceiling.
    cap = cap + cylinder(16.2, CAP_TOP, 7.)
    cap = cap - cylinder(3.3,16.2,5.5)
    # ID10.6 lets the keeper's OD10 seating shoulder pass right through on
    # deliberate disassembly. The head is larger and remains the cap stop.
    cap = cap - cylinder(16.19,17.,5.3)
    cap = cap - cylinder(17.,19.,5.3,7.3)
    cap = cap - cylinder(19.,CAP_TOP+.1,7.3)
    # Broad, shallow flutes improve turning without sharp knurl teeth.
    for angle in range(0,360,30):
        a=np.deg2rad(angle)
        cutter=cylinder(16.5,CAP_TOP+.1,2.).translate((19.8*np.cos(a),19.8*np.sin(a),0))
        cap=cap-cutter

    keeper = keeper_thread(-4.4, 2.8)
    keeper = keeper ^ (cylinder(-4.4,-3.6,3.3,4.05)+cylinder(-3.6,3.,4.1))
    keeper = keeper + cylinder(2.6,KEEPER_HEAD_BOTTOM+.1,KEEPER_ROOT_RADIUS)
    # Positive shoulder establishes the keeper's installed height.
    # Taper the upper shoulder so head-down printing grows this collar at
    # 45 degrees instead of requiring a floating horizontal flange.
    keeper = keeper + cylinder(3.,4.,5.) + cylinder(4.,5.65,5.,KEEPER_ROOT_RADIUS)
    # A 0.01 mm meshing allowance avoids faceted cone interference under
    # arbitrary cap rotation. Nominal cap stop is 4.61 mm, checked to 4.6 mm.
    keeper = keeper + cylinder(KEEPER_HEAD_BOTTOM,23.11,5.3,6.8) + cylinder(23.11,KEEPER_HEAD_TOP,6.8)
    slot=mf.Manifold.cube((10.,2.6,2.2)).translate((-5.,-1.3,23.6))
    keeper=keeper-slot
    result={}
    for n,s in {'barrel':barrel,'cap':cap,'keeper':keeper}.items():
        try:result[n]=as_trimesh(s)
        except Exception as exc:raise ValueError(f'{n} failed mesh checks: {exc}') from exc
    return result


def cap_pose(mesh, rise=0.):
    result=mesh.copy()
    if rise:
        result.apply_transform(trimesh.transformations.rotation_matrix(np.deg2rad(360.*rise/MAIN_PITCH),(0,0,1)))
        result.apply_translation((0,0,rise))
    return result


def keeper_pose(mesh, rise=0.):
    """Keeper release is CLOCKWISE viewed from above; shoulder lifts with rise."""
    result=mesh.copy()
    if rise:
        result.apply_transform(trimesh.transformations.rotation_matrix(np.deg2rad(-360.*rise/KEEPER_PITCH),(0,0,1)))
        result.apply_translation((0,0,rise))
    return result


def print_meshes(parts):
    result={}
    for n,m in parts.items():
        m=m.copy()
        if n in ('cap','keeper'):
            m.apply_transform(trimesh.transformations.rotation_matrix(np.pi,(1,0,0)))
        m.apply_translation((0,0,-m.bounds[0,2])); result[n]=m
    return result


def overlap(a,b):
    hit=as_manifold(a)^as_manifold(b)
    if hit.is_empty():return 0.
    bounds=hit.bounding_box()
    if bounds[5]-bounds[2] < CONTACT_DEPTH_TOL:return 0.
    return float(hit.volume())


def validate(parts):
    report={'units':'mm','scope':'Digital geometry only; printed fit and strength untested.',
      'main_pitch_mm':MAIN_PITCH,'main_phase_degrees':MAIN_PHASE,
      'barrel_top_mm':BARREL_TOP,'barrel_diameter_mm':2*BARREL_RADIUS,
      'fixed_flange_z_mm':[-6.,0.],'clamped_cap_top_mm':CAP_TOP,
      'verified_clear_cap_rise_mm':CAP_RISE_LIMIT,'nominal_conical_stop_rise_mm':4.61,'keeper_pitch_mm':KEEPER_PITCH,
      'keeper_release_direction':'clockwise (left-hand thread)',
      'keeper_head_driver_slot_mm':[10.,2.6,2.0],
      'barrel_to_cap_roof_closed_clearance_mm':CAP_ROOF_UNDERSIDE-BARREL_TOP,
      'full_release_main_thread_z_overlap_mm':BARREL_TOP-(3.4+CAP_RISE_LIMIT),
      'full_release_unmodified_profile_engagement_excluding_leadins_mm':20.2-(4.3+CAP_RISE_LIMIT),
      'washer_bottom_at_stop_mm':13.8+CAP_RISE_LIMIT,
      'washer_top_at_stop_mm':STACK_HEIGHT+CAP_RISE_LIMIT,
      'washer_barrel_axial_overlap_at_stop_mm':BARREL_TOP-(13.8+CAP_RISE_LIMIT),
      'cap_sweep':{},'keeper_removal_sweep':{},'cap_removal_after_keeper':{}}
    for rise in np.linspace(0.,CAP_RISE_LIMIT,24):
        cap=cap_pose(parts['cap'],float(rise))
        report['cap_sweep'][f'{rise:.3f}']={'barrel':overlap(cap,parts['barrel']), 'keeper':overlap(cap,parts['keeper'])}
    report['beyond_stop_overlap_mm3']=overlap(cap_pose(parts['cap'],CAP_RISE_LIMIT+.1),parts['keeper'])
    # Remove main cap first in this clearance check. During real teardown,
    # first undo keeper inside the stationary closed cap, then remove the cap.
    for rise in np.arange(0.,35.01,.5):
        k=keeper_pose(parts['keeper'],float(rise))
        report['keeper_removal_sweep'][f'{rise:.3f}']={'barrel':overlap(k,parts['barrel']), 'closed_cap':overlap(k,parts['cap'])}
    for rise in (0.,4.6,6.,10.,14.,18.,22.):
        report['cap_removal_after_keeper'][str(rise)]=overlap(cap_pose(parts['cap'],rise),parts['barrel'])
    wrong_main=parts['cap'].copy();wrong_main.apply_translation((0,0,1.25))
    wrong_keeper=parts['keeper'].copy()
    wrong_keeper.apply_transform(trimesh.transformations.rotation_matrix(np.deg2rad(54.),(0,0,1)))
    wrong_keeper.apply_translation((0,0,.3))
    report['wrong_main_thread_phase_interference_mm3']=overlap(wrong_main,parts['barrel'])
    report['wrong_keeper_thread_hand_interference_mm3']=overlap(wrong_keeper,parts['barrel'])
    report['seated_barrel_keeper_overlap_mm3']=overlap(parts['barrel'],parts['keeper'])
    report['passes']=(all(not any(v.values()) for v in report['cap_sweep'].values())
      and all(not any(v.values()) for v in report['keeper_removal_sweep'].values())
      and not any(report['cap_removal_after_keeper'].values())
      and report['wrong_main_thread_phase_interference_mm3']>0
      and report['wrong_keeper_thread_hand_interference_mm3']>0
      and report['beyond_stop_overlap_mm3']>0
      and report['seated_barrel_keeper_overlap_mm3']==0)
    return report


def main():
    from cover_v9 import attachment_ears,attachment_cutter
    parts=build_pivot(attachment_ears=attachment_ears(),attachment_cutter=attachment_cutter())
    out=ROOT/'intermediate'/'pivot';out.mkdir(parents=True,exist_ok=True)
    for n,m in parts.items():export_checked(m,out/f'{n}-assembled.stl')
    for n,m in print_meshes(parts).items():export_checked(m,out/f'{n}-print.stl')
    report=validate(parts)
    (out/'pivot-validation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))
    assert report['passes'],'Pivot audit failed; inspect pivot-validation.json'


if __name__=='__main__':main()
