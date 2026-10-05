"""V9 printed travel-cutlery sleeve and mechanically keyed latch.

Canonical assembly: fork Z0..4, knife Z4..8, spoon Z8..13.8, pivot (0,0),
folded tips +Y. The cover MUST be removed before repositioning utensils.
The fixed pivot base owns the attachment ears returned here. Nothing latches
onto a utensil. Units mm. These dimensions are an untested prototype.
"""
from __future__ import annotations
import json
from pathlib import Path
import math
import manifold3d as mf
import numpy as np
import trimesh

KEY_Y=-10.0
KEY_Z=-3.0
KEY_RELEASE_TRAVEL=0.8
EAR_BOUNDS={"right":[[13.5,-13.6,-6.0],[24.0,-6.4,0.0]],
            "left":[[-24.0,-13.6,-6.0],[-13.5,-6.4,0.0]]}


def cube(lo,hi):
    return mf.Manifold.cube(tuple(b-a for a,b in zip(lo,hi))).translate(lo)


def rounded_rectangle(width,height,radius):
    return mf.CrossSection.square((width-2*radius,height-2*radius),center=True).offset(radius,circular_segments=64)


def as_mesh(solid):
    raw=solid.to_mesh64()
    result=trimesh.Trimesh(np.asarray(raw.vert_properties)[:,:3],np.asarray(raw.tri_verts),process=True)
    if not(result.is_watertight and result.is_winding_consistent and result.volume>0):
        raise ValueError('Non-closed or invalid cover mesh')
    return result


def cylinder_x(length,radius,x,y=KEY_Y,z=KEY_Z):
    return mf.Manifold.cylinder(length,radius,circular_segments=64).rotate((0,90,0)).translate((x,y,z))


def attachment_ears():
    """Union these two solids into the stationary flange (OD38,Z−6..0)."""
    ears=[]
    for sign in (-1,1):
        # Flat-sided ear supplies a definite longitudinal insertion stop.
        ear=cube((13.5,-13.6,-6),(24,-6.4,0))
        if sign<0:ear=ear.mirror((1,0,0))
        ears.append(ear)
    return ears[0]+ears[1]


def keyway(xlo=-31.,xhi=31.):
    """Round shaft bore plus horizontal slot for the quarter-turn end lug."""
    return cylinder_x(xhi-xlo,1.85,xlo)+cube((xlo,KEY_Y-3.3,KEY_Z-1.45),(xhi,KEY_Y+3.3,KEY_Z+1.45))


def attachment_cutter():
    """Subtract after union of base and ears; clear of central threaded boss."""
    return keyway()


def triangle_prism(points,z,height):
    return mf.CrossSection([points]).extrude(height).translate((0,0,z))


def build_cover():
    """One solid sleeve with twin rigid sockets, end stops and thumb pad."""
    outer=rounded_rectangle(40.8,18.4,2.6).extrude(56.6).rotate((90,0,0)).translate((0,76.6,6.9))
    inner=rounded_rectangle(37.6,15.2,1.).extrude(56.).rotate((90,0,0)).translate((0,75.,6.9))
    cover=outer-inner
    for x in (-10.,10.):
        cover=cover-mf.Manifold.cylinder(3.6,1.6,circular_segments=64).rotate((90,0,0)).translate((x,77.6,6.9))
    for sign in (-1,1):
        # Constant-width arms continue from sleeve sides, all clear of R19.
        arm=rounded_rectangle(6.7,38.,.8).extrude(11.6).translate((22.65,3.,-8.4))
        # Blend arm into sleeve side without a sharp step at the mouth.
        blend=mf.Manifold.batch_hull([cube((19.3,17.,-3.),(26.,22.,3.2)),cube((19.3,29.,-1.),(20.35,31.,5.))])
        arm=arm+blend
        # Open at -Y; flat inner end at -6.4 is the seated insertion stop.
        socket=cube((18.8,-17.,-6.4),(24.4,-6.4,.4))
        arm=arm-socket
        if sign<0:arm=arm.mirror((1,0,0))
        cover=cover+arm
    cover=cover-keyway()
    # Right outer face has a vertical recess. The turned end lug seats in it,
    # so withdrawing the key is blocked and rotation requires pushing first.
    recess=cube((25.35,KEY_Y-1.5,KEY_Z-3.35),(26.2,KEY_Y+1.5,KEY_Z+3.35))
    cover=cover-recess
    # Smooth convex thumb pad; no small grooves in the thumb contact patch.
    pad=mf.Manifold.sphere(1.,circular_segments=64).scale((13.5,7.,1.4)).translate((0.,30.,15.95))
    cover=cover+pad
    # Shallow direction arrow on top indicates the working end (-Y).
    arrow=triangle_prism([(-2.,24.),(0.,21.),(2.,24.)],15.78,.55)
    cover=cover-arrow
    # Engraved alignment line on both exterior arm tops at the key axis.
    for sign in (-1,1):
        cover=cover-cube((sign*22.7-.2,-12.,2.9),(sign*22.7+.2,-8.,3.5))
    if cover.status()!=mf.Error.NoError or len(cover.decompose())!=1:
        raise ValueError(f'Cover not one valid solid: {cover.status()} / {len(cover.decompose())}')
    return cover


def build_latch_key():
    """Canonical LOCKED and SEATED key; push +X0.8, turn90deg, withdraw−X.

    Lug left face X25.40 seats within recess fromX25.35..26.0. Its rear face
    stops extraction; the recess side walls resist turning while seated.
    No spring force or friction-based seating retention is claimed.
    """
    shaft=cylinder_x(54.2,1.6,-28.6)
    # Vertical bar is locked; turned horizontal it passes through the keyway.
    lug=cube((25.4,KEY_Y-1.1,KEY_Z-3.),(27.6,KEY_Y+1.1,KEY_Z+3.))
    # Flat finger head 8.0mm across, ends kept rounded. Indicator points along
    # locked lug; head and lug are both visible from outside the grip.
    head=rounded_rectangle(8.,2.2,.8).extrude(2.4).rotate((0,90,0)).translate((-30.8,KEY_Y,KEY_Z))
    # The compact finger head is itself the keeper driver: 8mm wide,
    # 2.2mm thick and 2.4mm long; its outer X−30.8 face enters the slot.
    key=shaft+lug+head
    # One common flat bed face when the key is turned90 degrees about X.
    key=key-cube((-40.,KEY_Y-8.,KEY_Z-8.),(29.,KEY_Y-1.1,KEY_Z+8.))
    # Shallow indicator on head exterior; long axis shows locked lug direction.
    key=key-cube((-29.8,KEY_Y+.99,KEY_Z-2.4),(-29.4,KEY_Y+1.7,KEY_Z+2.4))
    if key.status()!=mf.Error.NoError or len(key.decompose())!=1:
        raise ValueError(f'Latch key invalid: {key.status()} / {[i.volume() for i in key.decompose()]}')
    return key


def key_pose(solid,push=0.,turn_deg=0.,withdraw=0.):
    return solid.translate((0,-KEY_Y,-KEY_Z)).rotate((turn_deg,0,0)).translate((push-withdraw,KEY_Y,KEY_Z))


def cover_print_transform():
    """Closed end on bed; arms point upward. Brim recommended for stability."""
    return np.array([[1.,0.,0.,0.],[0.,0.,1.,8.4],[0.,-1.,0.,76.6],[0.,0.,0.,1.]])


def latch_key_print_transform():
    """Lay long shaft flat on its common flat side; translate minZ to0 after.

    Head/driver, shaft and lug share the same build-plane contact.
    """
    return trimesh.transformations.rotation_matrix(math.pi/2,(1,0,0))


def main():
    out=Path(__file__).parent/'intermediate'/'cover';out.mkdir(parents=True,exist_ok=True)
    cover=build_cover();key=build_latch_key()
    base=mf.Manifold.cylinder(6,19,circular_segments=128).translate((0,0,-6))
    base=(base+attachment_ears())-attachment_cutter()
    names={'cover':cover,'cover_key':key,'attachment_base_fixture':base}
    parts={}
    for name,solid in names.items():
        mesh=as_mesh(solid);mesh.export(out/f'{name}.stl')
        parts[name]={'bounds_mm':mesh.bounds.tolist(),'volume_mm3':float(mesh.volume),'watertight':bool(mesh.is_watertight),'solid_count':len(solid.decompose())}
    collisions={}
    for name,a,b in [('cover/base',cover,base),('cover/key_locked',cover,key),('base/key_locked',base,key)]:
        collisions[name]=float((a^b).volume())
    sequence={}
    for push in (0,.2,.4,.6,.8):
        moving=key_pose(key,push=push)
        sequence[f'push_{push}']=float((moving^(cover+base)).volume())
    for angle in (0,15,30,45,60,75,90):
        moving=key_pose(key,push=.8,turn_deg=angle)
        sequence[f'turn_{angle}']=float((moving^(cover+base)).volume())
    for distance in (0,5,15,30,45,60):
        moving=key_pose(key,push=.8,turn_deg=90,withdraw=distance)
        sequence[f'withdraw_{distance}']=float((moving^(cover+base)).volume())
    insertion={}
    for travel in (0,.1,.5,1,5,10,15,20,35,55,80):
        insertion[str(travel)]=float((cover.translate((0,travel,0))^base).volume())
    # Intentionally forbidden motions must meet solid material.
    blockers={'pull_without_turn_1mm_mm3':float((key_pose(key,withdraw=1)^(cover+base)).volume()),
              'turn_without_push_20deg_mm3':float((key_pose(key,turn_deg=20)^(cover+base)).volume()),
              'cover_pulled_1mm_with_key_mm3':float((cover.translate((0,1,0))^(base+key)).volume()),
              'cover_overinserted_1mm_mm3':float((cover.translate((0,-1,0))^base).volume())}
    report={'units':'mm','parts':parts,'assembled_collisions_mm3':collisions,'key_release_sequence_collisions_mm3':sequence,
            'cover_insertion_fixture_collisions_mm3':insertion,'blocked_motion_intersections':blockers,
            'ear_bounds':EAR_BOUNDS,'key_axis_y_z_mm':[KEY_Y,KEY_Z],'key_push_to_release_mm':.8,
            'key_release_rotation_deg':90,'key_withdraw_direction':'-X','cover_remove_direction':'+Y',
            'socket_clearance_z_each_side_mm':.4,'socket_outer_x_clearance_mm':.4,
            'shaft_diameter_mm':3.2,'round_bore_diameter_mm':3.7,'lug_dimensions_x_y_z_locked_mm':[2.2,2.2,6.],
            'recess_depth_mm':.65,'keeper_driver_head_mm':{'width':8.,'thickness':2.2,'head_length':2.4,'tip_x':-30.8,'centre_y':KEY_Y,'centre_z':KEY_Z,'required_engagement':1.9},'cover_print_orientation':'Closed +Y end down; arms up. Inspect bridges and use brim.',
            'latch_key_print_orientation':'Rotate90 degrees aboutX and place common flat side onbed; shaft horizontal; inspect sliced preview.',
            'operation':['Remove transverse key: push head +X0.8mm, quarter-turn90 degrees, withdraw toward−X.',
                         'Pull cover +Y off the stored heads before unlocking or rotating any utensil.',
                         'After pivot is locked, slide cover −Y over unused heads until the arm end stops meet base ears.',
                         'Insert key toward+X with far lug horizontal; quarter-turn then pull head gently−X to seat vertical lug in recess.'],
            'limitations':['Untested printed fit, strength, fatigue, hand comfort and thermal/food-contact properties.',
                           'No spring or friction force is modelled to hold the key seated. Handling retention needs physical testing.',
                           'Cover is a guarded grip only after key installation; no established load rating.']}
    (out/'cover-evidence.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2))

if __name__=='__main__':main()
