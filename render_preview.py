"""Orthographic geometry preview with per-pixel depth testing; no invented surfaces."""
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import to_rgb
from PIL import Image
from build_cutlery import build_parts,assembly,as_manifold,as_trimesh
ROOT=Path(__file__).resolve().parent
COLORS={'fork':'#c8a761','knife':'#ab8443','spoon':'#e0c47f','barrel':'#536974','cap':'#536974','keeper':'#b6c6c3','cover_key':'#d69b53','washer':'#d98a4e','cover':'#587888'}
BG=np.array(to_rgb('#f7f5f0'))


def render(items,azim,elev,width=1000,height=680):
 a,e=np.radians([azim,elev]);view=np.array([np.cos(e)*np.cos(a),np.cos(e)*np.sin(a),np.sin(e)])
 right=np.array([-np.sin(a),np.cos(a),0]);up=np.cross(view,right)
 matrix=np.stack([right,up,view],axis=1)
 tris=[];colours=[]
 light=view*.7+up*.4+np.array([0,0,.4]);light/=np.linalg.norm(light)
 for n,m in items:
  visible=(m.face_normals@view)>1e-8
  t=m.triangles[visible]@matrix
  norm=m.face_normals[visible];shade=.45+.55*np.maximum(0,norm@light)
  tris.append(t);colours.append(np.array(to_rgb(COLORS[n]))[None,:]*shade[:,None])
 tris=np.concatenate(tris);colours=np.concatenate(colours)
 lo=tris[:,:,:2].min(axis=(0,1));hi=tris[:,:,:2].max(axis=(0,1));centre=(lo+hi)/2
 scale=min((width-60)/(hi[0]-lo[0]),(height-45)/(hi[1]-lo[1]))
 tris[:,:,0]=(tris[:,:,0]-centre[0])*scale+width/2
 tris[:,:,1]=height/2-(tris[:,:,1]-centre[1])*scale
 frame=np.broadcast_to(BG,(height,width,3)).copy();depth=np.full((height,width),-np.inf)
 for tri,color in zip(tris,colours):
  x0,y0,z0=tri[0];x1,y1,z1=tri[1];x2,y2,z2=tri[2]
  den=(y1-y2)*(x0-x2)+(x2-x1)*(y0-y2)
  if abs(den)<1e-10:continue
  xmin=max(0,int(np.floor(min(x0,x1,x2))));xmax=min(width-1,int(np.ceil(max(x0,x1,x2))))
  ymin=max(0,int(np.floor(min(y0,y1,y2))));ymax=min(height-1,int(np.ceil(max(y0,y1,y2))))
  if xmin>xmax or ymin>ymax:continue
  xx,yy=np.meshgrid(np.arange(xmin,xmax+1)+.5,np.arange(ymin,ymax+1)+.5)
  u=((y1-y2)*(xx-x2)+(x2-x1)*(yy-y2))/den
  v=((y2-y0)*(xx-x2)+(x0-x2)*(yy-y2))/den
  w=1-u-v;z=u*z0+v*z1+w*z2
  d=depth[ymin:ymax+1,xmin:xmax+1];mask=(u>=-1e-9)&(v>=-1e-9)&(w>=-1e-9)&(z>d)
  d[mask]=z[mask];frame[ymin:ymax+1,xmin:xmax+1][mask]=color
 return np.uint8(np.clip(frame*255,0,255))


def main():
 parts,_=build_parts();fold=assembly(parts,with_cover=True)
 images=[render(list(fold.items()),-62,40)]
 opened=assembly(parts,active='spoon',with_cover=True)
 images.append(render(list(opened.items()),-35,37))
 bare=assembly(parts,with_cover=False)
 shifts={'barrel':0,'fork':6,'knife':11,'spoon':16,'washer':21,'cap':26,'keeper':39}
 detail=[]
 for n,m in bare.items():
  if n in ['fork','knife','spoon']:m=as_trimesh(as_manifold(m).trim_by_plane((0,-1,0),-18.5))
  m=m.copy();m.apply_translation((0,0,shifts[n]));detail.append((n,m))
 images.append(render(detail,-57,25))
 fig,axes=plt.subplots(2,2,figsize=(13,9),facecolor='#f7f5f0')
 fig.suptitle('Folding travel cutlery · v9 prototype',fontsize=22,x=.045,ha='left',y=.97)
 fig.text(.045,.92,'Fixed pivot · captured cap and washer · removable quarter-turn cover key',fontsize=12,color='#4c5356')
 for ax,im,title in zip(axes.flat,images,['Packed and latched','Spoon deployed; other heads covered','Pivot parts · exploded for illustration']):
  ax.imshow(im);ax.axis('off');ax.set_title(title,fontsize=13,pad=10)
 ax=axes[1,1];ax.axis('off')
 lines=[('Fixed guide throughout opening','All utensil rings stay on the smooth barrel.\nThe cap can loosen without withdrawing it.'),('Captured, with a cleaning release','The recessed keeper retains the cap and\nwasher; remove it clockwise to dismantle.'),('Latched cover and thumb rest','Push the key, quarter-turn, then withdraw.\nIts flat head also drives the keeper.'),('Print the small mechanism test first','Digital geometry checked. Printed fit,\nstrength and handling remain untested.')]
 y=.95
 for title,body in lines:
  ax.text(.05,y,title,fontsize=13,weight='bold',color='#253b43',va='top');ax.text(.05,y-.08,body,fontsize=11,color='#50585b',va='top',linespacing=1.5);y-=.255
 plt.subplots_adjust(left=.02,right=.98,top=.87,bottom=.035,hspace=.14,wspace=.08)
 fig.savefig(ROOT/'assembly-preview.png',dpi=160,facecolor=fig.get_facecolor())
 print(ROOT/'assembly-preview.png')

if __name__=='__main__':main()
