"""Run beside optimal_transport.py: python make_overview.py --seed 67"""
import argparse
import os
os.environ.setdefault('MPLCONFIGDIR', '/tmp/sps-matplotlib')
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
import optimal_transport as ot

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--seed', type=int, default=67)
args = parser.parse_args()
plt.rcParams.update({'font.family':'DejaVu Sans','font.size':12, 'axes.spines.top':False, 'axes.spines.right':False})
fig, axs = plt.subplots(2,2,figsize=(16,10))
fig.subplots_adjust(left=.07,right=.96,top=.83,bottom=.15,hspace=.49,wspace=.23)
fig.suptitle('Four worlds delivery challenge.',x=.07,y=.97,ha='left',fontsize=27,fontweight='bold',color='#172C46')
fig.text(.07,.915,f'Public instance · seed {args.seed}', fontsize=14,color='#526277')
titles={'disk':'Disk · straight-line distance','sphere':'Sphere · great-circle distance','torus':'Flat torus · wrap across both edge pairs','klein':'Flat Klein bottle · wrap with a reflection'}
for ax,(name,inst) in zip(axs.flat,ot.generate_instances(args.seed).items()):
    L=inst.scale_m
    if name=='sphere':
        xs,ys=np.linspace(-180,180,500),np.linspace(-90,90,250)
        X,Y=np.meshgrid(xs,ys)
        lo,la=np.deg2rad(X),np.deg2rad(Y)
        P=L*np.stack([np.cos(la)*np.cos(lo),np.cos(la)*np.sin(lo),np.sin(la)],axis=-1)
        def project(p):
            p=np.atleast_2d(p)
            return np.column_stack([np.rad2deg(np.arctan2(p[:,1],p[:,0])),np.rad2deg(np.arcsin(np.clip(p[:,2]/L,-1,1)))])
        ax.set(xlabel='Longitude (degrees)',ylabel='Latitude (degrees)',xticks=[-180,-90,0,90,180],yticks=[-90,0,90])
        mask=np.zeros(X.shape,bool)
        for c,r in zip(inst.ocean_centers,inst.ocean_radii_m):
            mask |= L*np.arccos(np.clip(np.sum(P*c,axis=-1)/L**2,-1,1))<=r
    else:
        lower=-L if name=='disk' else 0
        xs=ys=np.linspace(lower/1000,L/1000,450)
        X,Y=np.meshgrid(xs,ys)
        P=np.stack([X*1000,Y*1000],axis=-1)
        mask=np.zeros(X.shape,bool)
        for c,r in zip(inst.ocean_centers,inst.ocean_radii_m):
            if name=='disk':
                d2=np.sum((P-c)**2,axis=-1)
            elif name=='torus':
                d2=np.sum(((P-c+L/2)%L-L/2)**2,axis=-1)
            else:
                d2=np.full(X.shape,np.inf)
                for n in range(-2,3):
                    for m in range(-2,3):
                        lift=np.array([(-1)**n*c[0]+m*L,c[1]+n*L])
                        d2=np.minimum(d2,np.sum((P-lift)**2,axis=-1))
            mask |= d2<=r*r
        def project(p): return np.atleast_2d(p)/1000
        ax.set(xlabel='x (km)',ylabel='y (km)',aspect='equal')
    bg=np.where(mask,1.,0.)
    if name=='disk':
        bg=np.ma.masked_where(X**2+Y**2>(L/1000)**2,bg)
        ax.add_patch(plt.Circle((0,0),L/1000,fill=False,color='#9BA8B7',lw=1))
    from matplotlib.colors import ListedColormap
    ax.pcolormesh(X,Y,bg,cmap=ListedColormap(['#F4F1E9','#B8DCEB']),vmin=0,vmax=1,shading='auto',rasterized=True,zorder=0)
    cp=project(inst.checkpoints)
    ax.scatter(*cp.T,s=13,marker='x',c='#8795A4',linewidths=.7,zorder=2)
    ep=project(inst.endpoints)
    for island in [False,True]:
        sel=inst.endpoint_island_mask==island
        ax.scatter(*ep[sel].T,s=inst.endpoint_demand_kg[sel]*.24,marker='^' if island else 'o',c='#A2478A' if island else '#172C46',edgecolors='white',linewidths=.65,zorder=3)
    depot=project(inst.start)
    ax.scatter(*depot.T,s=210,marker='*',c='#E79025',edgecolors='#172C46',linewidths=.8,zorder=5,clip_on=False)
    ax.set_xlim(xs[0],xs[-1]); ax.set_ylim(ys[0],ys[-1])
    ax.set_title(titles[name],loc='left',fontsize=14,fontweight='bold',pad=12,color='#172C46')
    ax.tick_params(labelsize=10)
    for spine in ax.spines.values(): spine.set_color('#ADB7C3')
handles=[Line2D([],[],marker='*',color='none',markerfacecolor='#E79025',markeredgecolor='#172C46',markersize=15,label='Depot'),Line2D([],[],marker='o',color='none',markerfacecolor='#172C46',markersize=8,label='Destination'),Line2D([],[],marker='^',color='none',markerfacecolor='#A2478A',markersize=9,label='Island flag (jet only)'),Line2D([],[],marker='x',color='#8795A4',linestyle='none',markersize=7,label='Checkpoint'),Patch(facecolor='#B8DCEB',label='Ocean region')]
fig.legend(handles=handles,loc='lower center',bbox_to_anchor=(.5,.047),ncol=5,frameon=False,fontsize=12)
out = Path.home() / 'sps_worlds_v2.png'
fig.savefig(str(out), dpi=180, facecolor='white')
print(f'Saved to: {out}')
