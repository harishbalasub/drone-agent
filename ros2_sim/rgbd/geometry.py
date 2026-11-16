"""Calibrated depth geometry, independent of training labels and object truth."""
import numpy as np


def localize_package(depth,mask,intrinsics,world_from_camera,package_height=.24,min_pixels=25):
    """Estimate package center from its visible horizontal top surface.

    Assumptions: registered optical-axis depth in meters, upright package on a
    horizontal support, known package height, downward-looking camera. Camera
    matrix converts Gazebo FLU camera coordinates to map; optical -> FLU is
    (Z,-X,-Y). Returns a top-surface estimate and explicit quality diagnostics.
    No ground-truth package pose or semantic sensor input is accepted.
    """
    depth=np.asarray(depth,dtype=np.float32);mask=np.asarray(mask,dtype=bool)
    if depth.shape!=mask.shape or depth.ndim!=2:raise ValueError('Depth and mask must have matching H×W shape')
    matrix=np.asarray(world_from_camera,dtype=float)
    if matrix.shape!=(4,4) or not np.isfinite(matrix).all():raise ValueError('Finite 4×4 camera transform required')
    fx,fy,cx,cy=[float(intrinsics[k]) for k in ['fx','fy','cx','cy']]
    if fx<=0 or fy<=0:raise ValueError('Positive focal lengths required')
    ys,xs=np.where(mask & np.isfinite(depth) & (depth>.05) & (depth<10))
    if len(xs)<min_pixels:raise ValueError('INSUFFICIENT_VALID_DEPTH')
    z=depth[ys,xs];x=(xs-cx)*z/fx;y=(ys-cy)*z/fy
    points=np.stack([z,-x,-y,np.ones_like(z)],axis=1)@matrix.T
    points=points[:,:3]
    # Top-plane selection rejects visible vertical faces and background leakage.
    top_z=float(np.quantile(points[:,2],.9))
    top=points[np.abs(points[:,2]-top_z)<.015]
    if len(top)<min_pixels:raise ValueError('TOP_SURFACE_NOT_OBSERVED')
    center=np.median(top,axis=0);center[2]-=package_height/2
    spread=np.median(np.abs(top-np.median(top,axis=0)),axis=0)
    return {'position':center.tolist(),'surface_position':(center+[0,0,package_height/2]).tolist(),
            'valid_pixels':len(points),'top_surface_pixels':len(top),'surface_mad_m':spread.tolist(),
            'assumptions':['upright package','known package height','visible horizontal top'],
            'source':'rgbd_geometry','uncertainty_calibrated':False}
