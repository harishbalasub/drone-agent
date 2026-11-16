"""Explicit surface and near-gripper observation domains; neither proves grasp."""
import math
import numpy as np

def filter_depth_mask(depth,mask,grasp_candidate=False):
    return np.asarray(mask,dtype=bool) & np.isfinite(depth) & (depth>(.08 if grasp_candidate else .35))

def validate_height(position,grasp_candidate=False,expected_payload_z=None):
    if grasp_candidate:
        if expected_payload_z is None or not math.isfinite(expected_payload_z) or abs(position[2]-expected_payload_z)>.12:
            raise ValueError('PACKAGE_NOT_AT_GRIPPER_HEIGHT')
    elif min(abs(position[2]-z) for z in [.12,.62])>.08:
        raise ValueError('UNSUPPORTED_PACKAGE_SUPPORT_HEIGHT')
