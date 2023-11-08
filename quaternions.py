import bpy
import numpy as np
import sys
import os
from mathutils import Matrix as MAT

psep = os.path.sep
path = '/home/rich/Desktop/cloth/character_engine'
sys.path.append(path)

try:
    U = bpy.data.texts['utils.py'].as_module()
    
except:
    import utils as U
    importlib.reload(U)

NQ = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
BNQ = np.array([0.7071067690849304, 0.7071067690849304, 0.0, 0.0], dtype=np.float32)
BNM3 = np.array([[ 1.0,  0.0,  0.0],
                 [ 0.0,  0.0, -1.0],
                 [ 0.0,  1.0, -0.0]], dtype=np.float32)

INV = np.linalg.inv(BNM3)
# print(INV @ BNM3)

#### quaternions ####
def get_quat(rad, axis, normalize=False):
    if normalize:
        axis = axis / np.sqrt(axis @ axis)
    theta = (rad * 0.5)
    w = np.cos(theta)
    q_axis = axis * np.sin(theta)
    return w, q_axis


#### quaternions ####
def get_quats(rad, axis, normalize=False, out=None):
    if normalize:    
        axis = axis / np.sqrt(np.einsum('ij,ij->i', axis, axis))[:, None]
    theta = (rad * 0.5)
    if out is None:
        out = np.empty((rad.shape[0], 4), dtype=np.float32)

    out[:, 0] = np.cos(theta)
    out[:, 1:] = axis * np.sin(theta)[:, None]
    out[np.any(np.isnan(out), axis=1)] = NQ
    dot = np.einsum('ij,ij->i', out, out)
    not_normal = np.round(dot, 4) != 1
    return out


def get_edge_match_quats(v1, v2, out, factor=None):
    """Get the quats that will rotate
    v1 to match v2"""
    dots = U.compare_vecs(v1, v2)
    angle = np.arccos(dots)
    if factor is not None:
        angle *= factor
    axis = np.cross(v2, v1)
    out = get_quats(angle, axis, normalize=True, out=out)
    return out
    

def _get_edge_match_quats(e1=None, e2=None, ue1=None, ue2=None, out=None, factor=None, r_ue1=False):
    """e1 and e2 are Nx2x3.
    Generate the quats that
    would rotate the e1, or
    bones, to match e2"""
    if ue1 is None:    
        e1_vecs = e1[:, 1] - e1[:, 0]
        ue1 = U.u_vecs(e1_vecs)
    if ue2 is None:
        e2_vecs = e2[:, 1] - e2[:, 0]
        ue2 = U.u_vecs(e2_vecs)
    
    print(ue1.shape, "ue1 shape")
    print(ue2.shape, "ue2 shape")
    dots = U.compare_vecs(ue1, ue2)
    angle = np.arccos(dots)
    if factor is not None:
        angle *= factor
    axis = np.cross(ue2, ue1)
    if out is not None:
        get_quats(angle, axis, normalize=True, out=out)
        if r_ue1:
            return ue1
        return
    w, quax = get_quats(angle, axis, normalize=True)
    

#### quaternions ####
def extract_quat_angle(q):
    if len(q.shape) == 1:
        return np.arccos(q[0]) * 2
    return np.arccos(q[:, 0]) * 2


#### quaternions ####
def partial_quat(q, factor=0.5):
    """Factor can be an array"""
    angle = extract_quat_angle(q)
    angle *= factor
    if len(q.shape) == 1:
        part_q = get_quat(angle, q[1:])
        return part_q
    part_q = get_quats(angle, q[:, 1:], out=q)
    return part_q


def m3_to_b_quats(m3, out=None):
    """Convert an m3 array to list
    of blender quaternion types."""
    b_quats = []
    for m in m3:
        b_quats += [MAT(m).to_quaternion()]
    return b_quats


def average_b_quat(quats):
    """Adds blender quaternions
    resulting in average quat"""
    sq = quats[0]
    for q in quats[1:]:
        sq = sq + q
    return sq


#### quaternions ####
def q_rotate(co, w, axis):
    """Takes an N x 3 numpy array and returns that array rotated around
    the axis by the angle in radians w. (standard quaternion)"""    
    move1 = np.cross(axis, co)
    move2 = np.cross(axis, move1)
    move1 *= w
    return co + (move1 + move2) * 2


def rotate_m3(m3, quat):
    w = quat[0]
    axis = quat[1:]
    m3 = q_rotate(m3.T, w, axis)
    return m3.T


def rotate_m3s(m3s, quat, out=None):
    if out is None:
        #out = np.empty_like(m3s)
        out = m3s
    for i in range(m3s.shape[0]):
        w = quat[i][0]
        axis = -quat[i][1:]
        rm3 = q_rotate(m3s[i].T, w, axis)
        out[i] = rm3.T
    return out


def rotate_matrices(matrices, quats):
    ms = matrices.shape
    cm = np.empty((ms[0], 3, 3), dtype=np.float32)
    for i in range(matrices.shape[0]):
        co = matrices[i][:3, :3].T
        w = quats[i][0]
        axis = -quats[i][1:]
        m3 = q_rotate(co, w, axis)
        cm[i] = m3.T
    return cm


#### quaternions ####
def quat_to_m3(q, m3=None):
    if m3 is None:
        m3 = np.eye(3)
    rot_m3 = q_rotate(m3.T, q[0], q[1:])
    return rot_m3.T # is it m3.T or just m3?


def m3_multiply(m31, m32):
    mult = m31.T @ m32.T
    return mult.T


if False:
    target = bpy.data.objects["target"]
    phys = bpy.data.objects["phys"]
    test_write = bpy.data.objects["test_write"]

    quats = get_ar_quats_world(target)
    set_ar_quats_world(test_write, quats)#, empties)
    set_ar_quats_world(phys, quats)#, empties)


#### quaternions ####
def quaternion_add(w1, v1, w2, v2):
    '''Get the combined rotation of two quaternions.'''
    w = w1 * w2 - np.dot(v1, v2)
    xyz = w1 * v2 + w2 * v1 + np.cross(v1, v2)
    return w, xyz


#### quaternions ####    
def add_quats(q1, q2, normalize=False, out=None):
    """Add q1 and q2. ([w, x, y, z])."""
    
    if out is None:
        out = np.empty_like(q1)

    for i in range(q1.shape[0]):
        w1 = q1[i][0]
        w2 = q2[i][0]
        v1 = q1[i][1:]
        v2 = q2[i][1:]
        
        w, axis = quaternion_add(w1, v1, w2, v2)
        out[i][0] = w
        out[i][1:] = axis
        
    return out    


    v1 = q1[:, 1:]
    v2 = q2[:, 1:]
    w1 = q1[:, 0][:, None]
    w2 = q2[:, 0][:, None]
    v_result = np.cross(v1, v2) + w1 * v2 + w2 * v1
    w_result = w1 * w2 - (np.einsum('ij,ij->i', v1, v2))[:, None]
        
    out[:, 0] = w_result.ravel()
    out[:, 1:] = v_result
            
    if normalize:    
        out /= np.linalg.norm(result)[:, None]

    return out


#### quaternions #### 
def get_quat_2(v1, v2, rot_mag=1, convert=True, axis=None):
    """Returns the quaternion that will rotate the object based on two vectors.
    If axis is provided: it can be non-unit. The quaternion will rotate around the axis
    until v1 lines up with v2 on the axis    
    To rotate part way rot mag can be a value between -1 and 1.
    For specific angles, rot_mag can be the cosine of the angle in radians (haven't tested this theory)"""  
    if convert: # if vectors are non-unit
        v1 = v1 / np.sqrt(v1 @ v1)
        v2 = v2 / np.sqrt(v2 @ v2)
    if axis is None:
        mid = v1 + v2 * rot_mag # multiply for slerping  
        Umid = mid / np.sqrt(mid @ mid)
        w = Umid @ v1
        xyz = np.cross(v1, Umid)
        return w, xyz
    vc1 = np.cross(axis, v1)        
    vc2 = np.cross(axis, v2) 
    v1 = vc1 / np.sqrt(vc1 @ vc1)
    v2 = vc2 / np.sqrt(vc2 @ vc2)
    mid = v1 + v2 * rot_mag # multiply for slerping  
    Umid = mid / np.sqrt(mid @ mid)
    w = Umid @ v1
    xyz = np.cross(v1, Umid)
    return w, xyz


#### quaternions ####
def get_quat_from_perp_vecs(v1, v2):
    x = np.array([1, 0, 0])
    z = np.array([0, 0, 1])
    uv1 = v1 / np.sqrt(v1 @ v1)
    norm = np.cross(v1, v2)
    uv3 = norm / np.sqrt(norm @ norm)
    w1, axis1 = get_quat_2(uv3, z)
    rot = q_rotate(uv1, w1, axis1)
    w2, axis2 = get_quat_2(rot, x)
    n_w, n_xyz = quaternion_add(-w1, axis1, -w2, axis2)


#### quaternions ####
def quaternion_subtract(w1, v1, w2, v2):
    """Get the quaternion that rotates q1 to match q2"""
    w = w1 * w2 - np.dot(v1, v2)
    v = w1 * v2 + w2 * v1 + np.cross(v1, v2)
    return w, -v


def quaternions_subtract(q1, q2, out=None):
    """Get the quaternions that rotate q1 to match q2"""
    if out is None:
        out = np.empty_like(q1)
    
    out[:, 0] = (q1[:, 0] * q2[:, 0]) - np.einsum('ij,ij->i', q1[:, 1:], q2[:, 1:])
    v = (q1[:, 0][:, None] * q2[:, 1:]) + (q2[:, 0][:, None] * q1[:, 1:]) + np.cross(q1[:, 1:], q2[:, 1:])
    #out[:, 1:] = -v
    out[:, 1:] = -v
    return out

# ----- end quaternions ----- #

    
#### physics ####
def quat_vel(ar, q_vel):
    """Keep track of the angular velocity
    as an array of quaternions. Manage
    offset rotation around center
    of mass."""
    
    # so we measure the rotation at the start
    # then we subtract the rotation at the end    
    q_vel_start = np.zeros((len(ar.pose.bones)), dtype=np.float32)
    q_vel_start[:, 0] = 1.0
    # we need to rotate around the center of mass
    # so we get the location of center of mass,
    # then we rotate then we move so the center of mass lines up again
    #   then we iterate and start over.
    # could automate finding center of mass by measureing vertex weights
    #   from the mesh for that bone then doing volumer or whatever
    #   but for now I think bullet used the object origins.
    #   so I can think of the mid point of the bone like an object
    #   origin in the game physics.
    
    #cant figure out how to combine linear bone move and angular bone move
    #I was thinking vectors at each end but I dont know what gets put
    #into angular versus what moves the object on a linar path along
    #its center of mass.


#### testing ####
if False:
    eee = bpy.data.objects['eee']
    ee = bpy.data.objects['ee']
    e1 = bpy.data.objects['e1']
    e2 = bpy.data.objects['e2']
    e3 = bpy.data.objects['e3']
    ar = bpy.data.objects['ar']

    get_relative_bones(ar)

    q1 = np.array(e1.rotation_quaternion)
    q2 = np.array(e2.rotation_quaternion)

    ar_quats = get_ar_quats(ar, quats=None)
    ar_quats_world = get_ar_quats_world(ar, quats=None)
    #print(np.round(ar_quats, 4), "quats")
    ee.rotation_quaternion = ar_quats_world[0]
    eee.rotation_quaternion = ar_quats_world[1]
    #print(ar_quats_world)

    eq = np.empty((2,4), dtype=np.float32)
    eq[0] = q1
    eq[1] = q2


#if True:
if False:
    q3 = add_quats(q1, q2)
    e3.rotation_quaternion = q3


#if True:
if False:
    rot = add_quats(ar_quats, eq, multi=True)
    set_ar_quats(ar, rot)






