import bpy
import numpy as np


def get_ar_quats(ar, quats=None):
    if quats is None:
        quats = np.empty((len(ar.pose.bones), 4), dtype=np.float32)
    ar.pose.bones.foreach_get('rotation_quaternion', quats.ravel())
    return quats


def set_ar_quats(ar, quats):
    ar.pose.bones.foreach_set('rotation_quaternion', quats.ravel())
    print(ar.pose.bones[0].rotation_quaternion)
    # looks like I have to set at least one
    #   bone this way to update all of them
    ar.pose.bones[0].rotation_quaternion = quats[0]

    

def add_quats(q1, q2, multi=False, normalize=False):
    """Add q1 and q2. ([w, x, y, z])."""
    
    if multi:
        v1 = q1[:, 1:]
        v2 = q2[:, 1:]
        w1 = q1[:, 0][:, None]
        w2 = q2[:, 0][:, None]
        v_result = np.cross(v1, v2) + w1 * v2 + w2 * v1
        w_result = w1 * w2 - (np.einsum('ij,ij->i', v1, v2))[:, None]
        q2[:, 0] = w_result.ravel()
        q2[:, 1:] = v_result
        result = q2
        
    else:
        v1 = q1[1:]
        v2 = q2[1:]
        w1 = q1[0]
        w2 = q2[0]

        v_result = np.cross(v1, v2) + w1 * v2 + w2 * v1
        w_result = w1 * w2 - (v1 @ v2)
        result = np.array([w_result, *v_result])
        
    if normalize:    
        result /= np.linalg.norm(result)

    return result


e1 = bpy.data.objects['e1']
e2 = bpy.data.objects['e2']
e3 = bpy.data.objects['e3']
ar = bpy.data.objects['ar']

q1 = np.array(e1.rotation_quaternion)
q2 = np.array(e2.rotation_quaternion)


ar_quats = get_ar_quats(ar, quats=None)
eq = np.empty((2,4), dtype=np.float32)
eq[0] = q1
eq[1] = q2



if True:
    q3 = add_quats(q1, q2)
    e3.rotation_quaternion = q3


if True:
    rot = add_quats(ar_quats, eq, multi=True)
    set_ar_quats(ar, rot)


def get_quat(rad, axis):
    u_axis = axis / np.sqrt(axis @ axis)
    theta = (rad * 0.5)
    w = np.cos(theta)
    q_axis = u_axis * np.sin(theta)
    return w, q_axis

 
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


def q_rotate(co, w, axis):
    """Takes an N x 3 numpy array and returns that array rotated around
    the axis by the angle in radians w. (standard quaternion)"""    
    move1 = np.cross(axis, co)
    move2 = np.cross(axis, move1)
    move1 *= w
    return co + (move1 + move2) * 2


def quat_to_euler(q, m3):
    rot_m3 = q_rotate(m3.T, q[0], q[1:])
    return m3 # or is it m3.T?


def quaternion_subtract(w1, v1, w2, v2):
    """Get the quaternion that rotates one object to another"""
    w = w1 * w2 - np.dot(v1, v2)
    v = w1 * v2 + w2 * v1 + np.cross(v1, v2)
    return w, -v



#so what if I used a combination of
#matrix and quat.
#I could rotate the 3x3 by the quat using
#slerping. Then I take that location (a fraction of the move)
#and target it from the 3x3 of a bone end.

#So... the head and tail of each bone
#gets its own matrix.

#the connected bones will push that
#matrix and rotate it around.


#the target pose will contain a matrix relative
#to each bone


#each bone at each target pose will be
#trying to get to a particular rotation
#relative to connected bones.

#So I generate a target rotation
#relative 













#When I apply a rotation to a bone I think I need to rotate the bone
#with the origin in the center of mass.
#The linear solve will then apply forces that will counter the rotation.
