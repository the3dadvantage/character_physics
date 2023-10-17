import bpy
import numpy as np
from mathutils import Matrix as MAT


#### quaternions ####
def get_ar_quats(ar, quats=None):
    if quats is None:
        quats = np.empty((len(ar.pose.bones), 4), dtype=np.float32)
    ar.pose.bones.foreach_get('rotation_quaternion', quats.ravel())
    return quats


def get_ar_quats_world(ar, quats=None):
    if quats is None:
        world_quats = np.array([bo.matrix.to_quaternion() for bo in ar.pose.bones], dtype=np.float32)
        return world_quats
    # Or overwrite in place
    for i in range(quats.shape[0]):
        quats[i] = ar.pose.bones[i].matrix.to_quaternion()


#### quaternions ####
def set_ar_quats(ar, quats):
    ar.pose.bones.foreach_set('rotation_quaternion', quats.ravel())


#### quaternions ####
def set_ar_quats_world(ar, quats=None, locations=None):
    """Sets the world rotation correctly
    in spite of parent bones. (and constraints??)"""
    for i in range(len(ar.pose.bones)):
        bpy.context.view_layer.update()
        arm = ar.pose.bones[i].matrix
        nparm = np.array(arm)
        if quats is not None:
            qte = quat_to_euler(quats[i], factor=1)        
            nparm[:3, :3] = qte
        if locations is not None:
            nparm[:3, 3] = locations[i]
        ar.pose.bones[i].matrix = MAT(nparm)


#### armature ####
def get_bone_head(ar, head=None):
    """Return Nx3 coordinates for bone head"""
    if head is None:
        head = np.empty((len(ar.pose.bones), 3), dtype=np.float32)
    ar.pose.bones.foreach_get('head', head.ravel())
    return head


#### armature ####
def get_bone_tail(ar, tail=None):
    if tail is None:
        tail = np.empty((len(ar.pose.bones), 3), dtype=np.float32)
    ar.pose.bones.foreach_get('tail', tail.ravel())
    return tail


def get_bone_centers(ar, scale=0.5):
    """Get bone center or put scale
    in as an array calculating
    center of mass."""
    head = get_bone_head(ar)
    tail = get_bone_tail(ar)
    vec = (tail - head) * scale
    return head + vec


#### armature ####
def get_bone_locations(ar, location=None):
    """y and z are flipped on pose bones... why???"""
    
    if location is None:
        location = np.empty((len(ar.pose.bones), 3), dtype=np.float32)
        yz = np.empty((len(ar.pose.bones), 3), dtype=np.float32)
    
    ar.pose.bones.foreach_get('location', location.ravel())
    yz[:, 0] = location[:, 0]
    yz[:, 1] = -location[:, 2]
    yz[:, 2] = location[:, 1]
    return yz


#### armature ####
def get_bone_location_world(ar, location=None):
    """y and z are flipped on pose bones... why???"""
    
    if location is None:
        location = np.empty((len(ar.pose.bones), 3), dtype=np.float32)
        #yz = np.empty((len(ar.pose.bones), 3), dtype=np.float32)
    for e, bo in enumerate(ar.pose.bones):
        location[e] = np.array(bo.matrix, dtype=np.float32)[:3, 3]
    yz = location
    #ar.pose.bones.foreach_get('location', location.ravel())
    #yz[:, 0] = location[:, 0]
    #yz[:, 1] = -location[:, 2]
    #yz[:, 2] = location[:, 1]
    return yz


#### armature ####    
def set_bone_locations(ar, location, yz=None):
    if yz is None:
        yz = np.empty((len(ar.pose.bones), 3), dtype=np.float32)
    yz=location # override!
    yz[:, 0] = location[:, 0]
    yz[:, 1] = location[:, 2]
    yz[:, 2] = -location[:, 1]    
    ar.pose.bones.foreach_set('location', yz.ravel())
    ar.pose.bones[0].location = yz[0]
    ar.pose.bones[0].location = ar.pose.bones[0].location

#### armature ####    
def update_bones(ar):
    ar.pose.bones[0].location = ar.pose.bones[0].location
    


#### armature ####    
def get_relative_bones(ar):
    """Get the index relationships of
    bones by parent and child
    relationships."""
    
    # add an index property to each bone
    flat = []
    repeater = []
    for e, b in enumerate(ar.pose.bones):
        b['index'] = e
    
    relationships = []
    for bo in ar.pose.bones:
        rel = []
        related_count = len(b.children)
        if bo.parent is not None:
            flat += [bo.parent['index']]
            repeater += [bo['index']]
            rel += [bo.parent['index']] 
        for ch in bo.children:
            flat += [ch['index']]
            repeater += [bo['index']]
            rel += [ch['index']]
        relationships += [rel]
    
    #print(flat, "this is flat")
    #print(repeater, "this is repeater")
    return relationships


if False:
    head = get_bone_head(ar)
    tail = get_bone_tail(ar)
    location = get_bone_location(ar)

if False:
    location += 0.1
    set_bone_location(ar, location)
    update_bones(ar)


class Rigs():
    def __init__(self):
        self.name = "this is doable"
        self.target_ar = bpy.data.objects['target']
        self.phys_ar = bpy.data.objects['physics']
        self.tb_count = len(self.target_ar.pose.bones)
        self.t_quats = np.empty((self.tb_count, 4), dtype=np.float32)
        get_ar_quats_world(self.target_ar, self.t_quats)        
    
    def get_target_data(self):
        get_ar_quats_world(self.target_ar, self.t_quats)
        
        print(self.t_quats[0])
        ee.rotation_quaternion = self.t_quats[0]
        

if False:
    print()
    R = Rigs()
    print(R.name)
    print("=====================")
    R.get_target_data()
