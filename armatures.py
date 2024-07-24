import bpy
import numpy as np
from mathutils import Matrix as MAT
import sys
import os
import importlib

try:
    from character_physics import utils as U
    importlib.reload(U)
    
except:
    U = bpy.data.texts['utils.py'].as_module()


def compare_rig_spaces(rig1, rig2):
    """Compare the head and tail of the active bone respecting
    each rig's world matrix. Return the index of the bone in rig2 whose
    head and tail together are closest to the active bone."""

    active_bone = bpy.context.active_pose_bone
    if active_bone:
            
        local_matrix = np.linalg.inv(rig1.matrix_world) @ np.array(rig2.matrix_world, dtype=np.float32)

        hco = get_bone_head(rig2) @ local_matrix[:3, :3].T
        tco = get_bone_tail(rig2) @ local_matrix[:3, :3].T
        hco += local_matrix[:3, 3]
        tco += local_matrix[:3, 3]

        bhco = np.array(active_bone.head, dtype=np.float32)
        thco = np.array(active_bone.tail, dtype=np.float32)
        
        h_dif = bhco - hco
        t_dif = thco - tco
        h_dist = U.measure_vecs(h_dif)
        t_dist = U.measure_vecs(t_dif)
        
        sumingtons = h_dist + t_dist
        closest = np.argmin(sumingtons)
        return closest        


def copy_roll(rig, bone_index):

    modes = U.manage_modes()
    active_bone_name = bpy.context.active_pose_bone.name
    bpy.ops.object.mode_set(mode='EDIT')
    roll = bpy.context.object.data.edit_bones[active_bone_name].roll
    bpy.ops.object.mode_set(mode='OBJECT')
    bpy.context.view_layer.objects.active = rig
    bpy.ops.object.mode_set(mode='EDIT')
    rig.data.edit_bones[bone_index].roll = roll    
    U.manage_modes(modes)
    

def manage_parents(phys, pose=None, phys_parents=None):
    """Set phys parents to match pose before rig setup"""
    original_selected = bpy.context.selected_objects[:]
    original_active = bpy.context.active_object
    original_object_modes = {ob: ob.mode for ob in original_selected}

    bpy.ops.object.mode_set(mode='OBJECT')

    if phys_parents:
        bpy.context.view_layer.objects.active = phys
        bpy.ops.object.select_all(action='DESELECT')
        phys.select_set(True)
        bpy.ops.object.mode_set(mode='EDIT')
        for i in range(len(phys.data.edit_bones)):
            phys.data.edit_bones[i].parent = phys_parents[i]

        bpy.ops.object.mode_set(mode='OBJECT')
        bpy.ops.object.select_all(action='DESELECT')
        for ob in original_selected:
            ob.select_set(True)
            bpy.context.view_layer.objects.active = ob
            bpy.ops.object.mode_set(mode=original_object_modes[ob])
        return

    bpy.context.view_layer.objects.active = pose
    bpy.ops.object.select_all(action='DESELECT')
    pose.select_set(True)
    bpy.ops.object.mode_set(mode='EDIT')

    for i in range(len(pose.data.edit_bones)):
        pose.data.edit_bones[i]['index'] = i
        print(i)
        
    parents = {}
    for b in pose.data.edit_bones:
        if b.parent:    
            parents[b['index']] = b.parent['index']
        else:
            parents[b['index']] = None
    
    bpy.ops.object.mode_set(mode='OBJECT')
    
    bpy.context.view_layer.objects.active = phys
    bpy.ops.object.select_all(action='DESELECT')
    phys.select_set(True)
    bpy.ops.object.mode_set(mode='EDIT')

    for i in range(len(phys.data.edit_bones)):
        phys.data.edit_bones[i]['index'] = i

    phys_parents = {}
    for b in phys.data.edit_bones:
        if b.parent:
            phys_parents[b['index']] = b.parent['index']
        else:
            phys_parents[b['index']] = None    
    
    for i in range(len(phys.data.edit_bones)):
        if parents[i] is not None:    
            phys.data.edit_bones[i].parent = phys.data.edit_bones[parents[i]]

    bpy.ops.object.mode_set(mode='OBJECT')

    bpy.ops.object.select_all(action='DESELECT')
    for ob in original_selected:
        ob.select_set(True)
        bpy.context.view_layer.objects.active = ob
        bpy.ops.object.mode_set(mode=original_object_modes[ob])
    
    print(phys_parents, "this")
    for k, v in phys_parents.items():
        print(k)
        
    return phys_parents


def disconnect_bones(ar):
    original_selected = bpy.context.selected_objects[:]
    original_active = bpy.context.active_object
    original_object_modes = {ob: ob.mode for ob in original_selected}    
    bpy.ops.object.mode_set(mode='OBJECT')

    bpy.context.view_layer.objects.active = ar
    bpy.ops.object.select_all(action='DESELECT')
    ar.select_set(True)
    bpy.ops.object.mode_set(mode='EDIT')
    
    for bone in ar.data.edit_bones:
        if bone.parent:    
            bone.use_connect = False

    bpy.ops.object.mode_set(mode='OBJECT')

    bpy.ops.object.select_all(action='DESELECT')
    for ob in original_selected:
        ob.select_set(True)
        bpy.context.view_layer.objects.active = ob
        bpy.ops.object.mode_set(mode=original_object_modes[ob])
     

def connect_bones(armature_ob, connected=False, clear_parents=False):
    """Switch armature bones between connected or disconnected"""
    
    pose_target = armature_ob.CP_props.pose_target
    if pose_target:
        for e, b in enumerate(pose_target.pose.bones):
            b['index'] = e
    
    original_selected = bpy.context.selected_objects[:]
    original_active = bpy.context.active_object
    original_object_modes = {ob: ob.mode for ob in original_selected}
    
    if bpy.context.mode == 'POSE':
        bpy.ops.object.mode_set(mode='OBJECT')
    bpy.context.view_layer.objects.active = armature_ob
    bpy.ops.object.select_all(action='DESELECT')
    armature_ob.select_set(True)

    bpy.ops.object.mode_set(mode='EDIT')
    
    for e, b in enumerate(armature_ob.data.edit_bones):
        b['index'] = e
    
    parents = {}
    for i in range(len(armature_ob.data.edit_bones)):
        bone = armature_ob.data.edit_bones[i]
        if connected:
            if pose_target:
                pose_parent = pose_target.pose.bones[i].parent
                parents[i] = None
                if bone.parent:
                    parents[i] = bone.parent['index']
                if pose_parent:
                    idx = pose_parent['index']
                    bone.parent = armature_ob.data.edit_bones[idx]
            else:    
                if not bone.parent:
                    for b in armature_ob.data.edit_bones:
                        if b != bone:
                            dif = np.array(bone.head) - np.array(b.tail)
                            dist = np.nan_to_num(np.sqrt(dif @ dif))
                            print(dist)
                            if dist <= 0.0000000001:
                                bone.parent = b
            
        if bone.parent:    
            bone.use_connect = connected
        if clear_parents:
            bone.parent = None
    
    bpy.ops.object.mode_set(mode='OBJECT')

    bpy.ops.object.select_all(action='DESELECT')
    for ob in original_selected:
        ob.select_set(True)
        bpy.context.view_layer.objects.active = ob
        bpy.ops.object.mode_set(mode=original_object_modes[ob])
    return parents
    
    
#### quaternions ####
def get_ar_quats(ar, quats=None):
    """Returns rotation relative to edit bone.
    In other words: useless."""
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


def get_ar_matrix_world(ar, m3=False):
    """Returns whirled matrix."""
    world_matrix = np.array([bo.matrix for bo in ar.pose.bones], dtype=np.float32)
    if m3:
        return world_matrix[:, :3, :3]
    return world_matrix


#### quaternions ####
def set_ar_quats(ar, quats):
    ar.pose.bones.foreach_set('rotation_quaternion', quats.ravel())


def set_b_matricies(ar, bm3s, locations=None):
    for i in range(len(ar.pose.bones)):
        bpy.context.view_layer.update()
        arm = ar.pose.bones[i].matrix
        nparm = np.array(arm)
        nparm[:3, :3] = bm3s[i]
        if locations is not None:
            nparm[:3, 3] = locations[i]
        #mat = MAT(nparm)
        mat = nparm.T
        ar.pose.bones[i].matrix = mat
    bpy.context.view_layer.update()


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


def get_bone_vecs(ar):
    head = get_bone_head(ar)
    tail = get_bone_tail(ar)
    return (tail - head)


def get_bone_eco(ar, out=None):
    if out is None:
        bc = len(ar.pose.bones)
        out = np.empty((bc, 2, 3), dtype=np.float32)
    head = get_bone_head(ar)
    tail = get_bone_tail(ar)
    out[:, 0] = head
    out[:, 1] = tail
    return out
        

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
    """Get locs from matrix"""
    if location is None:
        location = np.empty((len(ar.pose.bones), 3), dtype=np.float32)
    for e, bo in enumerate(ar.pose.bones):
        location[e] = np.array(bo.matrix, dtype=np.float32)[:3, 3]
    return location


def fast_m4(ar, m4=None):
    if m4 is None:
        bc = len(ar.pose.bones)
        m4 = np.empty((bc, 4, 4), dtype=np.float32)        
    ar.pose.bones.foreach_get('matrix', m4.ravel())
    return np.swapaxes(m4, 1, 2)


def just_m4(ar, m4=None):
    if m4 is None:
        bc = len(ar.pose.bones)
        m4 = np.empty((bc, 4, 4), dtype=np.float32)        
    ar.pose.bones.foreach_get('matrix', m4.ravel())
    return m4


def just_set_m4(m4, ar):
    ar.pose.bones.foreach_set('matrix', m4.ravel())
    ar.pose.bones[0].location = ar.pose.bones[0].location


def fast_m3(ar, m3=None, T=True):
    if m3 is None:
        bc = len(ar.pose.bones)
        m3 = np.empty((bc, 4, 4), dtype=np.float32)
    ar.pose.bones.foreach_get('matrix', m3.ravel())
    if T:    
        return np.swapaxes(m3[:, :3, :3], 1, 2)
    return m3[:, :3, :3]


def get_ar_m3(ar, m3=None):
    bc = len(ar.pose.bones)
    if m3 is None:
        m3 = np.empty((bc, 3, 3), dtype=np.float32)
    for e, bo in enumerate(ar.pose.bones):
        m3[e] = np.array(bo.matrix, dtype=np.float32)[:3, :3]
    return m3


def fast_set_m3(m4, ar, m3, locs):
    m4[:, :3, 3] = locs
    m4[:, :3, :3] = m3
    m4 = np.swapaxes(m4, 1, 2)
    ar.pose.bones.foreach_set('matrix', m4.ravel())
    
    ar.pose.bones[0].location = ar.pose.bones[0].location


def fast_set_m4(m4, ar):
    m4 = np.swapaxes(m4, 1, 2)
    ar.pose.bones.foreach_set('matrix', m4.ravel())
    
    ar.pose.bones[0].location = ar.pose.bones[0].location


#### quaternions ####
def set_ar_m3_world(ar, m3, locations=None, return_quats=False):
    """Sets the world rotation correctly
    in spite of parent bones. (and constraints??)"""
    if return_quats:
        quats = np.empty((m3.shape[0], 4), dtype=np.float32)
    for i in range(len(ar.pose.bones)):
        bpy.context.view_layer.update()
        arm = ar.pose.bones[i].matrix
        nparm = np.array(arm)
        nparm[:3, :3] = m3[i]
        if locations is not None:
            nparm[:3, 3] = locations[i]
        #mat = MAT(nparm)
        mat = nparm.T
        if return_quats:
            quats[i] = mat.to_quaternion()
        ar.pose.bones[i].matrix = mat
    
    bpy.context.view_layer.update()
    if return_quats:    
        return quats
    

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
def get_relative_bones(ar, flatten=True, return_repeater=True):
    """Get the index relationships of
    bones by parent and child
    relationships."""
    
    siblings=True # gets relative children by common parent
    
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
        if siblings:
            if bo.parent is not None:
                brothers = [b for b in bo.parent.children if b != bo]
                for br in brothers:
                    repeater += [bo['index']]
                    rel += [br['index']]
                    flat += [br['index']]
        relationships += [rel]
    
    if return_repeater:
        if flatten:    
            return flat, repeater
        return relative, repeater
    return relationships


def make_ar_mesh(ar, pose_target=None):

    if pose_target:        
        relative, repeater = get_relative_bones(pose_target)
    else:
        relative, repeater = get_relative_bones(ar)
    
    
    head = get_bone_head(ar)
    tail = get_bone_tail(ar)
        
    for bo in ar.pose.bones:
        bo['head'] = None
        bo['tail'] = None
        edges = []
    
    vert_count = 0
    co = []
    index = []
    for e, bo in enumerate(ar.pose.bones):
        edge = []
        
        if bo.parent is not None:
            if bo.parent['tail'] is not None:
                edge = bo.parent['tail']
                bo['head'] = bo.parent['tail']
            if bo.parent is None:
                bo['head'] = vert_count
                vert_count += 1
                index += [vert_count]
                co += [head[bo['index']]]
                
        if bo.parent is None:
            bo['head'] = vert_count
            vert_count += 1
            index += [vert_count]
            co += [head[bo['index']]]
            
        if len(bo.children) == 0:
            bo['tail'] = vert_count
            vert_count += 1
            index += [vert_count]
            co += [tail[bo['index']]]
        
        if len(bo.children) != 0:
            heads = np.array([ch['head'] for ch in bo.children])
            
            if np.any(heads != None):
                idx = heads[heads != None][0]
                for ch in bo.children:
                    ch['head'] = idx
                    bo['tail'] = idx
            
            if np.all(heads == None):
                bo['tail'] = vert_count
                vert_count += 1
                index += [vert_count]
                co += [tail[bo['index']]]
        
        edge = [bo['head'], bo['tail']]
        edges += [edge]
    
    ob = None
    name = ar.name + "_CE_physics_mesh"

    if name in bpy.data.objects:
        ob = bpy.data.objects[name]
        if ob.data.is_editmode:
            bpy.ops.object.mode_set()
        
    phys_mesh = U.link_mesh(co, edges, faces=[], name=name, ob=ob)
    return phys_mesh, edges, co, index, relative, repeater


def make_mix_mesh(ar, ph):
    
    for e, b in enumerate(ar.pose.bones):
        b['index'] = e

    head = get_bone_head(ar)
    tail = get_bone_tail(ar)
        
    for bo in ar.pose.bones:
        bo['head'] = None
        bo['tail'] = None
        edges = []
    
    vert_count = 0
    co = []
    index = []
    for e, bo in enumerate(ar.pose.bones):
        edge = []
        
        if bo.parent is not None:
            if bo.parent['tail'] is not None:
                edge = bo.parent['tail']
                bo['head'] = bo.parent['tail']
            if bo.parent is None:
                bo['head'] = vert_count
                vert_count += 1
                index += [vert_count]
                co += [head[bo['index']]]
                
        if bo.parent is None:
            bo['head'] = vert_count
            vert_count += 1
            index += [vert_count]
            co += [head[bo['index']]]
            
        if len(bo.children) == 0:
            bo['tail'] = vert_count
            vert_count += 1
            index += [vert_count]
            co += [tail[bo['index']]]
        
        if len(bo.children) != 0:
            heads = np.array([ch['head'] for ch in bo.children])
            
            if np.any(heads != None):
                idx = heads[heads != None][0]
                for ch in bo.children:
                    ch['head'] = idx
                    bo['tail'] = idx
            
            if np.all(heads == None):
                bo['tail'] = vert_count
                vert_count += 1
                index += [vert_count]
                co += [tail[bo['index']]]
        
        edge = [bo['head'], bo['tail']]
        edges += [edge]
    
    ob = None
    name = ar.name + "_Ce_physics_mesh"
    
    if name in bpy.data.objects:
        bpy.data.objects.remove(bpy.data.objects[name])

        #bpy.data.objects[name].name = "dead_physics_mesh"
        #if ob.data.is_editmode:
            #bpy.ops.object.mode_set()
    
    mesh_bone_idx = np.array(edges, dtype=np.int32)[:, 0]
    mesh_bone_tail_idx = np.array(edges, dtype=np.int32)[:, 1]
    npco = np.array(co, dtype=np.float32)
    y_vidx = np.arange(npco.shape[0])    
    # x cross thingy
    factor = 0.5
    mix_vidx = np.arange(npco.shape[0])
    mesh_bone_co = npco[mesh_bone_idx]
    mesh_tails_co = npco[mesh_bone_tail_idx]
    mid_vecs = (mesh_tails_co - mesh_bone_co) * factor
    middles = mesh_bone_co + mid_vecs
    
    x_vecs = ph.physics_m3[:, :, 0]
    x_edges = np.empty((x_vecs.shape[0], 2, 3), dtype=np.float32)
    x_edges[:, 1] = middles + (x_vecs * factor)
    x_edges[:, 0] = middles - (x_vecs * (1 - factor))
    eidx = np.arange(x_edges.shape[0] * 2, dtype=np.int32)
    eidx.shape = (x_edges.shape[0], 2)
    x_edges.shape = (x_edges.shape[0] * 2, 3)    
    
    co_with_x = np.array(co, dtype=np.float32).tolist() + x_edges.tolist()
    edges_with_x = edges + (eidx + len(co)).tolist() 
    x_start_eidx = np.copy(eidx)
    x_eidx = eidx + len(co)
    x_vidx = x_eidx.ravel()
    
    # make extra edges
    lateral_edges = []
    for i, e in enumerate(edges):
        l1 = [e[0], x_eidx[i][0]]
        l2 = [e[0], x_eidx[i][1]]
        l3 = [e[1], x_eidx[i][0]]
        l4 = [e[1], x_eidx[i][1]]
        lateral_edges += [l1, l2, l3, l4]
    
    with_lats = edges_with_x + lateral_edges
    mix_mesh = U.link_mesh(co_with_x, edges=with_lats, faces=[], name=name, check_ob=False)# name=ph.physics_rig.name + "_mix_mesh")
    mix_mesh.matrix_world = ph.physics_rig.matrix_world

    return mix_mesh, edges, x_eidx, x_start_eidx, mix_vidx, x_vidx, y_vidx
    

def build_xz_cross_thingy(ph, factor=0.5):

    mesh_bone_co = ph.current_co[ph.mesh_bone_idx]
    x_vecs = ph.physics_m3[:, :, 0]
    x_edges = np.empty((x_vecs.shape[0], 2, 3), dtype=np.float32)
    x_edges[:, 1] = mesh_bone_co + (x_vecs * factor)
    x_edges[:, 0] = mesh_bone_co - (x_vecs * (1 - factor))
    eidx = np.arange(x_edges.shape[0] * 2, dtype=np.int32)
    eidx.shape = (x_edges.shape[0], 2)
    x_edges.shape = (x_edges.shape[0] * 2, 3)    
    x_mesh = U.link_mesh(x_edges, eidx, faces=[], name=ph.physics_rig.name + "_x_edges")
    x_mesh.matrix_world = ph.physics_rig.matrix_world
    
    return x_mesh, eidx, x_edges


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
        ee.rotation_quaternion = self.t_quats[0]
        
