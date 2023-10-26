import bpy
import bmesh
import numpy as np
import sys
import os
import importlib
from mathutils import Matrix as MAT


DATA = {}


psep = os.path.sep
path = '/home/rich/Desktop/cloth/character_engine'
sys.path.append(path)

try:
    U = bpy.data.texts['utils.py'].as_module()
    Q = bpy.data.texts['quaternions.py'].as_module()
    C = bpy.data.texts['coordinates.py'].as_module()
    A = bpy.data.texts['armatures.py'].as_module()
    
except:
    import utils as U
    import quaternions as Q
    import coordinates as C
    import armatures as A
    importlib.reload(U)
    importlib.reload(Q)
    importlib.reload(C)
    importlib.reload(A)


print()
print("=== reload ===")
print()


def get_stretch_springs(ph):
    
    face = False
    if face:    
        fv = [list(set(U.flatten_list([[v.index
            for v in f.verts if v.index != ve.index]
                for f in ve.link_faces]))) for ve in ph.obm.verts]
    else:
        fv = [[e.other_vert(ve).index
            for e in ve.link_edges]
                for ve in ph.obm.verts]
    
    stretch_idx = []
    stretch_repeat = []
    stretch_counts = []
        
    for e, f in enumerate(fv):
        stretch_idx += f
        stretch_repeat += ([e] * len(f))
        stretch_counts += [len(f)]

    stretch_idx = np.array(stretch_idx, dtype=np.int32)
    stretch_repeat = np.array(stretch_repeat, dtype=np.int32)
    stretch_counts = np.array(stretch_counts, dtype=np.int32)[:, None]
                
    return stretch_idx, stretch_repeat, stretch_counts


def r_print(r, m=''):
    print()
    print(m + " --------")
    if isinstance(r, list):
        for v in r:
            print(np.round(v, 3), m)
        return        
    print(np.round(r, 3), m)


def linear_solve(ph):

    ph.current_co[ph.pinned] = ph.crawl[ph.pinned]
    
    ph.stretch_co[:] = ph.current_co[ph.stretch_idx]
    ph.repeat_co[:] = ph.current_co[ph.stretch_repeat]
    dif = np.subtract(ph.repeat_co, ph.stretch_co, out=ph.stretch_dif)
    dist = np.sqrt(np.einsum('ij,ij->i', dif, dif, out=ph.stretch_dist))
    
    u_dif = dif / dist[:, None]
    #u_dif = np.divide(dif finish this / dist[:, None]
    new_locs = ph.stretch_co + (u_dif * ph.target_dist)
    ph.stretch_mean[:] = 0.0
    np.add.at(ph.stretch_mean, ph.stretch_repeat, new_locs)
    ph.stretch_mean /= ph.stretch_counts
    
    if ph.phys_mesh.data.is_editmode:
        ph.current_co[~ph.pinned] = ph.stretch_mean[~ph.pinned]
    else:    
        ph.current_co = ph.stretch_mean


def angular_solve(ph):
    
    # get linear angular effect----------
    mesh_bone_co = ph.current_co[ph.mesh_bone_idx]     
    e1 = A.get_bone_eco(ph.phar)
    e2 = ph.current_co[ph.eidx]
    Q.get_edge_match_quats(e1, e2, out=ph.edge_match_quats)
    
    #matrices = A.get_ar_matrix_world(ph.phar)
    m3 = A.get_ar_matrix_world(ph.phar, m3=True)
    #rot_m = Q.rotate_matrices(matrices, ph.edge_match_quats)
    rot_m = Q.rotate_m3s(m3, ph.edge_match_quats)
    current_quats = A.set_ar_m3_world(ph.phar, rot_m, locations=mesh_bone_co, return_quats=True)

    set_target = False
    if set_target:
        A.set_ar_m3_world(ph.tar, rot_m, locations=mesh_bone_co, return_quats=True)
    # ------------------------------------
    
    # testing ---------
    repeat = [4,3,1]
    relative = [5,6,7]
    
    target = bpy.data.objects['target']
    physics = bpy.data.objects['physics']
        
    target_m3 = A.get_ar_matrix_world(target, m3=True)
    physics_m3 = A.get_ar_matrix_world(physics, m3=True)
    
    rot_m = np.linalg.inv(target_m3[repeat]) @ target_m3[relative]
    physics_m3[relative] = physics_m3[repeat] @ rot_m

    r_print(physics_m3)
    A.set_ar_m3_world(physics, physics_m3)
    return
    rep_to_rel = Q.quaternions_subtract(target_q[repeat], target_q[relative], out=None)

    t_to_p = Q.quaternions_subtract(target_q[repeat], physics_q[repeat])
    
    m31 = Q.rotate_m3s(target_m3[repeat], t_to_p)
    
    physics_m3[relative] = m31
    A.set_ar_m3_world(physics, physics_m3)

    

import time
T = time.time()



def live_update(scene=None):
    T = time.time()
    
    # update the bmesh ----------
    if ph.phys_mesh.data.is_editmode:    
        try:
            ph.obm.verts[0]
        except (ReferenceError, IndexError):
            ph.obm = U.get_bmesh(ph.phys_mesh, refresh=True)
        
        # update grabbed points ----------
        for i, v, in enumerate(ph.obm.verts):
            ph.pinned[i] = v.select
            if ph.pinned[i]:
                ph.current_co[i] = v.co                 
    else:
        ph.pinned[:] = False
    
    # run stretch and move if not pinned
    ph.crawl[:] = ph.current_co
    
    for i in range(10):    
        linear_solve(ph)
    
    ob_gravity = ph.phys_mesh.Ce_props.gravity
    gravity = ob_gravity
    ob_velocity = ph.phys_mesh.Ce_props.velocity
    #gravity = -0.1
    #gravity = 0.0
    ph.velocity[:,2] += gravity
    vel = ph.current_co - ph.vel_start
    ph.velocity += vel
        
    #if False:    
    if True:    
        ph.current_co[~ph.pinned] += ph.velocity[~ph.pinned]
        #if ph.phys_mesh.data.is_editmode:
            #ph.current_co[~ph.pin ned] += ph.velocity[~ph.pinned]
        #else:    
            #ph.current_co += ph.velocity
        ph.velocity *= ob_velocity
        ph.velocity[ph.pinned] = 0.0
        
    if ph.phys_mesh.data.is_editmode:
        for i, v in enumerate(ph.obm.verts):
            if not v.select:    
                v.co = ph.current_co[i] 
                #pass
                #ph.current_co[i] = v.co 
            #else:
            #ph.vel_start[i] = v.co
        bmesh.update_edit_mesh(ph.phys_mesh.data)
        #ph.refresh()

    else:
        C.set_shape_co(ph.phys_mesh, "Current", ph.current_co)
        ph.phys_mesh.data.update()
    
    #A.update_bones(ph.phar)
    
    for i in range(1):
        angular_solve(ph)
    
    ph.vel_start[:] = ph.current_co
    
    #ph.refresh()

    #A.update_bones(ph.phar)
    #A.set_ar_quats_world(ph.phar, quats=ph.edge_dif_quats, locations=mesh_bone_co)
    #A.set_ar_quats_world(ph.phar, quats=None, locations=mesh_bone_co)
    
    print(time.time() - T)
    delay = 0.0
    return delay


def install_handler(live=False, animated=False):
    """Adds a live or animated updater"""

    # clean dead versions of the animated handler
    handler_names = np.array([i.__name__ for i in bpy.app.handlers.frame_change_post])
    booly = [i == 'live_update' for i in handler_names]
    idx = np.arange(handler_names.shape[0])
    idx_to_kill = idx[booly]
    for i in idx_to_kill[::-1]:
        del(bpy.app.handlers.frame_change_post[i])

    # clean dead versions of the timer
    if bpy.app.timers.is_registered(live_update):
        bpy.app.timers.unregister(live_update)

    if live:    
        bpy.app.timers.register(live_update, persistent=True)
    
    if animated:            
        bpy.app.handlers.frame_change_post.append(live_update)


class physics():
    def __init__(self, phar, tar):
        DATA['ph'] = self
        phys_mesh, edges, co, index, relative, repeater = A.make_ar_mesh(tar)
        phys_mesh['ce_rig'] = phar.id_data
        phys_mesh.show_in_front = True
        phar['ce_phys_mesh'] = phys_mesh.id_data
        self.eidx = np.array(edges, dtype=np.int32)
        self.phys_mesh = phys_mesh
        self.relative_bones = relative
        self.relative_repeater = repeater
        self.mesh_bone_idx = self.eidx[:, 0]
        self.phar = phar
        self.tar = tar
        vc = len(phys_mesh.data.vertices)
        
        U.manage_shapes(phys_mesh, shapes=["Basis", "Target", "Current"])
        phys_mesh.data.shape_keys.key_blocks["Current"].value = 1.0
        phys_mesh.active_shape_key_index = 2
        #ob.data.update()
        
        # indexing
        self.obm = U.get_bmesh(phys_mesh)
        self.pinned = np.array([v.select for v in self.obm.verts], dtype=bool)
        self.stretch_idx, self.stretch_repeat, self.stretch_counts = get_stretch_springs(self)
        
        # targeting
        self.stretch_dist = np.empty(self.stretch_repeat.shape[0], dtype=np.float32)
        self.stretch_dif = np.empty((self.stretch_repeat.shape[0], 3), dtype=np.float32)
        self.stretch_mean = np.empty((vc, 3), dtype=np.float32)
        
        # coordinates
        self.target_co = np.empty((vc, 3), dtype=np.float32)
        C.get_shape_co_mode(ob=phys_mesh, co=self.target_co, key='Target')
        self.current_co = np.empty((vc, 3), dtype=np.float32)
        C.get_shape_co_mode(ob=phys_mesh, co=self.current_co, key='Current')
        self.crawl = np.copy(self.current_co)
        self.crawl_output = np.empty((vc, 3), dtype=np.float32)
        
        # linear
        self.stretch_co = self.current_co[self.stretch_idx]
        self.repeat_co = self.current_co[self.stretch_repeat]
        dif = np.subtract(self.target_co[self.stretch_idx], self.target_co[self.stretch_repeat], out=self.stretch_dif)
        self.target_dist = np.sqrt(np.einsum('ij,ij->i', dif, dif, out=self.stretch_dist))[:, None]
        
        # velocity
        self.velocity = np.zeros((vc, 3), dtype=np.float32)
        self.vel_start = np.empty((vc, 3), dtype=np.float32)
        self.vel_start[:] = self.current_co
        
        self.edge_match_quats = np.empty((self.eidx.shape[0], 4), dtype=np.float32)
        self.edge_dif_quats = np.empty((self.eidx.shape[0], 4), dtype=np.float32)
        # e1 is the start state of the mesh before forces
        # e2 comes after stretch and velocity
        
        # testing:
        self.start_tm3 = A.get_ar_matrix_world(self.tar, m3=True)[self.relative_repeater]
                
        
    def refresh(self):
        """No one is using this at the moment"""
        C.get_shape_co_mode(ob=self.phys_mesh, co=self.current_co, key='Current')
        C.get_shape_co_mode(ob=self.phys_mesh, co=self.vel_start, key='Current')
        #C.get_shape_co_mode(ob=self.ob, co=self.crawl, key='Current')
        #C.get_shape_co(self.ob, "Current", self.current_co)
        #C.get_shape_co(self.ob, "Current", self.crawl)
        #self.vel_start[:] = self.current_co

    
#phar = bpy.data.objects['phar']
#phar = bpy.data.objects['dup']

if False:    
    
    phar = bpy.data.objects['tar']
    tar = bpy.data.objects['tar']
    ph = physics(phar, tar)
    
    if False:
        C.co_to_shape(phys_mesh, co=None, key="Current")
        C.co_to_shape(phys_mesh, co=None, key="Target")
        C.co_to_shape(phys_mesh, co=None, key="Basis")


    install_handler(live=False, animated=True)
