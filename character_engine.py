import bpy
import bmesh
import numpy as np
import sys
import os
import importlib

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


class physics():
    def __init__(self, ob):
        self.ob = ob
        U.manage_shapes(ob, shapes=["Basis", "Target", "Current"])
        ob.data.shape_keys.key_blocks["Current"].value = 1.0
        #ob.data.update()
        
        # indexing
        self.obm = U.get_bmesh(ob)
        self.pinned = np.array([v.select for v in self.obm.verts], dtype=bool)
        self.stretch_idx, self.stretch_repeat, self.stretch_counts = get_stretch_springs(self)
        
        # targeting
        self.stretch_dist = np.empty(self.stretch_repeat.shape[0], dtype=np.float32)
        self.stretch_dif = np.empty((self.stretch_repeat.shape[0], 3), dtype=np.float32)
        self.stretch_mean = np.empty((len(ob.data.vertices), 3), dtype=np.float32)
        
        # coordinates
        self.target_co = np.empty((len(ob.data.vertices), 3), dtype=np.float32)
        C.get_shape_co_mode(ob=self.ob, co=self.target_co, key='Target')
        self.current_co = np.empty((len(ob.data.vertices), 3), dtype=np.float32)
        C.get_shape_co_mode(ob=self.ob, co=self.current_co, key='Current')
        self.crawl = np.copy(self.current_co)
        self.crawl_output = np.empty((len(ob.data.vertices), 3), dtype=np.float32)
        
        # linear
        self.stretch_co = self.current_co[self.stretch_idx]
        self.repeat_co = self.current_co[self.stretch_repeat]
        dif = np.subtract(self.target_co[self.stretch_idx], self.target_co[self.stretch_repeat], out=self.stretch_dif)
        self.target_dist = np.sqrt(np.einsum('ij,ij->i', dif, dif, out=self.stretch_dist))[:, None]
        
        # velocity
        self.velocity = np.zeros((len(ob.data.vertices), 3), dtype=np.float32)
        self.vel_start = np.empty((len(ob.data.vertices), 3), dtype=np.float32)
        self.vel_start[:] = self.current_co
        
        # quats
        self.eidx = get_edge_bone_idx(ar_mesh)
        self.edge_match_quats = np.empty((self.eidx.shape[0], 4), dtype=np.float32)
        self.edge_dif_quats = np.empty((self.eidx.shape[0], 4), dtype=np.float32)
        # e1 is the start state of the mesh before forces
        # e2 comes after stretch and velocity
        #get_edge_match_quats(e1, e2, out=None)
                
        
    def refresh(self):
        C.get_shape_co_mode(ob=self.ob, co=self.current_co, key='Current')
        C.get_shape_co_mode(ob=self.ob, co=self.vel_start, key='Current')
        #C.get_shape_co_mode(ob=self.ob, co=self.crawl, key='Current')
        #U.get_shape_co(self.ob, "Current", self.current_co)
        #U.get_shape_co(self.ob, "Current", self.crawl)
        #self.vel_start[:] = self.current_co


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
    
    if ph.ob.data.is_editmode:
        ph.current_co[~ph.pinned] = ph.stretch_mean[~ph.pinned]
    else:    
        ph.current_co = ph.stretch_mean


def angular_solve(ph):
    
    # get linear angular effect----------
    #e1 = ph.crawl[ph.eidx]
    e1 = A.get_bone_eco(ph.phar)
    e2 = ph.current_co[ph.eidx]



    Q.get_edge_match_quats(e1, e2, out=ph.edge_match_quats)
    ar_quats = A.get_ar_quats_world(ph.phar, quats=None)
    
    Q.quaternions_subtract(ph.edge_match_quats, ar_quats, out=ph.edge_dif_quats)
    
    #m3 = A.get_ar_m3(ph.phar)
    #eu = Q.quat_to_euler(ph.edge_dif_quats[0], factor=1, m3=m3[0])
    #print(m3[0])
    #print(np.round(eu, 3))
    #A.set_ar_m3_world(ph.phar, m3=eu, locations=None)
    #return ph.edge_dif_quats


import time
T = time.time()



def live_update(scene=None):
    T = time.time()
    
    # update the bmesh ----------
    if ph.ob.data.is_editmode:    
        try:
            ph.obm.verts[0]
        except (ReferenceError, IndexError):
            ph.obm = U.get_bmesh(ph.ob, refresh=True)
        
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

    for i in range(1):
        angular_solve(ph)
            
    gravity = -0.1
    gravity = 0.0
    ph.velocity[:,2] += gravity
    vel = ph.current_co - ph.vel_start
    ph.velocity += vel
        
    #if False:    
    if True:    
        ph.current_co[~ph.pinned] += ph.velocity[~ph.pinned]
        #if ph.ob.data.is_editmode:
            #ph.current_co[~ph.pinned] += ph.velocity[~ph.pinned]
        #else:    
            #ph.current_co += ph.velocity
        ph.velocity *= 0.95
        ph.velocity[ph.pinned] = 0.0
        
    if ph.ob.data.is_editmode:
        for i, v in enumerate(ph.obm.verts):
            if not v.select:    
                v.co = ph.current_co[i] 
                #pass
                #ph.current_co[i] = v.co 
            #else:
            #ph.vel_start[i] = v.co
        bmesh.update_edit_mesh(ph.ob.data)
        #ph.refresh()

    else:
        U.set_shape_co(ph.ob, "Current", ph.current_co)
        ph.ob.data.update()
    
    ph.vel_start[:] = ph.current_co
    #ph.refresh()
    
    mesh_bone_co = ph.current_co[ph.mesh_bone_idx]
    A.set_ar_quats_world(ph.phar, quats=ph.edge_dif_quats, locations=mesh_bone_co)
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
        



def get_edge_bone_idx(ob):
    # for now:
    return np.array([[0,1],[1,2],[2,3],[2,4]], dtype=np.int32)


def get_mesh_bone_idx(ob):
    return np.array([0, 1, 2, 2], dtype=np.int32)


def get_edge_centers(co, eidx, scale=0.5):
    """Start and end of edge
    has to match head and tail
    of bone if center of mass
    is not at 0.5."""
    head = co[eidx[:, 0]]
    vec = (co[eidx[:,1]] - head) * scale
    return head + vec



    



ob1 = bpy.data.objects["c1"]
ob2 = bpy.data.objects["c2"]
ar_mesh = bpy.data.objects['ar_mesh']
phar = bpy.data.objects['phar']
tar = bpy.data.objects['tar']
head = A.get_bone_head(tar)
tail = A.get_bone_tail(tar)

ar_mesh.data.vertices[0].co = head[0]
ar_mesh.data.vertices[1].co = head[1]
ar_mesh.data.vertices[2].co = tail[1]
ar_mesh.data.vertices[3].co = tail[2]
ar_mesh.data.vertices[4].co = tail[3]
ar_mesh.data.update()
C.co_to_shape(ar_mesh, co=None, key="Current")
C.co_to_shape(ar_mesh, co=None, key="Target")
C.co_to_shape(ar_mesh, co=None, key="Basis")


ph = physics(ar_mesh)

#ph.refresh()
ph.eidx = get_edge_bone_idx(ar_mesh)
ph.mesh_bone_idx = get_mesh_bone_idx(ar_mesh)
ph.phar = phar




install_handler(live=False, animated=True)
    


#if False:
#    co = U.get_co(ar_mesh)
#    ebidx = get_edge_bone_idx(ar_mesh)
#    mesh_bone_idx = get_mesh_bone_idx(ar_mesh)
#    bone_centers = A.get_bone_centers(phar)
#    edge_centers = get_edge_centers(co, ebidx, scale=0.5)

#    mesh_bone_co = co[mesh_bone_idx]

#    print(np.round(edge_centers,3))
#    print(np.round(bone_centers,3))
#    print(np.round(bone_centers,5) - np.round(edge_centers,5))

#    print(A.get_bone_location_world(phar))
#    print(A.get_bone_head(phar))

#    mesh_bone_co = co[mesh_bone_idx] - A.get_bone_head(phar)
#    A.se.t_ar_quats_world(phar, quats=None, locations=co[mesh_bone_idx])
#    #Q.set_bone_locations(phar, mesh_bone_co)
