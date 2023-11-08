import bpy
import bmesh
import numpy as np
import sys
import os
import importlib
from mathutils import Matrix as MAT
from scipy.linalg import logm, expm
from scipy.spatial.transform import Rotation as R

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
    stretch_counts = 1 / np.array(stretch_counts, dtype=np.int32)[:, None]
                
    return stretch_idx, stretch_repeat, stretch_counts


def r_print(r, m=''):
    print()
    print(m + " --------")
    if isinstance(r, list):
        for v in r:
            print(np.round(v, 3), m)
        return        
    print(np.round(r, 3), m)


def box_corner_normal_reset(corners, co):
    '''The theory is to make the corner
    of a box and check to make sure it's not
    inside out. Trouble is its center of
    mass would make for lopsided rotation.
    How to fix this... hmm...'''
    U.get_tri_normals(tco, normalize=True)
    

import time
T = time.time()


def collision_test(ph):
    ph.collided[:] = ph.current_co[:, 2] < 0.0
    ph.current_co[:, 2][ph.collided] = 0.0
    ph.vel_start[:, 2][ph.collided] = 0.0
    ph.velocity[ph.collided] = 0.0
    #ph.current_co[:, :2][ph.collided] = ph.vel_start[:, :2][ph.collided]
    ph.pinned[ph.collided] = True
    #ph.current_co[:, 2][ph.collided] = 0.0


def replot_orthagonal(ph):

    ### ===== X THINGY MID MEAN ===== ###
    #if True:
    if False:
        e_vecs = ph.x_co[1::2] - ph.x_co[::2]
        ph.x_mid = ph.x_co[::2] + (e_vecs * ph.x_factor)
        mix_mid = (ph.current_co[ph.mesh_bone_idx] + ph.x_mid) * 0.5
        ph.x_mid = mix_mid
        ph.current_co[ph.mesh_bone_idx] = mix_mid
    
    # update pinned:    
    #ph.current_co[ph.pinned] = ph.vel_start[ph.pinned]
    #ph.x_mid = ph.current_co[ph.mesh_bone_idx]

    ### ===== MAKE X ORTHAGONAL TO Y ===== ###
    eco = ph.current_co[ph.eidx]
    vecs = eco[:, 1] - eco[:, 0]
    
    cp, d = C.closest_points_edges(vecs=vecs, origins=ph.x_mid, p=ph.x_co[::2])
    cp_vec = cp - ph.x_co[::2]
    ucp = C.u_vecs(cp_vec)

    ph.x_co[::2] = ph.x_mid - (ucp * ph.x_factor)
    ph.x_co[1::2] = ph.x_mid + (ucp * (1 - ph.x_factor))    
    

def linear_solve(ph):
    
    ph.stretch_co[:] = ph.current_co[ph.stretch_idx]
    ph.repeat_co[:] = ph.current_co[ph.stretch_repeat]
    dif = np.subtract(ph.repeat_co, ph.stretch_co, out=ph.stretch_dif)
    dist = np.sqrt(np.einsum('ij,ij->i', dif, dif, out=ph.stretch_dist))
    
    u_dif = dif / dist[:, None]
    #u_dif = np.divide(dif finish this / dist[:, None]
    new_locs = ph.stretch_co + (u_dif * ph.target_dist)
    ph.stretch_mean[:] = 0.0
    np.add.at(ph.stretch_mean, ph.stretch_repeat, new_locs)
    ph.stretch_mean *= ph.stretch_counts
    ph.current_co[:] = ph.stretch_mean

    ### ===== UPDATE PINNED ===== ###    
    ph.current_co[ph.pinned] = ph.vel_start[ph.pinned]    
    
    ### ===== X THINGY MID MEAN ===== ###
    if False:    
        e_vecs = ph.x_co[1::2] - ph.x_co[::2]
        ph.x_mid = ph.x_co[::2] + (e_vecs * ph.x_factor)
        mix_mid = (ph.current_co[ph.mesh_bone_idx] + ph.x_mid) * 0.5
        ph.x_mid = mix_mid
        ph.current_co[ph.mesh_bone_idx] = mix_mid
    
    else:    
        ph.x_mid = ph.current_co[ph.mesh_bone_idx]

    replot_orthagonal(ph)
    
    C.set_shape_co(ph.x_mesh, "Current", ph.x_co.ravel())
    ph.x_mesh.data.update()

    #return
    if False: # will do this in angular
        bone_x = ph.physics_m3[:, :, 0]
        ph.x_co[ph.x_eidx[:, 0]] = ph.x_mid - (bone_x * (1 - ph.x_factor))
        ph.x_co[ph.x_eidx[:, 1]] = ph.x_mid + (bone_x * ph.x_factor)

    #ph.current_co[ph.mesh_bone_idx] = ph.x_mid
    bpy.data.objects['b0'].location = ph.x_mid[0]
    bpy.data.objects['b1'].location = ph.x_mid[1]



def matrix_from_edges(ph):
    yeco = ph.current_co[ph.eidx]
    yvecs = yeco[:, 1] - yeco[:, 0]
    u_y = C.u_vecs(yvecs)
    
    # should be u and orthagonal already: 
    u_x = ph.x_co[1::2] - ph.x_co[::2]
    cross = np.cross(u_x, u_y)

    new_m3 = np.copy(ph.physics_m3)
    new_m3[:,:,0] = u_x
    new_m3[:,:,1] = u_y
    new_m3[:,:,2] = cross
    
    return new_m3


def one_angle_2(ph):

    ### === updeat physics m3 to edge === ###
    bone_x = ph.physics_m3[:, :, 0]
    mesh_edge_co = ph.x_co[ph.x_eidx]
    mesh_vecs = mesh_edge_co[:, 1] - mesh_edge_co[:, 0]
    mesh_u_vecs = C.u_vecs(mesh_vecs)
    x_quats = Q.get_edge_match_quats(bone_x, mesh_u_vecs, out=None)
    Q.rotate_m3s(ph.physics_m3, x_quats)
    ### ===
    
    rot_to_plot = np.linalg.inv(ph.target_m3)[ph.repeater] @ ph.target_m3[ph.relative]
    plot_relative = ph.physics_m3[ph.repeater] @ rot_to_plot
    
    plot_x_vecs = plot_relative[:, :, 0]    
    relative_rot_edges = C.vec_to_vec_pivot(mesh_edge_co[ph.relative], plot_x_vecs, factor=0.5)
    # fix some shape issues...
    relative_rot_edges.shape = (relative_rot_edges.shape[0] * 2, 3)

    ph.x_stretch_mean[:] = 0.0
    np.add.at(ph.x_stretch_mean, ph.x_eidx[ph.relative].ravel(), relative_rot_edges)
    x_rot_locs = ph.x_stretch_mean * ph.x_angle_mean_counts        

    move = x_rot_locs - ph.x_co
    ph.x_co[:] += (move * 0.5)


def one_angle(ph):

    ### === updeat physics m3 to edge === ###
    bone_y = ph.physics_m3[:, :, 1]    
    mesh_edge_co = ph.current_co[ph.eidx]
    mesh_vecs = mesh_edge_co[:, 1] - mesh_edge_co[:, 0]
    mesh_u_vecs = C.u_vecs(mesh_vecs)
    y_quats = Q.get_edge_match_quats(bone_y, mesh_u_vecs, out=None)
    Q.rotate_m3s(ph.physics_m3, y_quats)
    ### ===
    
    rot_to_plot = np.linalg.inv(ph.target_m3)[ph.repeater] @ ph.target_m3[ph.relative]
    plot_relative = ph.physics_m3[ph.repeater] @ rot_to_plot
    
    plot_y_vecs = plot_relative[:, :, 1]    
    relative_rot_edges = C.vec_to_vec_pivot(mesh_edge_co[ph.relative], plot_y_vecs, factor=0.5)
    # fix some shape issues...
    relative_rot_edges.shape = (relative_rot_edges.shape[0] * 2, 3)

    ph.stretch_mean[:] = 0.0
    np.add.at(ph.stretch_mean, ph.relative_eidx.ravel(), relative_rot_edges)
    y_rot_locs = ph.stretch_mean * ph.angle_mean_counts        

    move = y_rot_locs - ph.current_co
    ph.current_co[:] += (move * 0.5)
    ph.current_co[ph.pinned] = ph.vel_start[ph.pinned]
    replot_orthagonal(ph)


def angular_solve(ph):
    one_angle(ph)
    one_angle_2(ph)
    return
    rot_to_plot = np.linalg.inv(ph.target_m3)[ph.repeater] @ ph.target_m3[ph.relative]
    plot_relative = ph.physics_m3[ph.repeater] @ rot_to_plot

    phys_x = ph.physics_m3[:, :, 0]
    phys_y = ph.physics_m3[:, :, 1]
    phys_z = ph.physics_m3[:, :, 2]
    rel_x = plot_relative[:, :, 0]
    rel_y = plot_relative[:, :, 1]
    rel_z = plot_relative[:, :, 2]
    print(rel_x.shape, "rel_x shape")
    print(phys_x[ph.repeater].shape, "phys_x shape")
    
    #Q.get_edge_match_quats(phys_x[ph.repeater], rel_x, out=ph.x_edge_match_quats, factor=0.25)
    Q.get_edge_match_quats(phys_x[ph.relative], rel_x, out=ph.x_edge_match_quats, factor=1.0)
    #Q.get_edge_match_quats(phys_y[ph.repeater], rel_y, out=ph.y_edge_match_quats, factor=0.25)
    Q.get_edge_match_quats(phys_y[ph.repeater], rel_y, out=ph.y_edge_match_quats, factor=1.0)
    Q.get_edge_match_quats(phys_z[ph.repeater], rel_z, out=ph.z_edge_match_quats, factor=0.25)
    mix1 = Q.add_quats(ph.x_edge_match_quats, ph.y_edge_match_quats, normalize=False, out=None)
    mix = Q.add_quats(mix1, ph.z_edge_match_quats, normalize=False, out=None)
    phys_cop = np.copy(ph.physics_m3)
    
    #Q.rotate_m3s(phys_cop, mix)
    Q.rotate_m3s(phys_cop, ph.y_edge_match_quats)
    #ph.physics_m3 = phys_cop
    plot_x = phys_cop[:, :, 0][ph.repeater]
    plot_y = phys_cop[:, :, 1][ph.repeater]
    
    mesh_edge_co = ph.current_co[ph.eidx]
    y_pivot = C.vec_to_vec_pivot(mesh_edge_co[ph.relative], plot_y, factor=0.5)
    y_pivot.shape = (y_pivot.shape[0] * 2, 3)

    ph.stretch_mean[:] = 0.0
    np.add.at(ph.stretch_mean, ph.eidx[ph.repeater].ravel(), y_pivot)
    ph.current_co[:] = ph.stretch_mean * ph.angle_mean_counts

    return
    x_edge_co = ph.x_co[ph.x_eidx]
    x_pivot = C.vec_to_vec_pivot(x_edge_co[ph.relative], plot_x, factor=0.5)
    x_pivot.shape = (x_pivot.shape[0] * 2, 3)
    ph.x_stretch_mean[:] = 0.0
    np.add.at(ph.x_stretch_mean, ph.x_eidx[ph.relative].ravel(), x_pivot)
    x_rot_locs = ph.x_stretch_mean * ph.x_angle_mean_counts
    ph.x_co = x_rot_locs    
    
    ### ===== RESET PINNED ===== ###
    ph.current_co[ph.pinned] = ph.vel_start[ph.pinned]
    replot_orthagonal(ph)
    
    new_m3 = matrix_from_edges(ph)
    #ph.physics_m3 = new_m3

    # visualize in test rig:
    rel = bpy.data.objects['rel']
    A.set_ar_m3_world(rel, phys_cop, locations=None, return_quats=False)
    
    
    return

    #if False:
    if True:
        if True:
            ph.current_co[:] = y_rot_locs
        else:
            move = y_rot_locs - ph.current_co
            ph.current_co += (move * 0.5)
        


    
    #edge_set = True
    edge_set = False
    if edge_set:    
        mesh_bone_co = ph.current_co[ph.mesh_bone_idx] # this is for plotting the locations of bones relative to coordinates in the physics mesh
        mesh_edge_co = ph.current_co[ph.eidx]
        mesh_vecs = mesh_edge_co[:, 1] - mesh_edge_co[:, 0]
        mesh_u_vecs = C.u_vecs(mesh_vecs)
        
        bone_y = physics_m3[:, :, 1]
        Q.get_edge_match_quats(bone_y, mesh_u_vecs, out=ph.edge_match_quats)
        physics_m3 = Q.rotate_m3s(physics_m3, ph.edge_match_quats)        
    
        A.set_ar_m3_world(ph.physics_rig, physics_m3, locations=mesh_bone_co, return_quats=False)
    


def live_update(scene=None):
    T = time.time()
    
    # update the bmesh ----------
    ph = DATA['ph']
    if ph.physics_mesh.data.is_editmode:    
        try:
            ph.obm.verts[0]
        except (ReferenceError, IndexError):
            ph.obm = U.get_bmesh(ph.physics_mesh, refresh=True)
        
        # update grabbed points ----------
        for i, v, in enumerate(ph.obm.verts):
            ph.pinned[i] = v.select
            if ph.pinned[i]:
                ph.current_co[i] = v.co                 
                ph.vel_start[i] = v.co
    else:
        ph.pinned[:] = False
    
    ph.pinned[ph.collided] = True
    
    ph.current_co[ph.pinned] = ph.vel_start[ph.pinned]
    ph.vel_start[:] = ph.current_co
    
    ### -----LINEAR----- ###
    for i in range(4):
        linear_solve(ph)
        collision_test(ph)
        
    ### -----ANGULAR----- ###
    ph.target_m3 = A.get_ar_matrix_world(ph.target_rig, m3=True)
    #ph.physics_m3 = A.get_ar_matrix_world(ph.physics_rig, m3=True)
    angular_force = ph.target_rig.Ce_props.angular_force
    int_part, decimal = U.split_float(angular_force)
    
    #for i in range(int_part):
    for i in range(int_part):
        angular_solve(ph)
        collision_test(ph)
    
    #return
    ### -----GRAVITY----- ###
    ob_gravity = ph.physics_mesh.Ce_props.gravity
    gravity = ob_gravity
    ob_velocity = ph.physics_mesh.Ce_props.velocity
    ph.velocity[:,2] += gravity
    
    ### -----VELOCITY----- ###
    vel = ph.current_co - ph.vel_start
    ph.velocity += vel
    ph.velocity[ph.pinned] = 0.0
    ph.current_co += ph.velocity
    ph.velocity *= ob_velocity

    ### ----- COLLISION ----- ###
    collision_test(ph)

    ph.current_co[ph.pinned] = ph.vel_start[ph.pinned]
    
    ### -----WRITE TO MESH----- ###
    if ph.physics_mesh.data.is_editmode:
        for i, v in enumerate(ph.obm.verts):
            if not v.select:    
                v.co = ph.current_co[i]
                    
        bmesh.update_edit_mesh(ph.physics_mesh.data)

    else:
        C.set_shape_co(ph.physics_mesh, "Current", ph.current_co)
        ph.physics_mesh.data.update()
    
    ### ----- UPDATE RIG ----- ###
    mesh_bone_co = ph.current_co[ph.mesh_bone_idx]
    A.set_ar_m3_world(ph.physics_rig, ph.physics_m3, locations=mesh_bone_co, return_quats=False)
    
    
    if ph.mesh_only:
        ph.vel_start[:] = ph.current_co
        return 0.0    
    
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


def get_regroup_index(repeater):
    """Groups the bones based on
    their number of relative bones:
    for gr in rel_rep_regroup:
        for g in gr:
            bone = repeater[g]
            relative = relative[g]
    """
    current_index = 0
    rel_rep_regroup = []
    current_group = []
    for e, rep in enumerate(repeater):
        if rep == current_index:
            current_group += [e]
        else:
            rel_rep_regroup += [current_group]
            current_group = [e]
            current_index = rep
        if e == (len(repeater) - 1):
            rel_rep_regroup += [current_group]
    return rel_rep_regroup


class physics():
    def __init__(self, physics_rig, target_rig, mesh_only=False):
        DATA['ph'] = self
        
        self.mesh_only = mesh_only
        physics_mesh = mesh_only
        self.physics_mesh = physics_mesh
        if not mesh_only:
            physics_mesh, edges, co, index, relative, repeater = A.make_ar_mesh(target_rig)
            self.rel_rep_regroup = get_regroup_index(repeater)
            
            physics_mesh.show_in_front = True
            self.eidx = np.array(edges, dtype=np.int32)
            self.physics_mesh = physics_mesh
            self.relative = np.array(relative, dtype=np.int32)
            self.repeater = np.array(repeater, dtype=np.int32)
            self.mesh_bone_idx = self.eidx[:, 0]
            self.physics_rig = physics_rig
            self.target_rig = target_rig
            bc = len(physics_rig.pose.bones)
            self.bc = bc
        
        vc = len(physics_mesh.data.vertices)
        self.vc = vc
        self.vidx = np.arange(vc)
        U.manage_shapes(physics_mesh, shapes=["Basis", "Target", "Current"])
        physics_mesh.data.shape_keys.key_blocks["Current"].value = 1.0
        physics_mesh.active_shape_key_index = 2
        
        # indexing
        self.obm = U.get_bmesh(physics_mesh)
        self.pinned = np.array([v.select for v in self.obm.verts], dtype=bool)
        self.collided = np.zeros(vc, dtype=bool)
        self.stretch_idx, self.stretch_repeat, self.stretch_counts = get_stretch_springs(self)
        
        # targeting
        self.stretch_dist = np.empty(self.stretch_repeat.shape[0], dtype=np.float32)
        self.stretch_dif = np.empty((self.stretch_repeat.shape[0], 3), dtype=np.float32)
        self.stretch_mean = np.empty((vc, 3), dtype=np.float32)
        self.x_stretch_mean = np.empty((bc * 2, 3), dtype=np.float32)
        
        # coordinates
        self.target_co = np.empty((vc, 3), dtype=np.float32)
        C.get_shape_co_mode(ob=physics_mesh, co=self.target_co, key='Target')
        self.current_co = np.empty((vc, 3), dtype=np.float32)
        C.get_shape_co_mode(ob=physics_mesh, co=self.current_co, key='Current')
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

        if mesh_only:
            return
        
        # angular
        self.target_m3 = A.get_ar_matrix_world(self.target_rig, m3=True)
        self.physics_m3 = A.get_ar_matrix_world(self.physics_rig, m3=True)
        self.start_physics_m3 = A.get_ar_matrix_world(self.physics_rig, m3=True)
        self.relative_m3 = self.target_m3[self.relative]
        self.repeater_m3 = self.target_m3[self.repeater]
        self.x_mean = np.zeros((self.target_m3.shape[0], 3), dtype=np.float32)
        
        self.relative_eidx = self.eidx[self.relative]
        self.angle_mean_counts = 1 / np.array([np.count_nonzero(self.relative_eidx.ravel() == v) for v in self.vidx], dtype=np.float32)[:, None]
        x_1 = 1 / np.array([np.count_nonzero(self.repeater == v) for v in np.arange(len(self.physics_rig.pose.bones))], dtype=np.float32)
        self.x_angle_mean_counts = np.repeat(x_1, 2)[:, None]
        
        quat_shape = self.eidx[self.repeater].shape[0]
        
        self.x_edge_match_quats = np.empty((quat_shape, 4), dtype=np.float32)
        self.y_edge_match_quats = np.empty((quat_shape, 4), dtype=np.float32)
        self.z_edge_match_quats = np.empty((quat_shape, 4), dtype=np.float32)

        
        
        
        ### ===== XZ CROSS THINGY ===== ###
        # in the future we can raycast on the x and z axis
        #   to get the length of these edges.
        #   Maybe for now we just use u_vecs.
        # The xz cross thingy should be placed along
        #   the center of mass on the y axis as well.
        #   It should exert a force on the y axis edge
        #       moving both ends of the y axis edge 
        #       based on the difference between the
        #       y axis center of mass and the xz center of mass.
        # For now let's just put stuff at 0.5
        
        self.x_factor = 0.5 # will eventually need to be an array
        self.x_mesh, self.x_eidx, self.x_co = A.build_xz_cross_thingy(self, factor=0.5)
        U.manage_shapes(self.x_mesh, shapes=["Basis", "Target", "Current"])
        self.x_mesh.data.shape_keys.key_blocks["Current"].value = 1.0
        self.x_mesh.active_shape_key_index = 2


        ### ===== TESTING ===== ###
        self.start_tm3 = A.get_ar_matrix_world(self.target_rig, m3=True)[self.relative]


print("!!! clearing handler here !!!")
install_handler(live=False, animated=False)
for ob in bpy.data.objects:
    if "physics_mesh" in ob:
        del(ob["physics_mesh"])
    if "target_rig" in ob:
        del(ob["target_rig"])
    if "physics_rig" in ob:
        del(ob["physics_rig"])

if False:
#if True:
    
    #physics_rig = bpy.data.objects['tar']
    #target_rig = bpy.data.objects['tar']
    physics_mesh = bpy.context.object
    ph = physics(physics_rig=None, target_rig=None, mesh_only=physics_mesh)
    
    if True:
        C.co_to_shape(physics_mesh, co=None, key="Current")
        C.co_to_shape(physics_mesh, co=None, key="Target")
        C.co_to_shape(physics_mesh, co=None, key="Basis")


    install_handler(live=False, animated=True)
