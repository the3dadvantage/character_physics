import bpy
import bmesh
import numpy as np
import sys
import os
import importlib
import time

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

r_print = U.r_print

print()
print("=== reload ===")
print()


def get_stretch_springs_mix(ph):
    
    face = False
    if face:    
        fv = [list(set(U.flatten_list([[v.index
            for v in f.verts if v.index != ve.index]
                for f in ve.link_faces]))) for ve in ph.mix_obm.verts]
    else:
        fv = [[e.other_vert(ve).index
            for e in ve.link_edges]
                for ve in ph.mix_obm.verts]
    
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


def linear_solve(ph):
    vecs = ph.current_co[ph.mix_eidx[:, 1]] - ph.current_co[ph.mix_eidx[:, 0]]
    current_lengths = np.sqrt(np.einsum('ij,ij->i', vecs, vecs))
    div = ph.target_edge_lengths / current_lengths
    plot = vecs * div[:, None]
    move = (vecs - plot)

    np.add.at(ph.current_co, ph.bone_and_x_eidx[:, 0], move[:ph.bone_count * 2] * ph.left_move_force)
    np.subtract.at(ph.current_co, ph.mix_eidx[:, 1], move * ph.right_move_force)
        

def vecs_to_matrix(ph):
    mesh_edge_co = ph.current_co[ph.eidx]
    mesh_vecs = mesh_edge_co[:, 1] - mesh_edge_co[:, 0]
    mesh_u_vecs = C.u_vecs(mesh_vecs)    
    u_x_vecs = U.replot_orthagonal_mix(ph)
    z_vecs = np.cross(u_x_vecs, mesh_u_vecs)    
    ph.physics_m3[:, :, 0] = u_x_vecs
    ph.physics_m3[:, :, 1] = mesh_u_vecs
    ph.physics_m3[:, :, 2] = z_vecs
    return mesh_edge_co, u_x_vecs, mesh_u_vecs, z_vecs 
    

def both_angles_mix(ph):

    # update matrix------
    mesh_edge_co, ux, uy, uz = vecs_to_matrix(ph)
    # -------------------
    
    rot_to_plot = np.linalg.inv(ph.target_m3)[ph.repeater] @ ph.target_m3[ph.relative]
    plot_relative = ph.physics_m3[ph.repeater] @ rot_to_plot
    
    plot_y_vecs = plot_relative[:, :, 1]
    relative_rot_edges = C.vec_to_vec_pivot(mesh_edge_co[ph.relative], plot_y_vecs, factor=0.5)
    relative_rot_edges.shape = (relative_rot_edges.shape[0] * 2, 3)
    
    # ------- new solver
    move = (relative_rot_edges - ph.current_co[ph.relative_eidx.ravel()])
    np.add.at(ph.current_co, ph.relative_eidx.ravel(), move * ph.rel_rot_multiplier)
    # ------- new solver
        
    # now x
    plot_x_vecs = plot_relative[:, :, 0]    
    x_edge_co = ph.current_co[ph.mx_eidx][ph.relative]
    
    relative_rot_edges = C.vec_to_vec_pivot(x_edge_co, plot_x_vecs, factor=0.5)
    relative_rot_edges.shape = (relative_rot_edges.shape[0] * 2, 3)
    x_edge_co.shape = (x_edge_co.shape[0] * 2, 3)
        
    # ------- new solver
    x_move = relative_rot_edges - x_edge_co
    np.add.at(ph.current_co, ph.mx_eidx_relative.ravel(), x_move * ph.x_rot_multiplier)
    # ------- new solver
    

def edit_mode_update(ph):
    """Detect selected and update
    current_co and vel_start to
    selected location."""
    try:
        ph.mix_obm.verts[0]
    except (ReferenceError, IndexError):
        ph.mix_obm = U.get_bmesh(ph.mix_mesh, refresh=True)
    
    for i, v, in enumerate(ph.mix_obm.verts):
        ph.pinned[i] = v.select
        if ph.pinned[i]:
            ph.vel_start[i] = v.co
            ph.current_co[i] = v.co
    

def update_pinned(ph):
    ph.current_co[ph.pinned] = ph.vel_start[ph.pinned]
    influence_targets(ph)


def collision(ph):
    ### ----- COLLISION ----- ###
    yz = ph.current_co[:, 2][:ph.yvc]
    ph.hit = yz <= 0.0
    
    ph.vel_start[:ph.yvc][:, 2][ph.hit] = 0.0
    ph.current_co[:ph.yvc][ph.hit] = ph.vel_start[:ph.yvc][ph.hit]
    ph.velocity[:ph.yvc][ph.hit] = 0.0
    ### ----- --------- ----- ###


def collision_refresh(ph):
    ph.current_co[:ph.yvc][ph.hit] = ph.vel_start[:ph.yvc][ph.hit]


def influence_check(ph):
    """Check for needed updates outside of
    iterations.
    !!! can run this on prop callbacks when
    creating heads, tail, or adjusting
    values on heads or tails !!!"""
    
    bpy.context.view_layer.update()
    
    for b in ph.physics_rig.pose.bones:
        if b.Ce_props.target_object:
            if not (b.Ce_props.target_object.name in bpy.context.scene.objects):
                b.Ce_props.target_object = None
        if b.Ce_props.tail_target_object:
            if not (b.Ce_props.tail_target_object.name in bpy.context.scene.objects):
                b.Ce_props.tail_target_object = None
        
    targets = [b.Ce_props.target_object for b in ph.physics_rig.pose.bones if b.Ce_props.target_object]
    tail_targets = [b.Ce_props.tail_target_object for b in ph.physics_rig.pose.bones if b.Ce_props.tail_target_object]
    
    both = targets + tail_targets
    ph.head_tail_targets = both
    ph.inf_targets = True
    if len(both) == 0:
        ph.inf_targets = None
        return

    ph.inf_spring_influence = np.array([b.Ce_props.influence_spring for b in both], dtype=np.float32)[:, None]
    ph.inf_linear_influence = np.array([b.Ce_props.influence_directional for b in both], dtype=np.float32)[:, None]
    
    ph.any_spring = np.any(ph.inf_spring_influence != 0.0)
    ph.any_linear = np.any(ph.inf_linear_influence != 0.0)
    if (not ph.any_spring) & (not ph.any_linear):
        ph.inf_targets = None
        return

    ph.target_matricies = np.array([b.matrix_world for b in both], dtype=np.float32)
    ph.head_tail_eidx = np.array([[b.Ce_props.influence_head_idx, b.Ce_props.influence_tail_idx] for b in both])
    
    tails = [t.Ce_props.influence_is_tail for t in both]
    heads = ph.head_tail_eidx[:, 0]
    heads[tails] = ph.head_tail_eidx[tails][:, 1]
    ph.head_tail_idx = heads


def influence_targets(ph):

    if not ph.inf_targets:
        return
    
    both = ph.head_tail_targets
        
    if len(both) == 0:
        return
    
    spring_influence = ph.inf_spring_influence
    linear_influence = ph.inf_linear_influence
    
    any_spring = ph.any_spring
    any_linear = ph.any_linear

    BM = ph.target_matricies

    head_tail = ph.head_tail_eidx
    world_co = U.apply_transforms(ph.mix_mesh, ph.current_co)
    head_co = world_co[head_tail[:, 0]]
    tail_co = world_co[head_tail[:, 1]]

    heads = ph.head_tail_idx
    
    if any_linear:
        y = BM[:, :3, 1]
        lin_move = y * linear_influence    
        np.add.at(ph.current_co, heads, lin_move)

    if any_spring:        
        both_co = BM[:, :3, 3]
        move = both_co - world_co[ph.head_tail_idx]
        move *= spring_influence        
        np.add.at(ph.current_co, ph.head_tail_idx, move)        


def live_update(scene=None):

    ph = DATA['ph']
    Ce = ph.target_rig.Ce_props
    # update mix bmesh -----------
    if ph.mix_mesh.data.is_editmode:
        edit_mode_update(ph)
    else:    
        ph.pinned[:] = False

    ph.target_m3 = A.get_ar_matrix_world(ph.target_rig, m3=True)
    angular_force = ph.target_rig.Ce_props.angular_force
    influence_check(ph) # see if there are any targets
        
    for i in range(Ce.sub_frames):

        ### -----GRAVITY----- ###
        gravity_prop = ph.target_rig.Ce_props.gravity * 0.01
        ph.velocity[:,2] += gravity_prop
        
        ### ----- MIX VELOCITY ----- ###
        velocity_prop = ph.target_rig.Ce_props.velocity
        vel_mix = ph.current_co - ph.vel_start
        ph.velocity += vel_mix
        ph.velocity[ph.pinned] = 0.0
        ph.velocity *= velocity_prop
        
        ph.current_co += ph.velocity
        update_pinned(ph)

        ph.vel_start[:] = ph.current_co
        collision(ph)
            
        ### -----LINEAR----- ###
        for i in range(Ce.linear_iters):
            linear_solve(ph)
            update_pinned(ph)

        ### -----ANGULAR----- ###
        int_part, decimal = U.split_float(angular_force)
            
        collision_refresh(ph)
        for i in range(Ce.angular_iters):
            collision_refresh(ph)
            both_angles_mix(ph)
            update_pinned(ph)
            
            for i in range(Ce.linear_post_iters):
                linear_solve(ph)
                update_pinned(ph)

        collision(ph)
        
    ### ----- UPDATE MESH ----- ###
    if ph.mix_mesh.data.is_editmode:
        for i, v in enumerate(ph.mix_obm.verts):
            if not v.select:    
                v.co = ph.current_co[i]
                    
        bmesh.update_edit_mesh(ph.mix_mesh.data)

    else: 
        C.set_shape_co(ph.mix_mesh, "Current", ph.current_co)
        ph.mix_mesh.data.update()
        
    ### ----- UPDATE RIG ----- ###
    mesh_bone_co = ph.current_co[ph.mesh_bone_idx]
    A.set_ar_m3_world(ph.physics_rig, ph.physics_m3, locations=mesh_bone_co, return_quats=False)   
            
    if ph.mesh_only:
        ph.vel_start[:] = ph.current_co
        return 0.0    
    
    #print(time.time() - T)
    delay = 0.0
    return delay


### ================= ###
#                       #
#   ===== SETUP =====   #
#                       #
### ================= ###
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

 
def distribute_mass(ph):
    # come back here
    bone_count = len(ph.target_rig.pose.bones)

    # quick edge ------- 
    ph.mix_eidx = np.array(ph.mix_mesh.data.edge_keys, dtype=np.int32)
    vecs = ph.target_co_mix[ph.mix_eidx[:, 1]] - ph.target_co_mix[ph.mix_eidx[:, 0]]
    ph.target_edge_lengths = np.sqrt(np.einsum('ij,ij->i', vecs, vecs))
    # quick edge -------

    bone_edges = ph.mix_eidx[:bone_count]
    x_edgex = ph.mix_eidx[bone_count: bone_count * 2]
    lat_edges = ph.mix_eidx[bone_count * 2:]

    ph.bone_edges = bone_edges
    ph.bone_count = bone_count

    mass_balance = np.zeros(ph.mix_eidx.shape[0])[:, None]
    mass_balance[bone_count:] = 0.25
    
    ph.bone_and_x_eidx = ph.mix_eidx[:bone_count * 2]

    head_mass = np.array([b.Ce_props.head_mass for b in ph.target_rig.pose.bones], dtype=np.float32)
    tail_mass = np.array([b.Ce_props.tail_mass for b in ph.target_rig.pose.bones], dtype=np.float32)
    bone_mass = (head_mass + tail_mass) * 0.5
    
    testing = False
    if testing:
        head_mass[:] = 1.0
        tail_mass[:] = 1.0
    
        
    node_mass = np.zeros(ph.current_co.shape[0])
    np.add.at(node_mass, ph.mesh_bone_idx, head_mass)
    np.add.at(node_mass, ph.mesh_bone_tail_idx, tail_mass)
    node_mass[ph.mx_eidx.ravel()] = np.repeat(bone_mass, 2)

    mb_div = 1 / np.sum(node_mass[bone_edges], axis=1)[:, None]
    l_r_mass = node_mass[bone_edges] * mb_div

    l_uni, l_inv, l_counts = np.unique(bone_edges[:, 0], return_inverse=True, return_counts=True)
    r_uni, r_inv, r_counts = np.unique(bone_edges[:, 1], return_inverse=True, return_counts=True)
    l_mult = 1 / l_counts[l_inv]
    r_mult = 1 / r_counts[r_inv]

    lr_uni, lr_inv, lr_counts = np.unique(bone_edges.ravel(), return_inverse=True, return_counts=True)
    lr_mult = 1 / lr_counts[lr_inv]

    swap = np.roll(l_r_mass, 1, 1)
    l_r_mass = swap.ravel() * lr_mult
    l_r_mass.shape = (l_r_mass.shape[0] // 2, 2)

    left_move_force = np.copy(mass_balance.ravel())
    left_move_force[:bone_count] = l_r_mass[:, 0]
    
    right_move_force = np.copy(mass_balance.ravel())
    right_move_force[:bone_count] = l_r_mass[:, 1]

    ph.left_move_force = left_move_force[:bone_count * 2][:, None]
    ph.right_move_force = right_move_force[:, None]
    
    rel_eidx = ph.eidx[ph.relative]
    rep_eidx = ph.eidx[ph.repeater]
    
    shared = []
    rel_rot_idxer = []
    offset = 0
    for i in range(rel_eidx.shape[0]):
        
        if rep_eidx[i][0] in rel_eidx[i]:
            shared += [rep_eidx[i][0]]
            rel_rot_idxer += [0 + offset]
        else:
            shared += [rep_eidx[i][1]]            
            rel_rot_idxer += [1 + offset]

        offset += 2

    # mass all set to one
    bones_1 = np.ones_like(bone_mass)
    
    relative_bone_1 = bones_1[ph.relative]
    acc_bone_1 = np.zeros(ph.y_vidx.shape[0], dtype=np.float32)
    np.add.at(acc_bone_1, ph.eidx[ph.relative].ravel(), np.repeat(relative_bone_1, 2))
    ph.rel_rot_multiplier = (1 / acc_bone_1)[ph.eidx][ph.relative].ravel()[:, None]

    # x attempt
    offset_x = ph.mx_eidx - ph.mx_eidx[0]
    acc_bone_x = np.zeros(ph.x_vidx.shape[0], dtype=np.float32)
    relative_bone_x = bones_1[ph.relative]
    np.add.at(acc_bone_x, offset_x[ph.relative].ravel(), np.repeat(relative_bone_1, 2))
    ph.x_rot_multiplier = (1 / acc_bone_x)[offset_x][ph.relative].ravel()[:, None]
    
    # compare_mass
    # left side of relative should compare to right side of repeat
    relative_mass = bone_mass[ph.relative]
    repeat_mass = bone_mass[ph.repeater]
    compare_sum = relative_mass + repeat_mass
    print(compare_sum)
    relative_balanced = relative_mass / compare_sum
    repeat_balanced = (repeat_mass / compare_sum) * 2

    ph.repeat_eidx = ph.eidx[ph.repeater]
    ph.relative_eidx = ph.eidx[ph.relative]    

    ph.rel_rot_multiplier = ph.rel_rot_multiplier * np.repeat(repeat_balanced, 2)[:, None]
    ph.x_rot_multiplier = ph.x_rot_multiplier * np.repeat(repeat_balanced, 2)[:, None]
    
    # steve


class physics():
    def __init__(self, physics_rig, target_rig, mesh_only=False):
        DATA['ph'] = self
        
        self.mesh_only = mesh_only
        physics_mesh = mesh_only
        self.physics_mesh = physics_mesh
        
        self.target_rig = target_rig
        self.physics_rig = physics_rig
        self.target_m3 = A.get_ar_matrix_world(self.target_rig, m3=True)
        self.physics_m3 = A.get_ar_matrix_world(self.physics_rig, m3=True)
        
        # mass
        self.head_mass = np.array([b.Ce_props.head_mass for b in self.target_rig.pose.bones], dtype=np.float32)
        self.tail_mass = np.array([b.Ce_props.tail_mass for b in self.target_rig.pose.bones], dtype=np.float32)

        # friction
        self.head_friction = np.array([b.Ce_props.head_friction for b in self.target_rig.pose.bones], dtype=np.float32)
        self.tail_friction = np.array([b.Ce_props.tail_friction for b in self.target_rig.pose.bones], dtype=np.float32)
        
        if not mesh_only:            
            relative, repeater = A.get_relative_bones(target_rig)
            self.relative = np.array(relative, dtype=np.int32)
            self.repeater = np.array(repeater, dtype=np.int32)            
            self.physics_rig = physics_rig
            self.target_rig = target_rig
            bc = len(physics_rig.pose.bones)
            self.bc = bc
        
        # mix    
        mix_mesh, edges, mx_eidx, x_start_eidx, mix_vidx, x_vidx, y_vidx = A.make_mix_mesh(target_rig, self)
        self.edges = edges
        self.mix_mesh = mix_mesh
        self.mx_eidx = mx_eidx
        self.mx_eidx_relative = self.mx_eidx[self.relative]
        self.x_start_eidx = x_start_eidx
        self.x_vidx = x_vidx
        self.y_vidx = y_vidx
        self.eidx = np.array(edges, dtype=np.int32)
        self.mix_vidx = mix_vidx
        self.yvc = len(mix_vidx)
        
        self.v_count = len(mix_mesh.data.vertices)
        self.x_bool = np.zeros(self.v_count, dtype=bool)
        self.x_bool[self.x_vidx] = True
        self.y_bool = np.zeros(self.v_count, dtype=bool)
        self.y_bool[self.y_vidx] = True
        self.yvc = self.y_vidx.shape[0]        
        self.mesh_bone_idx = self.eidx[:, 0]
        self.mesh_bone_tail_idx = self.eidx[:, 1]
        
        vc_mix = len(mix_mesh.data.vertices)
        self.vc_mix = vc_mix
        self.vidx_mix = np.arange(vc_mix)
        U.manage_shapes(mix_mesh, shapes=["Basis", "Target", "Current"])
        mix_mesh.data.shape_keys.key_blocks["Current"].value = 1.0
        mix_mesh.active_shape_key_index = 2
        
        # mix coordinates:
        self.target_co_mix = np.empty((vc_mix, 3), dtype=np.float32)
        C.get_shape_co_mode(ob=mix_mesh, co=self.target_co_mix, key='Target')
        self.current_co = np.empty((vc_mix, 3), dtype=np.float32)
        C.get_shape_co_mode(ob=mix_mesh, co=self.current_co, key='Current')
        self.pre_collided = np.copy(self.current_co)
        
        self.mix_obm = U.get_bmesh(mix_mesh)
        self.pinned = np.array([v.select for v in self.mix_obm.verts], dtype=bool)
        self.collided = np.zeros(vc_mix, dtype=bool)
        self.stretch_idx_mix, self.stretch_repeat_mix, self.stretch_counts_mix = get_stretch_springs_mix(self)
        
        # mix
        self.stretch_dist_mix = np.empty(self.stretch_repeat_mix.shape[0], dtype=np.float32)
        self.u_dif_mix = np.empty((self.stretch_repeat_mix.shape[0], 3), dtype=np.float32)
        self.stretch_dif_mix = np.empty((self.stretch_repeat_mix.shape[0], 3), dtype=np.float32)
        self.stretch_mean_mix = np.empty((self.vc_mix, 3), dtype=np.float32)
        self.stretch_mean_y = np.empty((self.yvc, 3), dtype=np.float32)

        self.x_stretch_mean_mix = np.empty((bc * 2, 3), dtype=np.float32)

        # linear mix
        self.stretch_co_mix = self.current_co[self.stretch_idx_mix]
        self.repeat_co_mix = self.current_co[self.stretch_repeat_mix]
        dif_mix = np.subtract(self.target_co_mix[self.stretch_idx_mix], self.target_co_mix[self.stretch_repeat_mix], out=self.stretch_dif_mix)
        self.target_dist_mix = np.sqrt(np.einsum('ij,ij->i', dif_mix, dif_mix, out=self.stretch_dist_mix))[:, None]

        self.velocity = np.zeros((vc_mix, 3), dtype=np.float32)
        self.vel_start = np.empty((vc_mix, 3), dtype=np.float32)
        self.vel_start[:] = self.current_co

        if mesh_only:
            return
        
        # angular ------------
        self.start_physics_m3 = A.get_ar_matrix_world(self.physics_rig, m3=True)
        self.relative_m3 = self.target_m3[self.relative]
        self.repeater_m3 = self.target_m3[self.repeater]
        
        self.relative_eidx = self.eidx[self.relative]
        self.angle_mean_counts = 1 / np.array([np.count_nonzero(self.relative_eidx.ravel() == v) for v in self.y_vidx], dtype=np.float32)[:, None]
        
        self.start_x_eidx_relative = self.x_start_eidx[self.relative]
        self.start_x_eidx_repeater = self.x_start_eidx[self.repeater]
        
        uni, counts = np.unique(self.start_x_eidx_repeater, return_counts=True)
        self.x_angle_mean_counts = (1 / counts)[:, None]
        
        self.hit = np.zeros(self.current_co.shape[0], dtype=bool)
        
        # setup mass:
        distribute_mass(self)
        # ----------- come back here
        
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


