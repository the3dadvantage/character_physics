import bpy
import bmesh
import numpy as np
import sys
import os
import importlib
import time

DATA = {}

try:
    from character_physics import utils as U
    from character_physics import armatures as A
    from character_physics import node_collide as NC
    importlib.reload(U)
    importlib.reload(A)
    importlib.reload(NC)

except:
    U = bpy.data.texts['utils.py'].as_module()
    A = bpy.data.texts['armatures.py'].as_module()
    NC = bpy.data.texts['node_collide.py'].as_module()

r_print = U.r_print

print()
print("=== reload === (out of bullets)")
print()

# ------- modules -------
# __init__.py
# ui.py
# character_engine.py
# utils.py
# armatures.py
# node_collide.py
# -----------------------

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
    mesh_u_vecs = U.u_vecs(mesh_vecs)    
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
    relative_rot_edges = U.vec_to_vec_pivot(mesh_edge_co[ph.relative], plot_y_vecs, factor=0.5)
    relative_rot_edges.shape = (relative_rot_edges.shape[0] * 2, 3)
    
    # ------- new solver
    move = (relative_rot_edges - ph.current_co[ph.relative_eidx.ravel()])
    np.add.at(ph.current_co, ph.relative_eidx.ravel(), move * ph.rel_rot_multiplier)
    # ------- new solver
        
    # now x
    plot_x_vecs = plot_relative[:, :, 0]    
    x_edge_co = ph.current_co[ph.mx_eidx][ph.relative]
    
    relative_rot_edges = U.vec_to_vec_pivot(x_edge_co, plot_x_vecs, factor=0.5)
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


class Nc():
    pass


def update_node_friction(ph):
    rig = ph.physics_rig
    head_friction = np.array([b.Ce_props.head_friction for b in rig.pose.bones], dtype=np.float32)
    tail_friction = np.array([b.Ce_props.tail_friction for b in rig.pose.bones], dtype=np.float32)

    ph.node_friction = np.zeros(ph.yvc, dtype=np.float32)
    np.add.at(ph.node_friction, ph.mesh_bone_idx, head_friction)
    np.add.at(ph.node_friction, ph.mesh_bone_tail_idx, tail_friction)    

    idxc = ph.mesh_bone_idx.shape[0]
    counts = np.zeros(ph.yvc, dtype=np.float32)
    np.add.at(counts, ph.mesh_bone_idx, np.ones(idxc, dtype=np.float32))
    np.add.at(counts, ph.mesh_bone_tail_idx, np.ones(idxc, dtype=np.float32))

    ph.node_friction = (ph.node_friction / counts)[:, None]


def update_node_sizes(ph):
    
    rig = ph.physics_rig
    head_node_sizes = np.array([b.Ce_props.head_node_size for b in rig.pose.bones], dtype=np.float32)
    tail_node_sizes = np.array([b.Ce_props.tail_node_size for b in rig.pose.bones], dtype=np.float32)
    
    ph.node_sizes = np.zeros(ph.yvc, dtype=np.float32)
    np.add.at(ph.node_sizes, ph.mesh_bone_idx, head_node_sizes)
    np.add.at(ph.node_sizes, ph.mesh_bone_tail_idx, tail_node_sizes)    

    idxc = ph.mesh_bone_idx.shape[0]
    counts = np.zeros(ph.yvc, dtype=np.float32)
    np.add.at(counts, ph.mesh_bone_idx, np.ones(idxc, dtype=np.float32))
    np.add.at(counts, ph.mesh_bone_tail_idx, np.ones(idxc, dtype=np.float32))

    ph.node_sizes = (ph.node_sizes / counts)[:, None]
        
    visual = False
    if visual:
        print("update visual node empties")
    else:
        print("need updater for visual node empties")


def update_colliders(ph):
    ob_list = []
    for ob in bpy.data.objects:
        if ob.name in bpy.context.scene.objects:
            if ob.type == "MESH":
                if ob.Ce_props.collider:
                    ob_list += [ob.id_data]
    
    bpy.context.scene['Ce_collider_objects'] = ob_list


def refresh_collision_objects(ph):
    
    try:            
        ph.physics_rig.name
    except ReferenceError:
        candidates = [ob for ob in bpy.data.objects if ob.Ce_props.data_key == ph.data_key]
        if len(candidates) == 2:
            ph.physics_rig = [ob for ob in candidates if ob.type == "ARMATURE"][0]
            ph.mix_mesh = [ob for ob in candidates if ob.type == "MESH"][0]
            ph.pose_target = ph.physics_rig.Ce_props.pose_target
        else:
            return
            
    rig = ph.physics_rig

    if len(bpy.context.scene['Ce_collider_objects']) == 0:
        return

    nc = Nc()

    nc.yco = ph.current_co[:ph.yvc] # this is a view that will overwrite ph.current_co
    nc.start_yco = np.copy(nc.yco)
    nc.joined_yco = np.empty((ph.yvc, 2, 3), dtype=np.float32)
    nc.joined_yco[:, 0] = nc.yco
    nc.joined_yco[:, 1] = nc.start_yco

    update_node_sizes(ph)
    update_node_friction(ph)
    # get all collider coords in rig space
    WMDS = [] # worlds of matrix destruction
    OBCOS = [] # objective concerns about the letter "S"
    for ob in bpy.context.scene['Ce_collider_objects']:
        OBCOS += [U.absolute_co(ob, world=False)]
        WMDS += [ob.matrix_world] # why monsters don't sweat
    
    mesh_matrix = ph.mix_mesh.matrix_world
    local_matrix = np.linalg.inv(mesh_matrix) @ np.array(WMDS, dtype=np.float32)
    
    nc.rig_space_co = np.empty((0, 3), dtype=np.float32)    
    for i in range(len(OBCOS)):
        rig_space_co = OBCOS[i] @ local_matrix[i][:3, :3].T
        rig_space_co += local_matrix[i][:3, 3]
        nc.rig_space_co = np.concatenate((nc.rig_space_co, rig_space_co), axis=0)
                
    nc.start_rig_space_co = np.copy(nc.rig_space_co)
    # ------------------------------------

    nc.tridex_offset = [0] + [len(ob.data.vertices) for ob in bpy.context.scene['Ce_collider_objects']]
    nc.collider_v_count = np.sum(nc.tridex_offset)
    nc.collider_f_count = np.sum([len(ob.data.polygons) for ob in bpy.context.scene['Ce_collider_objects']])
    
    nc.tridex = np.empty((0, 3), dtype=np.int32)
    nc.triangle_friction = np.empty((0, 1), dtype=np.float32)
    for e, ob in enumerate(bpy.context.scene['Ce_collider_objects']):
        prox = U.prox_object(ob)
        tridex = U.get_tridex(prox, free=True)
        nc.tridex = np.concatenate((nc.tridex, tridex + nc.tridex_offset[e]), axis=0)
        vw = U.get_vertex_weights(prox, "Ce_friction")[nc.tridex] * ob.Ce_props.object_friction
        nc.triangle_friction = np.concatenate((np.sum(vw, axis=1) / 3)[:, None], axis=0)[:, None]
    
    nc.tri_co = nc.rig_space_co[nc.tridex]
    nc.joined_tri_co = np.empty((nc.tridex.shape[0], 6, 3), dtype=np.float32)
    
    nc.joined_tri_co[:] = -5
    
    nc.joined_tri_co[:, :3] = nc.tri_co
    nc.joined_tri_co[:, 3:] = nc.tri_co

    nc.tid = np.arange(nc.tridex.shape[0])
    nc.eid = np.arange(ph.yvc)
    nc.box_max = 375000
    ph.nc = nc
    return nc
    

def collider_objects(ph):
    ph.full_hits[:] = False
    update_colliders(ph)
    if len(bpy.context.scene['Ce_collider_objects']) == 0:
        return

    nc = ph.nc
    
    # check for collider object changes
    collider_v_count = np.sum([len(ob.data.vertices) for ob in bpy.context.scene['Ce_collider_objects']])
    collider_f_count = np.sum([len(ob.data.polygons) for ob in bpy.context.scene['Ce_collider_objects']])

    refresh = False
    if collider_v_count != nc.collider_v_count:
        refresh = True
    if collider_f_count != nc.collider_f_count:
        refresh = True
            
    if refresh:    
        nc = refresh_collision_objects(ph)
    # ----------------------------------
    
    nc.yco = ph.current_co[:ph.yvc] # this is a view that will overwrite ph.current_co
    nc.joined_yco[:, 0] = nc.start_yco
    nc.joined_yco[:, 1] = nc.yco    
    nc.start_yco[:] = nc.yco
    
    # get all collider coords in rig space
    WMDS = [] # worlds of matrix destruction
    OBCOS = [] # objective concerns about the letter "S"
    for ob in bpy.context.scene['Ce_collider_objects']:
        OBCOS += [U.absolute_co(ob, world=False)]
        WMDS += [ob.matrix_world] # why monsters don't sweat
    
    mesh_matrix = ph.mix_mesh.matrix_world
    local_matrix = np.linalg.inv(mesh_matrix) @ np.array(WMDS, dtype=np.float32)
    
    nc.rig_space_co = np.empty((0, 3), dtype=np.float32)    
    
    for i in range(len(OBCOS)):
        rig_space_co = OBCOS[i] @ local_matrix[i][:3, :3].T
        rig_space_co += local_matrix[i][:3, 3]
        nc.rig_space_co = np.concatenate((nc.rig_space_co, rig_space_co), axis=0)
    # ------------------------------------
    nc.joined_tri_co[:, :3] = nc.start_rig_space_co[nc.tridex]
    nc.joined_tri_co[:, 3:] = nc.rig_space_co[nc.tridex]
    nc.start_rig_space_co[:] = nc.rig_space_co
    
    # !!! Collision !!!
    pad = np.max(ph.node_sizes)
    
    nc.e_bounds = U.get_edge_bounds(nc.joined_yco)
    nc.e_bounds[0] -= ph.node_sizes
    nc.e_bounds[1] += ph.node_sizes
    
    nc.t_bounds = U.get_edge_bounds(nc.joined_tri_co)
    # pad
    nc.t_bounds[0] -= pad
    nc.t_bounds[1] += pad
    
    nc.ees = []
    nc.trs = []        
    finished = NC.split_boxes(nc)
    
    NC.ray_check_mem(nc, np.array(nc.ees), np.array(nc.trs), ph)

    

def collision(ph):
    ph.do_collision = True
    #ph.do_collision = False
    ph.skip_refresh = True
    #ph.hit_eidx = []
    if ph.do_collision:
        collider_objects(ph)

    else:
        ### ----- COLLISION ----- ###
        yz = ph.current_co[:, 2][:ph.yvc]
        ph.hit = yz <= 0.0
        
        ph.vel_start[:ph.yvc][:, 2][ph.hit] = 0.0
        ph.current_co[:ph.yvc][ph.hit] = ph.vel_start[:ph.yvc][ph.hit]
        ph.velocity[:ph.yvc][ph.hit] = 0.0
        ### ----- --------- ----- ###


def collision_refresh(ph, slip=False):

    if ph.skip_refresh:
        return
    
    if slip:    
        hit_co = ph.current_co[ph.hit_eidx]
        ori_vecs = hit_co - ph.hit_ori
        ori_dots = np.einsum('ij,ij->i', ori_vecs, ph.hit_norms) - ph.nodes.ravel()
        slick = hit_co + (ph.hit_norms * -ori_dots[:, None])
        
        slip = slick * (1 - ph.friction)
        stick = ph.vel_start[ph.hit_eidx] * ph.friction
        mix = slip + stick
        ph.current_co[ph.hit_eidx] = mix
        ph.vel_start[ph.hit_eidx] = mix
        return

    if ph.do_collision:
        ph.current_co[:ph.yvc][ph.full_hits] = ph.vel_start[:ph.yvc][ph.full_hits]
        # ph.current_co[:ph.yvc][ph.full_hits] = ph.vel_start[:ph.yvc][ph.full_hits]
    else:
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

    ph.target_matricies = np.array([t.matrix_world for t in both], dtype=np.float32)
    ph.head_tail_eidx = np.array([[b.Ce_props.influence_head_idx, b.Ce_props.influence_tail_idx] for b in both])
    
    tails = [t.Ce_props.influence_is_tail for t in both]
    heads = ph.head_tail_eidx[:, 0]
    heads[tails] = ph.head_tail_eidx[tails][:, 1]
    ph.head_tail_idx = heads


def influence_targets(ph):

    if not ph.inf_targets:
        return
    
    both = ph.head_tail_targets
    ph.target_matricies = np.array([t.matrix_world for t in both], dtype=np.float32)
    delete_check = [t.name in bpy.context.scene.objects for t in both]
    if not np.all(delete_check):
        influence_check(ph)
        
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


def record_to_keyframes(ph, frame):
    
    armature = ph.physics_rig
    # Store the current frame
    current_frame = bpy.context.scene.frame_current

    # Set the frame to the desired frame
    bpy.context.scene.Ce_props.skip_handler = True
    
    bpy.context.scene.frame_set(frame)

    A.set_ar_m3_world(ph.physics_rig, ph.physics_m3, locations=ph.current_co[ph.mesh_bone_idx], return_quats=False)
    # Iterate through all bones in the armature
    for bone in armature.pose.bones:
        # Get the pose bone
        pose_bone = armature.pose.bones[bone.name]

        # Keyframe location and rotation
        #pose_bone.location = pose_bone.bone.head
        #pose_bone.rotation_quaternion = pose_bone.bone.matrix.to_quaternion()
        pose_bone.keyframe_insert(data_path="location", index=-1)
        pose_bone.keyframe_insert(data_path="rotation_quaternion", index=-1)

    # Restore the original frame
    bpy.context.scene.frame_set(current_frame)
    bpy.context.scene.Ce_props.skip_handler = False


def live_update(scene=None):

    if bpy.context.scene.Ce_props.skip_handler:
        return
    
    for k, ph in DATA.items():
        
        try:            
            ph.physics_rig.name
        except ReferenceError:
            candidates = [ob for ob in bpy.data.objects if ob.Ce_props.data_key == ph.data_key]
            if len(candidates) == 2:
                ph.physics_rig = [ob for ob in candidates if ob.type == "ARMATURE"][0]
                ph.mix_mesh = [ob for ob in candidates if ob.type == "MESH"][0]
                ph.pose_target = ph.physics_rig.Ce_props.pose_target
                print("hit undo")
            
        Ce = ph.physics_rig.id_data.Ce_props
        if not ph.physics_rig.Ce_props.animated:
            continue
        if not ph.physics_rig.Ce_props.pose_target:
            continue
        #Ce = ph.pose_target.Ce_props
        Ce = ph.physics_rig.Ce_props
        # update mix bmesh -----------
        if ph.mix_mesh.data.is_editmode:
            edit_mode_update(ph)
        else:    
            ph.pinned[:] = False

        ph.target_m3 = A.get_ar_matrix_world(ph.pose_target, m3=True)
        angular_force = ph.physics_rig.Ce_props.angular_force
        #influence_check(ph) # see if there are any targets
            
        for i in range(Ce.sub_frames):

            ### -----GRAVITY----- ###
            gravity_prop = ph.physics_rig.Ce_props.gravity * 0.01
            ph.velocity[:,2] += gravity_prop
            
            ### ----- MIX VELOCITY ----- ###
            velocity_prop = ph.physics_rig.Ce_props.velocity
            vel_mix = ph.current_co - ph.vel_start
            ph.velocity += vel_mix
            ph.velocity[ph.pinned] = 0.0
            ph.velocity *= velocity_prop
            
            ph.current_co += ph.velocity

            collision(ph)
            update_pinned(ph)
            ph.vel_start[:] = ph.current_co
                
            ### -----LINEAR----- ###
            for i in range(Ce.linear_iters):
                linear_solve(ph)
                update_pinned(ph)
                collision_refresh(ph)

            #collision(ph)
            ### -----ANGULAR----- ###
            int_part, decimal = U.split_float(angular_force)
                
            #collision_refresh(ph)

            for i in range(Ce.angular_iters):
                both_angles_mix(ph)
                #collision(ph)
                update_pinned(ph)
                
                for j in range(Ce.linear_post_iters):
                    linear_solve(ph)
                    #collision_refresh(ph)
                    update_pinned(ph)

                    if i < Ce.angular_iters - 1:
                        collision_refresh(ph)#, slip=True)
                        #collision(ph)
            collision(ph)    
        ### ----- UPDATE MESH ----- ###
        if ph.mix_mesh.data.is_editmode:
            for i, v in enumerate(ph.mix_obm.verts):
                if not v.select:    
                    v.co = ph.current_co[i]
                        
            bmesh.update_edit_mesh(ph.mix_mesh.data)

        else: 
            U.set_shape_co(ph.mix_mesh, "Current", ph.current_co)
            ph.mix_mesh.data.update()
            
        ### ----- UPDATE RIG ----- ###
        mesh_bone_co = ph.current_co[ph.mesh_bone_idx]
        A.set_ar_m3_world(ph.physics_rig, ph.physics_m3, locations=mesh_bone_co, return_quats=False)   
        
        if Ce.record_to_keyframes:
            ticker = Ce.ticker
            Ce.ticker += 1

            if ticker >= Ce.skip_frames:
                Ce.ticker = 0
                record_to_keyframes(ph, Ce.record_frame)
                A.set_ar_m3_world(ph.physics_rig, ph.physics_m3, locations=mesh_bone_co, return_quats=False)   
            
            Ce.record_frame += 1
    
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
    bone_count = len(ph.pose_target.pose.bones)

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

    head_mass = np.array([b.Ce_props.head_mass for b in ph.physics_rig.pose.bones], dtype=np.float32)
    tail_mass = np.array([b.Ce_props.tail_mass for b in ph.physics_rig.pose.bones], dtype=np.float32)
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

    relative_balanced = relative_mass / compare_sum
    repeat_balanced = (repeat_mass / compare_sum) * 2

    ph.repeat_eidx = ph.eidx[ph.repeater]
    ph.relative_eidx = ph.eidx[ph.relative]    

    ph.rel_rot_multiplier = ph.rel_rot_multiplier * np.repeat(repeat_balanced, 2)[:, None]
    ph.x_rot_multiplier = ph.x_rot_multiplier * np.repeat(repeat_balanced, 2)[:, None]
    
    # steve


class physics():
    def __init__(self, physics_rig, pose_target, mesh_only=False):
        
        keys = [k for k in DATA.keys()]
        if len(keys) == 0:
            physics_rig.Ce_props.data_key = 737
        else:
            if False:
                # manage dead keys
                dead_keys = []
                for k, v in DATA.items():
                    rig = v.physics_rig
                    if rig.Ce_props.pose_target is None:
                        dead_keys += [k]

                for k in dead_keys:
                    del DATA[k]
                
            # manage key prop
            key_prop = physics_rig.Ce_props.data_key
            if key_prop in DATA.keys():
                ph = DATA[key_prop]
                if ph.physics_rig == physics_rig:
                    del(DATA[key_prop])
                else:
                    new_key = max([k for k in DATA.keys()]) + 1
                    physics_rig.Ce_props.data_key = new_key
        
        DATA[physics_rig.Ce_props.data_key] = self
        self.data_key = physics_rig.Ce_props.data_key
        self.mesh_only = mesh_only
        physics_mesh = mesh_only
        self.physics_mesh = physics_mesh
        
        self.pose_target = pose_target.id_data
        self.physics_rig = physics_rig.id_data
        self.target_m3 = A.get_ar_matrix_world(self.pose_target, m3=True)
        self.physics_m3 = A.get_ar_matrix_world(self.physics_rig, m3=True)
        
        # mass
        self.head_mass = np.array([b.Ce_props.head_mass for b in self.physics_rig.pose.bones], dtype=np.float32)
        self.tail_mass = np.array([b.Ce_props.tail_mass for b in self.physics_rig.pose.bones], dtype=np.float32)
        
        if not mesh_only:            
            relative, repeater = A.get_relative_bones(pose_target)
            self.relative = np.array(relative, dtype=np.int32)
            self.repeater = np.array(repeater, dtype=np.int32)            
            self.physics_rig = physics_rig
            self.pose_target = pose_target
            bc = len(physics_rig.pose.bones)
            self.bc = bc
        
        # mix    
        mix_mesh, edges, mx_eidx, x_start_eidx, mix_vidx, x_vidx, y_vidx = A.make_mix_mesh(physics_rig, self)
        mix_mesh.Ce_props.data_key = self.data_key
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
        U.get_shape_co_mode(ob=mix_mesh, co=self.target_co_mix, key='Target')
        self.current_co = np.empty((vc_mix, 3), dtype=np.float32)
        U.get_shape_co_mode(ob=mix_mesh, co=self.current_co, key='Current')
        self.previous_co = np.copy(self.current_co) # for dynamic collisions
        
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
        self.full_hits = np.zeros(self.yvc, dtype=bool)
        
        # setup mass:
        distribute_mass(self)
        # ----------- come back here
        
        # collision objects
        update_colliders(self)
        self.nc = refresh_collision_objects(self)
        self.skip_refresh = True
        # -----------------
        
        ### ===== TESTING ===== ###
        self.start_tm3 = A.get_ar_matrix_world(self.pose_target, m3=True)[self.relative]
        # influence
        influence_check(self)
        #self.inf_targets = False



if False:
#if True:
    
    #physics_rig = bpy.data.objects['tar']
    #pose_target = bpy.data.objects['tar']
    physics_mesh = bpy.context.object
    ph = physics(physics_rig=None, pose_target=None, mesh_only=physics_mesh)
    
    if True:
        U.co_to_shape(physics_mesh, co=None, key="Current")
        U.co_to_shape(physics_mesh, co=None, key="Target")
        U.co_to_shape(physics_mesh, co=None, key="Basis")


    install_handler(live=False, animated=True)
