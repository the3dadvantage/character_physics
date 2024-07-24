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
    #S = bpy.data.texts['solvers.py'].as_module()

r_print = U.r_print

print()
print("=== reload === (out of bullets)")
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

def yl_data(ph):
    target_edge_vecs = ph.target_co_mix[ph.eidx[:, 1]] - ph.target_co_mix[ph.eidx[:, 0]]
    ph.yl_target_lengths = U.measure_vecs(target_edge_vecs)
        
    uni, inverse, counts = np.unique(ph.eidx, return_inverse=True, return_counts=True)
    ph.yl_force = (1 / counts[inverse])[:, None] * 0.9
    
    rig = ph.physics_rig
    mass = np.array([b.CP_props.head_mass for b in rig.pose.bones])
    rep_mass = np.repeat(mass, 2)
    mass_holder = np.zeros(ph.yvc, dtype=np.float32)

    np.add.at(mass_holder, ph.eidx.ravel(), rep_mass)

    m_sort = np.sort(mass_holder[ph.eidx], axis=1)
    div = 1 / m_sort[:, 1]
    normalized = mass_holder[ph.eidx] * div[:, None]    
    
    ph.yl_force *= np.roll(normalized, 1, 1).ravel()[:, None]


def linear_solve(ph):
    
    vecs = ph.current_co[ph.eidx[:, 1]] - ph.current_co[ph.eidx[:, 0]]
    current_lengths = np.sqrt(np.einsum('ij,ij->i', vecs, vecs))
    div = ph.yl_target_lengths / current_lengths
    plot = vecs * div[:, None]
    move = (vecs - plot)
    yl_force = ph.yl_force #* ph.physics_rig.CP_props.test_prop
    np.add.at(ph.current_co, ph.eidx[:, 0], move * yl_force[::2])
    np.subtract.at(ph.current_co, ph.eidx[:, 1], move * yl_force[1::2])


def __linear_solve(ph):
    vecs = ph.current_co[ph.mix_eidx[:, 1]] - ph.current_co[ph.mix_eidx[:, 0]]
    current_lengths = np.sqrt(np.einsum('ij,ij->i', vecs, vecs))
    div = ph.target_edge_lengths / current_lengths
    plot = vecs * div[:, None]
    move = (vecs - plot)
        
    np.add.at(ph.current_co, ph.bone_and_x_eidx[:, 0], move[:ph.bone_count * 2] * ph.left_move_force)
    np.subtract.at(ph.current_co, ph.mix_eidx[:, 1], move * ph.right_move_force)


def q_rotate(co, w, axis):
    """Takes an N x 3 numpy array and returns that array rotated around
    the axis by the angle in radians w. (standard quaternion)"""    
    move1 = np.cross(axis, co)
    move2 = np.cross(axis, move1)
    move1 *= w
    return co + (move1 + move2) * 2


def get_quat(rad, axis, normalize=False):
    if normalize:
        axis = axis / np.sqrt(axis @ axis)
    theta = (rad * 0.5)
    w = np.cos(theta)
    q_axis = axis * np.sin(theta)
    return w, np.nan_to_num(q_axis)


def vec_rot(co, v1, v2, factor=1):

    dot = v1 @ v2
    angle = np.arccos(dot)
    angle *= factor
    
    cross = np.cross(v1, v2)
    w, axis = get_quat(angle, cross, normalize=True)
    rot_co = q_rotate(co, w, axis)
    return rot_co
    

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
    

def update_pinned(ph, vel=False):
    ph.current_co[ph.pinned] = ph.vel_start[ph.pinned]
    #influence_targets(ph, vel=vel)
    

class Nc():
    pass


def update_node_friction(ph):
    rig = ph.physics_rig
    head_friction = np.array([b.CP_props.head_friction for b in rig.pose.bones], dtype=np.float32)
    tail_friction = np.array([b.CP_props.tail_friction for b in rig.pose.bones], dtype=np.float32)

    ph.node_friction = np.zeros(ph.yvc, dtype=np.float32)
    np.add.at(ph.node_friction, ph.mesh_bone_idx, head_friction)
    np.add.at(ph.node_friction, ph.mesh_bone_tail_idx, tail_friction)    

    idxc = ph.mesh_bone_idx.shape[0]
    counts = np.zeros(ph.yvc, dtype=np.float32)
    np.add.at(counts, ph.mesh_bone_idx, np.ones(idxc, dtype=np.float32))
    np.add.at(counts, ph.mesh_bone_tail_idx, np.ones(idxc, dtype=np.float32))

    ph.node_friction = (ph.node_friction / counts)[:, None]


def update_mocap_influence(ph):
    rig = ph.physics_rig

    head_mocap_influence = np.array([b.CP_props.head_mocap_influence for b in rig.pose.bones], dtype=np.float32)
    tail_mocap_influence = np.array([b.CP_props.tail_mocap_influence for b in rig.pose.bones], dtype=np.float32)
    
    ph.mocap_influence = np.zeros(ph.yvc, dtype=np.float32)
    np.add.at(ph.mocap_influence, ph.mesh_bone_idx, head_mocap_influence)
    np.add.at(ph.mocap_influence, ph.mesh_bone_tail_idx, tail_mocap_influence)    

    idxc = ph.mesh_bone_idx.shape[0]
    counts = np.zeros(ph.yvc, dtype=np.float32)
    np.add.at(counts, ph.mesh_bone_idx, np.ones(idxc, dtype=np.float32))
    np.add.at(counts, ph.mesh_bone_tail_idx, np.ones(idxc, dtype=np.float32))

    ph.mocap_influence = (ph.mocap_influence / counts)[:, None]


def update_node_sizes(ph):
    rig = ph.physics_rig

    head_node_sizes = np.array([b.CP_props.head_node_size for b in rig.pose.bones], dtype=np.float32)
    tail_node_sizes = np.array([b.CP_props.tail_node_size for b in rig.pose.bones], dtype=np.float32)
    
    ph.node_sizes = np.zeros(ph.yvc, dtype=np.float32)
    np.add.at(ph.node_sizes, ph.mesh_bone_idx, head_node_sizes)
    np.add.at(ph.node_sizes, ph.mesh_bone_tail_idx, tail_node_sizes)    

    idxc = ph.mesh_bone_idx.shape[0]
    counts = np.zeros(ph.yvc, dtype=np.float32)
    np.add.at(counts, ph.mesh_bone_idx, np.ones(idxc, dtype=np.float32))
    np.add.at(counts, ph.mesh_bone_tail_idx, np.ones(idxc, dtype=np.float32))

    ph.node_sizes = (ph.node_sizes / counts)[:, None]

        
def influence_check(ph):
    """Check for needed updates outside of
    iterations.
    !!! can run this on prop callbacks when
    creating heads, tail, or adjusting
    values on heads or tails !!!"""
    
    bpy.context.view_layer.update()
    
    ph.inf_rot_bone_idx = []
    for e, b in enumerate(ph.physics_rig.pose.bones):

        if b.CP_props.target_object:
            if not (b.CP_props.target_object.name in bpy.context.scene.objects):
                b.CP_props.target_object = None

        if b.CP_props.tail_target_object:
            if not (b.CP_props.tail_target_object.name in bpy.context.scene.objects):
                b.CP_props.tail_target_object = None

        if b.CP_props.rotation_target_object:
            if not (b.CP_props.rotation_target_object.name in bpy.context.scene.objects):
                b.CP_props.rotation_target_object = None
            else:
                ph.inf_rot_bone_idx += [e]
    
    ph.inf_rot_edges_y = ph.eidx[ph.inf_rot_bone_idx]
    ph.inf_rot_edges_x = ph.mx_eidx[ph.inf_rot_bone_idx]
        
    targets = [b.CP_props.target_object for b in ph.physics_rig.pose.bones if b.CP_props.target_object]
    tail_targets = [b.CP_props.tail_target_object for b in ph.physics_rig.pose.bones if b.CP_props.tail_target_object]
    rotation_targets = [b.CP_props.rotation_target_object for b in ph.physics_rig.pose.bones if b.CP_props.rotation_target_object]
    ph.stabilizer = [ob.CP_props.stabilizer for ob in rotation_targets]
    
    both = targets + tail_targets + rotation_targets
    
    ph.head_tail_targets = both
    ph.inf_anim_check = np.array([[it.CP_props.influence_spring, it.CP_props.influence_directional, it.CP_props.influence_rotation] for it in both], dtype=np.float32).ravel()
    
    ph.inf_targets = True
    ph.both_rot_matrix_idx = np.arange(len(both))[len(targets) + len(tail_targets):]
    
    if len(both) == 0:
        ph.inf_targets = None
        return

    ph.target_matricies = np.array([t.matrix_world for t in both], dtype=np.float32)
    ph.head_tail_eidx = np.array([[b.CP_props.influence_head_idx, b.CP_props.influence_tail_idx] for b in both])
    
    tails = [t.CP_props.influence_is_tail for t in both]
    heads = ph.head_tail_eidx[:, 0]
    heads[tails] = ph.head_tail_eidx[tails][:, 1]
    ph.head_tail_idx = heads

    ph.inf_spring_influence = np.array([b.CP_props.influence_spring for b in both], dtype=np.float32)[:, None]
    ph.inf_linear_influence = np.array([b.CP_props.influence_directional for b in both], dtype=np.float32)[:, None]
    ph.inf_rotation_influence = np.array([b.CP_props.influence_rotation for b in rotation_targets], dtype=np.float32)
    
    ph.any_spring = np.any(ph.inf_spring_influence != 0.0)
    ph.any_linear = np.any(ph.inf_linear_influence != 0.0)
    ph.any_rotation = np.any(ph.inf_rotation_influence != 0.0)    

    if (not ph.any_spring) & (not ph.any_linear) & (not ph.any_rotation):
        ph.inf_targets = None
        return
        

def influence_targets(ph, vel=False):

    both = ph.head_tail_targets
    try:    
        delete_check = [t.name in bpy.context.scene.objects for t in both]
    except:    
        influence_check(ph)
    
    if not np.all(delete_check):
        influence_check(ph)
        both = ph.head_tail_targets

    if len(both) == 0:
        ph.inf_targets = None
        return

    inf_anim_check = np.array([[it.CP_props.influence_spring, it.CP_props.influence_directional, it.CP_props.influence_rotation] for it in both], dtype=np.float32).ravel()
    inf_dif = ph.inf_anim_check - inf_anim_check
    if np.any(inf_anim_check != 0.0):
        ph.inf_targets = True
    
    if np.any(inf_dif != 0.0):
        influence_check(ph)    
    
    if not ph.inf_targets:
        return

    # --------------
    WMDS = []
    stabilizer = []
    for ob in both:
        WMDS += [ob.matrix_world] # why monsters don't sweat
        stabilizer += [ob.CP_props.stabilizer]
        
    mesh_matrix = ph.mix_mesh.matrix_world
    ph.target_matricies = np.linalg.inv(mesh_matrix) @ np.array(WMDS, dtype=np.float32)
    # --------------
        
    spring_influence = ph.inf_spring_influence
    linear_influence = ph.inf_linear_influence
    rotation_influence = ph.inf_rotation_influence
    
    any_spring = ph.any_spring
    any_linear = ph.any_linear
    any_rotation = ph.any_rotation

    BM = ph.target_matricies

    head_tail = ph.head_tail_eidx
    head_co = ph.current_co[head_tail[:, 0]]
    tail_co = ph.current_co[head_tail[:, 1]]

    heads = ph.head_tail_idx
    
    if any_linear:
        y = BM[:, :3, 1]
        lin_move = y * (linear_influence * 0.001)    
        np.add.at(ph.current_co, heads, lin_move)
        
    if any_spring:
        both_co = BM[:, :3, 3]
        move = both_co - ph.current_co[ph.head_tail_idx]
        move *= spring_influence        
        np.add.at(ph.current_co, ph.head_tail_idx, move)        
    
    if any_rotation:
        
        rms = BM[ph.both_rot_matrix_idx]
        for e, M in enumerate(rms):

            factor = ph.inf_rotation_influence[e]
            em3 = M[:3, :3] * 0.5
            
            xid = ph.inf_rot_edges_x[e]
            yid = ph.inf_rot_edges_y[e]
            
            y_dist = ph.y_dists[e]
            
            x = ph.current_co[ph.inf_rot_edges_x[e]] * 0.5
            y = ph.current_co[ph.inf_rot_edges_y[e]]
            
            xv = x[1] - x[0]
            yv = y[1] - y[0]
            yv = (yv / np.sqrt(yv @ yv)) * 0.5
            
            emxv = em3[:, 0]
            emyv = em3[:, 1]            
            
            y_dif = (emyv - yv) * y_dist * factor
            x_dif = (emxv - xv) * factor
            
            #loc = np.mean(ph.current_co[yid], axis=0)
            ready = True
            #ready = False
            if ready:
                #ph.current_co[yid[0]] -= y_dif# * .4
                ph.current_co[yid[1]] += y_dif
                
                #ph.velocity[[yid[0]]] += y_dif
                #if False:    
                if not ph.stabilizer[e]: # change
                    #if False:    
                    ph.current_co[yid[0]] -= y_dif
                    if vel:
                        ph.vel_start[yid] = ph.current_co[yid]
                
                

                
                #print("need a record mode that ignores location")
                #dif = loc - np.mean(ph.current_co[yid], axis=0)
                #print(np.sqrt(dif @ dif))
                #ph.current_co[yid[0]] -= dif
                #ph.current_co[yid[1]] -= dif
                
                ph.current_co[xid[0]] -= x_dif
                ph.current_co[xid[1]] += x_dif
                
                #xco[[e, e + 1]] = ph.current_co[xid]
                #yco[[e, e + 1]] = ph.current_co[yid]

       # return xco, yco
#            if False:
#                c1 = bpy.data.objects['c1']
#                c2 = bpy.data.objects['c2']
#                #c3 = bpy.data.objects['c2']
#                #c4 = bpy.data.objects['c2']
#                
#                c1.location = ph.current_co[yid[0]]
#                c2.location = ph.current_co[yid[0]] - y_dif
#                #c3.location = ph.current_co[yid[1]] + y_dif
#                #c4.location = ph.current_co[yid[1]] + y_dif
        

def record_to_keyframes(rig, frame, action_name):
        
    bpy.context.scene.CP_props.skip_handler = True
    current_frame = bpy.context.scene.frame_current
    bpy.context.scene.frame_set(frame)

    action_name = rig.CP_props.action_name
    if action_name in bpy.data.actions:
        action = bpy.data.actions[action_name]
    else:    
        action = bpy.data.actions.new(name=action_name)
    
    if rig.animation_data is None:
        rig.animation_data_create()    
    rig.animation_data.action = action

    if rig.CP_props.record_selection_only:
        for bone in rig.pose.bones:
            if bone.bone.select:    
                pose_bone = rig.pose.bones[bone.name]
                if rig.CP_props.record_location:    
                    pose_bone.keyframe_insert(data_path="location", index=-1)#, group="Ce_test_action")
                if rig.CP_props.record_rotaion:    
                    pose_bone.keyframe_insert(data_path="rotation_quaternion", index=-1)#, group="Ce_test_action")
    
    else:
        for bone in rig.pose.bones:
            pose_bone = rig.pose.bones[bone.name]
            if rig.CP_props.record_location:    
                pose_bone.keyframe_insert(data_path="location", index=-1)#, group="Ce_test_action")
            if rig.CP_props.record_rotation:    
                pose_bone.keyframe_insert(data_path="rotation_quaternion", index=-1)#, group="Ce_test_action")
    
    rig.animation_data.action = None
    bpy.context.scene.frame_set(current_frame)
    bpy.context.scene.CP_props.skip_handler = False


def update_colliders(ph):
    ob_list = []
    for ob in bpy.data.objects:
        if ob.name in bpy.context.scene.objects:
            if ob.type == "MESH":
                if ob.CP_props.collider:
                    ob_list += [ob.id_data]
    
    bpy.context.scene['Ce_collider_objects'] = ob_list
    

def do_collision(ph):
    ph.full_hits[:] = False

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
        nc = U.refresh_collision_objects(ph)

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

    # update tris ------------------------------------
    nc.joined_tri_co[:, :3] = nc.start_rig_space_co[nc.tridex]
    nc.joined_tri_co[:, 3:] = nc.rig_space_co[nc.tridex]
    nc.start_rig_space_co[:] = nc.rig_space_co
    
    ph.joined_tri_co = nc.joined_tri_co
    
    # update rig -------------------------------------
    nc.yco = ph.current_co[:ph.yvc] # this is a view that will overwrite ph.current_co
    nc.joined_yco[:, 0] = nc.start_yco
    nc.joined_yco[:, 1] = nc.yco    
    nc.start_yco[:] = nc.yco
    
    # !!! Collision !!!
    pad = np.max(ph.node_sizes)
    
    nc.e_bounds = U.get_edge_bounds(nc.joined_yco)
    
    nc.e_bounds[0] -= ph.node_sizes
    nc.e_bounds[1] += ph.node_sizes
    
    nc.t_bounds = U.get_edge_bounds(nc.joined_tri_co)
    
    nc.abs_min = np.min(nc.e_bounds[0], axis=0)
    nc.abs_max = np.max(nc.e_bounds[1], axis=0)
    
    check1 = np.all(nc.t_bounds[0] <= nc.abs_max, axis=1)
    check2 = np.all(nc.t_bounds[1] >= nc.abs_min, axis=1)
    
    double_check = check1 & check2
    
    #double_check[:] = True
    
    nc.btid = nc.tid[double_check]
    #nc.btid = nc.tid#[double_check]
    
    nc.t_bounds[0] = nc.t_bounds[0][double_check]
    nc.t_bounds[1] = nc.t_bounds[1][double_check]
    
    nc.ees = []
    nc.trs = []        
    finished = NC.split_boxes(nc)
        
    #NC.ray_check_mem(nc, np.array(nc.ees, dtype=np.int32), np.array(nc.trs, dtype=np.int32), ph)
    NC.np_closest_point_mesh(ph, nc, np.array(nc.ees, dtype=np.int32), np.array(nc.trs, dtype=np.int32))
         

def update_display(ph):
    ### ----- UPDATE RIG ----- ###
    U.vecs_to_matrix(ph, cp=True) # makes the armature line up better
    
    CP = ph.physics_rig.CP_props
    mesh_bone_co = ph.current_co[ph.mesh_bone_idx]
            
    rig = ph.physics_rig
    alt_target = rig.CP_props.alternate_target
    if alt_target:
        rig = alt_target

    A.fast_set_m3(ph.physics_m4, rig, ph.physics_m3, mesh_bone_co)
    
    ### ----- UPDATE MESH ----- ###
    if ph.mix_mesh.data.is_editmode:
        for i, v in enumerate(ph.mix_obm.verts):
            if not v.select:    
                v.co = ph.current_co[i]
                    
        bmesh.update_edit_mesh(ph.mix_mesh.data)

    else: 
        U.set_shape_co(ph.mix_mesh, "Current", ph.current_co)
        ph.mix_mesh.data.update()
        
    if ph.physics_rig.CP_props.reset_at_frame == bpy.context.scene.frame_current:
        U.rig_reset(DATA, ph)
    
    if CP.record_to_keyframes:
        ticker = CP.ticker
        CP.ticker += 1
        
        if ticker >= CP.skip_frames:
            CP.ticker = 0
            bpy.context.view_layer.update()
            record_to_keyframes(rig, CP.record_frame, ph.physics_rig.CP_props.action_name)
        
        if CP.record_frame == CP.end_frame:
            CP.record_to_keyframes = False
            return
        
        CP.record_frame += 1


def collision_refresh(ph, last_time=False, first_time=False):
    
    col_margin = bpy.context.scene.CP_props.collision_margin
    if ph.tco_idx.shape[0] > 0:
        offset_co = ph.current_co[ph.tco_idx] - (ph.tri_normals * ph.t_node_sizes[:, None])
        vecs = offset_co - ph.tri_origins
        dots = U.compare_vecs(vecs, ph.tri_normals) 
        below = dots <= col_margin
        
        tco_idx = ph.tco_idx[below]
        pre_tidx = ph.pre_tidx[below]
        tri_norms_below = ph.tri_normals[below]
        move = tri_norms_below * -dots[below][:, None]
        
        slick = ph.current_co[tco_idx] + (move * 0.999)
                
        tco = ph.joined_tri_co[pre_tidx]
        current_tco = tco[:, 3:]
        weight_plot = current_tco * ph.hit_weights[below][:, :, None]
        tri_plot = np.sum(weight_plot, axis=1)
        
        sticky_move = tri_norms_below * ph.t_node_sizes[below][:, None]# * holdback
        sticky = tri_plot + (sticky_move * 0.999)

        node_friction = ph.node_friction[tco_idx]
        triangle_friction = ph.triangle_friction[pre_tidx]
        friction = node_friction * triangle_friction
        
        slip = slick * (1 - friction)
        stick = sticky * friction
        mix = slip + stick
        ph.current_co[tco_idx] = mix
        
        static_friction = False
        static_friction = True
        if static_friction:
            static = ph.physics_rig.CP_props.object_static_friction ** 2
            static_dif = sticky - mix
            static_dist = U.compare_vecs(static_dif, static_dif)
            static_bool = static_dist < static
            static_tco = tco_idx[static_bool]
            ph.current_co[static_tco] = sticky[static_bool]

        if last_time:
            ph.tco_idx = ph.tco_idx[below]
            ph.tri_normals = ph.tri_normals[below]
            ph.t_node_sizes = ph.t_node_sizes[below]
            ph.hit_weights = ph.hit_weights[below]
            ph.pre_tidx = pre_tidx
            ph.tri_origins = ph.tri_origins[below]
            ph.friction = ph.friction[below]
    
    if ph.eco_idx.shape[0] > 0:
        offset_co = ph.current_co[ph.eco_idx] - (ph.edge_normals * ph.e_node_sizes[:, None])
        vecs = offset_co - ph.edge_origins
        dots = U.compare_vecs(vecs, ph.edge_normals)
        below = dots <= col_margin
        edge_normals_below = ph.edge_normals[below]
        move = edge_normals_below * -dots[below][:, None]
        ph.current_co[ph.eco_idx[below]] += (move * 0.999)


def plot_empties(var):
    obs = [bpy.data.objects['sp' + str(i)] for i in range(10)]
    obs[0].location = ph.current_co[1] + (ph.physics_m3[1].T[0] * .5)
    obs[1].location = ph.current_co[1] + (ph.physics_m3[1].T[1] * .5)
    obs[2].location = ph.current_co[1] + (ph.physics_m3[1].T[2] * .5)
    
    obs[6].location = ph.current_co[1] + (plot_relative[0].T[0] * .4)
    obs[7].location = ph.current_co[1] + (plot_relative[0].T[1] * .4)
    obs[8].location = ph.current_co[1] + (plot_relative[0].T[2] * .4)


def angular_solve(ph):
        
    rot_to_plot = np.linalg.inv(ph.target_m3)[ph.repeater] @ ph.target_m3[ph.relative]
    plot_relative = ph.physics_m3[ph.repeater] @ rot_to_plot
    
    difs = (plot_relative - ph.physics_m3[ph.relative])
        
    y_difs = difs[:, :, 1]
    x_difs = difs[:, :, 0]
            
    np.subtract.at(ph.current_co, ph.eidx_rel[:, 0], y_difs * ph.eidx_rel_counts[::2])
    np.add.at(ph.current_co, ph.eidx_rel[:, 1], y_difs * ph.eidx_rel_counts[1::2])
            
    np.subtract.at(ph.current_co, ph.mx_eidx_relative[:, 0], x_difs * ph.x_eidx_rel_counts[::2])
    np.add.at(ph.current_co, ph.mx_eidx_relative[:, 1], x_difs * ph.x_eidx_rel_counts[1::2])


def ___angular_solve(ph):
    
    '''
    was wondering if there was a way to fix
    drift by using a mirrored mesh and averaging...
    '''
    
    #pivot = np.mean(ph.current_co[ph.eidx], axis=1)
    #print(pivot)
    base = np.unique(ph.eidx[:, 0])
    bco = ph.current_co[base]

    rot_to_plot = np.linalg.inv(ph.target_m3)[ph.repeater] @ ph.target_m3[ph.relative]
    plot_relative = ph.physics_m3[ph.repeater] @ rot_to_plot
    
    difs = (plot_relative - ph.physics_m3[ph.relative])
        
    y_difs = difs[:, :, 1]
    x_difs = difs[:, :, 0]
            
    np.subtract.at(ph.current_co, ph.eidx_rel[:, 0], y_difs * ph.eidx_rel_counts[::2])
    np.add.at(ph.current_co, ph.eidx_rel[:, 1], y_difs * ph.eidx_rel_counts[1::2])
            
    np.subtract.at(ph.current_co, ph.mx_eidx_relative[:, 0], x_difs * ph.x_eidx_rel_counts[::2])
    np.add.at(ph.current_co, ph.mx_eidx_relative[:, 1], x_difs * ph.x_eidx_rel_counts[1::2])

    #pivot_2 = np.mean(ph.current_co[ph.eidx], axis=1)
    
    bco2 = ph.current_co[base]
    dif = bco2 - bco
    
    #ph.current_co[base] += dif * ph.physics_rig.CP_props.test_prop
    
    #p_move = pivot - pivot_2
    
    #uni, counts = np.unique(ph.eidx, return_counts=True)
    #p_move /= counts[:, None]
    
    #np.add.at(ph.current_co, ph.eidx[:, 0], (p_move / counts[ph.eidx[:, 0]]) * ph.physics_rig.CP_props.test_prop)
    #np.add.at(ph.current_co, ph.eidx[:, 1], (p_move / counts[ph.eidx[:, 1]]) * ph.physics_rig.CP_props.test_prop)
    


def __angular_solve(ph, rot_x=False, fix_x=False):
    '''Keeping this version for testing drift fix'''    
    rot_to_plot = np.linalg.inv(ph.target_m3)[ph.repeater] @ ph.target_m3[ph.relative]
    plot_relative = ph.physics_m3[ph.repeater] @ rot_to_plot
    
    difs = (plot_relative - ph.physics_m3[ph.relative]) * 0.5
    y_difs = difs[:, :, 1]
    
    if rot_x:
        y_edges = ph.current_co[ph.eidx_rel]
        y_vecs = U.u_vecs(y_edges[:, 1] - y_edges[:, 0])
    
    np.subtract.at(ph.current_co, ph.eidx_rel[:, 0], y_difs * ph.eidx_rel_counts[::2])
    np.add.at(ph.current_co, ph.eidx_rel[:, 1], y_difs * ph.eidx_rel_counts[1::2])
    
    x_difs = difs[:, :, 0]
    
    if rot_x:
        y_edges = ph.current_co[ph.eidx_rel]
        y_vecs2 = U.u_vecs(y_edges[:, 1] - y_edges[:, 0])
        x_difs = np.copy(difs[:, :, 0])
        
        for i in range(x_difs.shape[0]):
            co = x_difs[i]
            v1 = y_vecs[i]
            v2 = y_vecs2[i]
            nx = vec_rot(co, v1, v2, factor=1)
            nans = np.any(np.isnan(nx))
            if not nans:
                x_difs[i] = nx

    if fix_x:
        y_eco = ph.current_co[ph.eidx]
        pivots = np.mean(y_eco, axis=1)
        
        x_eco = ph.current_co[ph.mx_eidx]
        x_pivots = np.mean(x_eco, axis=1)
        
        dif = pivots - x_pivots
        ph.current_co[ph.mx_eidx] += dif[:, None]
        
    np.subtract.at(ph.current_co, ph.mx_eidx_relative[:, 0], x_difs * ph.x_eidx_rel_counts[::2])
    np.add.at(ph.current_co, ph.mx_eidx_relative[:, 1], x_difs * ph.x_eidx_rel_counts[1::2])


def update_mocap(ph):
    rig = ph.physics_rig
    CP = rig.CP_props
    if CP.mocap_target:
        mocap_influence = CP.mocap_influence
        if mocap_influence > 0.0:
                    
            mocap_co = ph.mocap_co
            
            target = CP.mocap_target
            tail = A.get_bone_tail(target, tail=None)
            head = A.get_bone_head(target, head=None)
            
            ph.mocap_co[ph.mesh_bone_idx] = head
            ph.mocap_co[ph.mesh_bone_tail_idx] = tail
            
            dif = (ph.mocap_co - ph.current_co[:ph.yvc]) * ph.mocap_influence * mocap_influence
            
            ph.current_co[:ph.yvc] += dif
        

def get_advanced_m3(ph):

    rig = ph.physics_rig
    p_rig = ph.pose_target
    CP = rig.CP_props
    action_name = CP.advanced_anim_action.name
    
    #print(CP.advanced_anim_action, "this is action")
    #print(CP.advanced_anim_action)
    
    if action_name in bpy.data.actions:

        current_action = p_rig.animation_data.action
        current_frame = bpy.context.scene.frame_current
        bpy.context.view_layer.update()
        #current_m4 = A.just_m4(ph.pose_target, ph.target_m4)
        
        
        action = bpy.data.actions[action_name]
        p_rig.animation_data.action = action

        bpy.context.scene.CP_props.skip_handler = True
        
        start = CP.target_frame_start
        end = CP.target_frame_end
        current = CP.target_frame_current
        reset = CP.target_frame_reset
        
        resetting = current_frame == reset
        if resetting:
            CP.target_frame_current = start
            #bpy.context.scene.frame_set(CP.target_frame_current)
            #return
        
        if current < start:
            CP.target_frame_current = start
            #print("we reset to start")
        if current > end:
            CP.target_frame_current = start
            #print("we reset to start because end")
        
        bpy.context.scene.frame_set(CP.target_frame_current)
        if not resetting:    
            CP.target_frame_current += 1
        #print(CP.target_frame_current)
        #print("update armature here?")
        target_m3 = A.fast_m3(ph.pose_target, m3=ph.target_m4)
        
        # restore settings
        #A.fast_set_m3(ph.physics_m4, p_rig, current_m3)#, mesh_bone_co)
        p_rig.animation_data.action = current_action
        bpy.context.scene.frame_set(current_frame)
        #A.just_set_m4(ph.target_m4, p_rig)#, mesh_bone_co)
        
        bpy.context.scene.CP_props.skip_handler = False

        return target_m3

    target_m3 = A.fast_m3(ph.pose_target, m3=ph.target_m4)
    return target_m3


def anim_update(scene=None):
    run_update(mode=0)


def live_update(scene=None):
    run_update(mode=1)
    delay = 0.0
    return delay


def run_update(mode):

    debug = True
    debug = False
    if debug:
        print()
        print("--- new live ---")
    
    for k, ph in DATA.items():

        #S.test(ph)

        # check if object references are evil zombies
        U.validate_references(ph)
        rig = ph.physics_rig
        if rig is None:
            del(DATA[k])
        CP = rig.CP_props
        
        if bpy.context.scene.CP_props.skip_handler:
            if CP.record_to_keyframes:
                continue
            if CP.advanced_mode:
                continue
            
        if mode == 1:
            if not CP.live:
                continue    

        if mode == 0:
            if not CP.animated:
                continue

        if not ph.physics_rig.CP_props.pose_target:
            continue
            
        update_colliders(ph)

        # update mix bmesh -----------
        if ph.mix_mesh.data.is_editmode:
            edit_mode_update(ph)
        else:    
            ph.pinned[:] = False

        if CP.advanced_mode:
            ph.target_m3 = get_advanced_m3(ph)
            
        else:
            ph.target_m3 = A.fast_m3(ph.pose_target, m3=ph.target_m4)
        
        angular_force = CP.angular_force
        velocity_prop = CP.velocity
            
        ### ----- GRAVITY ----- ###
        gravity_prop = CP.gravity * 0.01
        #if CP.advanced_gravity:
        if CP.gravity_object:
            grav_matrix = np.linalg.inv(rig.matrix_world) @ np.array(CP.gravity_object.matrix_world, dtype=np.float32)
            grav_vec = grav_matrix[:3, 2] * gravity_prop
        else:
            grav_vec = np.linalg.inv(rig.matrix_world)[:3, 2] * gravity_prop
            

        for i in range(CP.sub_frames):            
            
            ### ----- MIX VELOCITY ----- ###
            vel_mix = ph.current_co - ph.vel_start
            ph.velocity += vel_mix
            ph.velocity[ph.pinned] = 0.0
            ph.velocity *= velocity_prop

            ph.velocity += grav_vec
            ph.velocity[ph.tco_idx] *= (1 - ph.friction)
            ph.current_co += ph.velocity
            update_mocap(ph)
            
            update_pinned(ph)
            influence_targets(ph)#, vel=False)

            do_collision(ph)            

            ph.vel_start[:] = ph.current_co

            mix = False
            mix = True
            if mix:
                li = CP.linear_iters
                ai = CP.angular_iters
                for i in range(max(li, ai)):                
                    if i < ai:
                        #cp = i % 2 == 0
                        #cp = False
                        cp = True
                        #print(cp)
                        U.vecs_to_matrix(ph, cp=True)
                        angular_solve(ph)#, rot_x=True)#, fix_x=True)
                        linear_solve(ph)
                    #if i < li:
                    else:    
                        linear_solve(ph)
                    update_pinned(ph)
                    #update_mocap(ph)
                    influence_targets(ph)#, vel=False, rot_update=inf_rot_update)
                    collision_refresh(ph)
                
                #update_mocap(ph)
                #collision_refresh(ph)
                    
            else:
                ### -----LINEAR----- ###            
                for i in range(CP.linear_iters):
                    linear_solve(ph)
                    update_pinned(ph)
                    collision_refresh(ph)
                
                ### -----ANGULAR----- ###
                int_part, decimal = U.split_float(angular_force)
        
                for i in range(CP.angular_iters):
                    U.vecs_to_matrix(ph)
                    angular_solve(ph)
                    update_pinned(ph)

                    for j in range(CP.linear_post_iters):
                        
                        linear_solve(ph)
                        update_pinned(ph)
                        last_time = (i + 1 == CP.angular_iters) & (j + 1 == CP.linear_post_iters)
                        collision_refresh(ph, last_time)

            #update_pinned(ph, vel=True)
            #update_mocap(ph)
            #influence_targets(ph, vel=True)#, rot_update=inf_rot_update)
            do_collision(ph)
            #ph.previous_co[:] = ph.current_co
        update_display(ph)
            
    #print(time.time() - T)
    delay = 0.0
    return delay


### ================= ###
#                       #
#   ===== SETUP =====   #
#                       #
### ================= ###
def install_handler_live(skip=False):
    """Adds a live or animated updater"""
    
#    print("running install hanlder")
#    print("handler live", live)
#    print("handler animated", animated)
#    
    # clean dead versions of the animated handler
#    handler_names = np.array([i.__name__ for i in bpy.app.handlers.frame_change_post])
#    booly = [i == 'anim_update' for i in handler_names]
#    idx = np.arange(handler_names.shape[0])
#    idx_to_kill = idx[booly]
#    for i in idx_to_kill[::-1]:
#        del(bpy.app.handlers.frame_change_post[i])

    # clean dead versions of the timer
    if bpy.app.timers.is_registered(live_update):
        bpy.app.timers.unregister(live_update)

    if skip:
        return
    bpy.app.timers.register(live_update, persistent=True)
#    if live:    
    
#    if animated:            
#        bpy.app.handlers.frame_change_post.append(anim_update)


def install_handler_animated(skip=False):
    """Adds a live or animated updater"""
    
#    print("running install hanlder")
#    print("handler live", live)
#    print("handler animated", animated)
#    
    # clean dead versions of the animated handler
    handler_names = np.array([i.__name__ for i in bpy.app.handlers.frame_change_post])
    booly = [i == 'anim_update' for i in handler_names]
    idx = np.arange(handler_names.shape[0])
    idx_to_kill = idx[booly]
    for i in idx_to_kill[::-1]:
        del(bpy.app.handlers.frame_change_post[i])
    
    if skip:
        return
    
#    # clean dead versions of the timer
#    if bpy.app.timers.is_registered(live_update):
#        bpy.app.timers.unregister(live_update)

#    bpy.app.timers.register(live_update, persistent=True)
#    if live:    
    
#    if animated:            
    bpy.app.handlers.frame_change_post.append(anim_update)
#    


def mass_and_force(ph):
    
    rig = ph.physics_rig
    
    ph.eidx_rel = ph.eidx[ph.relative]
    ph.eidx_rep = ph.eidx[ph.repeater]
    
    ang_force = np.array([b.CP_props.angle_force for b in rig.pose.bones], dtype=np.float32)
    ang_force_rel = ang_force[ph.relative]
    ang_force_rep = ang_force[ph.repeater]
    
    ang_force_multipier = ang_force_rel * ang_force_rep
    
    for i in range(ph.repeater.shape[0]):
        rep = ph.repeater[i]
        rel = ph.relative[i]
        if rel > rep:
            ang_force_multipier[i] = ang_force_rel[i]
    
    ph.ang_force_multipier = np.repeat(ang_force_multipier, 2)[:, None]
    
    mass = np.array([b.CP_props.head_mass for b in rig.pose.bones])
    ph.mass_rel = mass[ph.relative]
    ph.mass_rep = mass[ph.repeater]

    mass_div = ph.mass_rep / ph.mass_rel

    max = np.max(mass_div)
    reduce = 2 / max
    
    mass_div[mass_div > 2.0] = 2.0

    this = ph.mass_rel + ph.mass_rep
        
    ph.mass_ratio = ph.mass_rel / ph.mass_rep
    ph.mass_ratio /= this    
    
    ph.mass_multiplier = np.repeat(mass_div, 2)[:, None]
        
    y_difs = ph.target_co_mix[ph.eidx[:, 1]] - ph.target_co_mix[ph.eidx[:, 0]]
    ph.y_dists = U.measure_vecs(y_difs)
    ph.y_dists_rel = np.repeat(ph.y_dists[ph.relative], 2)[:, None]
    
    #test = ph.physics_rig.CP_props.test_prop
    test = 0.49
        
    uni, inverse, counts = np.unique(ph.eidx_rel, return_inverse=True, return_counts=True)
    ph.eidx_rel_counts = (1 / counts[inverse])[:, None] * test
    ph.eidx_rel_counts *= ph.y_dists_rel # so it scales to the y dist avoiding real world size issues
    ph.eidx_rel_counts *= ph.ang_force_multipier
    ph.eidx_rel_counts *= ph.mass_multiplier    
    ph.eidx_rel_counts *= test 
    
    # X (not formerly Twitter)    
    ph.x_eidx_rel = ph.mx_eidx[ph.relative]
    ph.x_eidx_rep = ph.mx_eidx[ph.repeater]

    x_uni, x_inverse, x_counts = np.unique(ph.eidx_rel, return_inverse=True, return_counts=True)
    ph.x_eidx_rel_counts = 1 / x_counts[x_inverse][:, None] * test
    ph.x_eidx_rel_counts *= ph.ang_force_multipier
    ph.x_eidx_rel_counts *= ph.mass_multiplier
    ph.x_eidx_rel_counts *= test



def new_solver_data(ph):
    self.target_m3 = A.fast_m3(self.pose_target)
    self.physics_m3 = A.fast_m3(self.physics_rig)


class physics():
    def __init__(self, physics_rig, pose_target):
        

        DATA[physics_rig.CP_props.data_key] = self
        
        self.phys_m4 = A.just_m4(physics_rig)
        self.pose_m4 = A.just_m4(pose_target)
        
        self.data_key = physics_rig.CP_props.data_key
        
        self.pose_target = pose_target.id_data
        self.physics_rig = physics_rig.id_data
        bc = len(physics_rig.pose.bones)
        tbc = len(pose_target.pose.bones)
        self.bc = bc
        self.tbc = tbc
        
        for b in physics_rig.pose.bones:
            b.bone.use_inherit_rotation = False    
            b.bone.inherit_scale = "NONE"

        self.target_m3 = A.fast_m3(self.pose_target)
        self.physics_m3 = A.fast_m3(self.physics_rig)
        
        self.target_m4 = np.empty((bc, 4, 4), dtype=np.float32)
        self.physics_m4 = A.fast_m4(self.physics_rig)
        
        # mass
        self.head_mass = np.array([b.CP_props.head_mass for b in self.physics_rig.pose.bones], dtype=np.float32)
        #self.tail_mass = np.array([b.CP_props.tail_mass for b in self.physics_rig.pose.bones], dtype=np.float32)        

        relative, repeater = A.get_relative_bones(pose_target)
        if len(relative) == 0:
            relative = np.zeros(1, dtype=np.int32)
            repeater = np.zeros(1, dtype=np.int32)
        self.relative = np.array(relative, dtype=np.int32)
        self.repeater = np.array(repeater, dtype=np.int32)            
        self.physics_rig = physics_rig
        self.pose_target = pose_target
        
        # mix    
        #mix_mesh, edges, mx_eidx, x_start_eidx, mix_vidx, x_vidx, y_vidx = A.make_mix_mesh(physics_rig, self)
        mix_mesh, edges, mx_eidx, x_start_eidx, mix_vidx, x_vidx, y_vidx = A.make_mix_mesh(pose_target, self)
        mix_mesh.CP_props.data_key = self.data_key
        self.edges = edges
        self.mix_mesh = mix_mesh
        self.mx_eidx = mx_eidx
        self.mx_eidx_relative = self.mx_eidx[self.relative]
        self.mx_eidx_repeat = self.mx_eidx[self.repeater]

        x_uni, x_inverse, x_counts = np.unique(self.mx_eidx_relative, return_inverse=True, return_counts=True)
        self.mx_eidx_rel_counts = 1 / x_counts[x_inverse][:, None]
        self.x_start_eidx = x_start_eidx
        self.x_vidx = x_vidx
        self.y_vidx = y_vidx
        self.eidx = np.array(edges, dtype=np.int32)
        self.mix_vidx = mix_vidx
        self.yvc = len(mix_vidx)
        self.mocap_co = np.empty((self.yvc, 3), dtype=np.float32)
        
        self.v_count = len(mix_mesh.data.vertices)
        self.x_bool = np.zeros(self.v_count, dtype=bool)
        self.x_bool[self.x_vidx] = True
        self.y_bool = np.zeros(self.v_count, dtype=bool)
        self.y_bool[self.y_vidx] = True
        self.yvc = self.y_vidx.shape[0]        
        self.mesh_bone_idx = self.eidx[:, 0]
        self.mesh_bone_tail_idx = self.eidx[:, 1]        

        # used for individual bone rotation force
        angle_multiplier = np.zeros(self.yvc, dtype=np.float32)
        bone_angle_forces = np.array([b.CP_props.angle_force for b in physics_rig.pose.bones], dtype=np.float32)
        self.angle_multiplier = bone_angle_forces[:, None] ** 2
        # --------------------------------------------
    
        # --------------------------------------------
        vc_mix = len(mix_mesh.data.vertices)
        self.vc_mix = vc_mix
        self.vidx_mix = np.arange(vc_mix)
        U.manage_shapes(mix_mesh, shapes=["Basis", "Target", "Current"])
        mix_mesh.data.shape_keys.key_blocks["Current"].value = 1.0
        if len(mix_mesh.data.shape_keys.key_blocks) == 3: # in case there are saved keys
            mix_mesh.active_shape_key_index = 2
        
        # mix coordinates:
        self.target_co_mix = np.empty((vc_mix, 3), dtype=np.float32)
        U.get_shape_co_mode(ob=mix_mesh, co=self.target_co_mix, key='Target')
        self.current_co = np.empty((vc_mix, 3), dtype=np.float32)
        U.get_shape_co_mode(ob=mix_mesh, co=self.current_co, key='Current')
        #self.previous_co = np.copy(self.current_co) # for dynamic collisions
        
        self.mix_obm = U.get_bmesh(mix_mesh)
        self.pinned = np.array([v.select for v in self.mix_obm.verts], dtype=bool)
        self.collided = np.zeros(vc_mix, dtype=bool)
        self.stretch_idx_mix, self.stretch_repeat_mix, self.stretch_counts_mix = get_stretch_springs_mix(self)

        # angular solve data
        
        # angle forces
        mass_and_force(self)        
        yl_data(self)
        
        y_difs = self.target_co_mix[self.eidx[:, 1]] - self.target_co_mix[self.eidx[:, 0]]
        self.y_dists = U.measure_vecs(y_difs)
        self.y_dists_rel = np.repeat(self.y_dists[self.relative], 2)[:, None]
        
        
        eidx_rel = self.eidx[self.relative]
        uni, inverse, counts = np.unique(eidx_rel, return_inverse=True, return_counts=True)
        #self.eidx_rel_counts = (1 / counts[inverse][:, None] * self.y_dists_rel)
        self.eidx_rel = eidx_rel
        
        # mix
        self.stretch_dist_mix = np.empty(self.stretch_repeat_mix.shape[0], dtype=np.float32)
        self.u_dif_mix = np.empty((self.stretch_repeat_mix.shape[0], 3), dtype=np.float32)
        self.stretch_dif_mix = np.empty((self.stretch_repeat_mix.shape[0], 3), dtype=np.float32)

        # linear mix
        self.stretch_co_mix = self.current_co[self.stretch_idx_mix]
        self.repeat_co_mix = self.current_co[self.stretch_repeat_mix]
        dif_mix = np.subtract(self.target_co_mix[self.stretch_idx_mix], self.target_co_mix[self.stretch_repeat_mix], out=self.stretch_dif_mix)
        self.target_dist_mix = np.sqrt(np.einsum('ij,ij->i', dif_mix, dif_mix, out=self.stretch_dist_mix))[:, None]
        
        self.velocity = np.zeros((vc_mix, 3), dtype=np.float32)
        self.vel_start = np.empty((vc_mix, 3), dtype=np.float32)
        self.vel_start[:] = self.current_co
        
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
        
        # setup edges:
        self.mix_eidx = np.array(self.mix_mesh.data.edge_keys, dtype=np.int32)
        vecs = self.target_co_mix[self.mix_eidx[:, 1]] - self.target_co_mix[self.mix_eidx[:, 0]]
        self.target_edge_lengths = np.sqrt(np.einsum('ij,ij->i', vecs, vecs))
        # -----------
        
        self.relative_edge_lengths = self.target_edge_lengths[self.relative][:, None]
        self.relative_y_edge_lengths_half = self.relative_edge_lengths * 0.5
        self.relative_y_edge_lengths = self.target_edge_lengths[self.relative][:, None]
        self.plot_edge_container = np.empty((self.relative.shape[0] * 2, 3), dtype=np.float32)
        #self.flat_relative_eidx = self.eidx[self.relative].ravel()
        
        self.y_edge_lengths_half = self.target_edge_lengths[:, None] * 0.5
        self.y_edges = np.empty((self.eidx.shape[0] * 2, 3), dtype=np.float32)
        self.x_edges = np.empty((self.eidx.shape[0] * 2, 3), dtype=np.float32)
        self.relative_rot_edges = np.empty_like(self.current_co[self.eidx[self.relative]])
        self.rel_rot_shape = self.relative_rot_edges.shape
        self.relative_edge_mids = np.empty_like(self.current_co[self.relative])
        self.pivots = np.empty((self.eidx.shape[0], 3), dtype=np.float32)

        # collision objects
        self.pre_tidx = np.empty(0, dtype=np.int32)
        self.tco_idx = np.empty(0, dtype=np.int32)
        self.eco_idx = np.empty(0, dtype=np.int32)
        self.edge_origins = np.empty((0, 3), dtype=np.float32)
        self.tri_origins = np.empty((0, 3), dtype=np.float32)
        self.edge_normals = np.empty((0, 3), dtype=np.float32)
        self.tri_normals = np.empty((0, 3), dtype=np.float32)
        self.t_node_sizes = np.empty(0, dtype=np.float32)
        self.e_node_sizes = np.empty(0, dtype=np.float32)
        self.friction = np.empty((0, 1), dtype=np.float32)
        
        self.hit_points = np.zeros(self.current_co.shape[0], dtype=bool)

        self.hit_idx = []
        self.hit_eidx = np.zeros(0, dtype=bool)
        update_colliders(self)
        self.nc = U.refresh_collision_objects(self)
        self.skip_refresh = True
        update_mocap_influence(self)
        # -----------------
        
        ### ===== TESTING ===== ###
        self.start_tm3 = A.get_ar_matrix_world(self.pose_target, m3=True)[self.relative]
        # influence
        influence_check(self)
        #self.inf_targets = False

        U.vecs_to_matrix(self)




# ============ NOTES ============ #
# ------- modules -------
# __init__.py
# ui.py
# character_engine.py
# utils.py
# armatures.py
# node_collide.py
# -----------------------

# ============ OPTIMIZING ============ #
'''
1. Everywhere we are doing numpy stuff we can have memory
    allocation and use out=my_array.

2. Can move core engine stuff to dll recoding in c++

'''

# ============ BUGS ============ #
'''
1. Blender bugs limits bone animation properties.
    Might need to move some bone properties out
    of the bone property groups.
    
2. Current angular solve has drift when there is x axis
    and y axis rotation.

3. when you toggle collider on and off when there
    is a physics armature set up, if the mesh
    is in edit mode the proxy gets applied.

'''

# ============ FEATURES ============ #
'''
1. Make a tool for automatically setting up IK targets maybe?
    For example the feet on a spider.
    Possibly use vertex parents for the physics mesh?

2. Decide if we should keep display of the physics mesh.
    Only things a user might do with it is make reset targets shapes
    or drag vertices around. Dragging coyuld be done with influence
    target.
    Move shapes into the ui with an enum property where people
    could name reset shapes and switch to them.

3. Currently no support for scaling animation. Simply update
    the linear scale in real time. (update angular solver scale also)
    
4. Add visualization for collision nodes.

5. Improved collisions: 
    1. Tapered cylinder primitives around bones.
    2. Better integration of mocap collisions with friction.
    3. Fix flipping issue on large rotated planes.
    4. Collision layers so two characters can interact without
        running into their own collider mesh.
    
6. intellegence for walking creatures to avoid each other.
    Something like the steering influence does raycasts and
    slightly alters the path of whatever is walking.
    Imagine a target path like a sidewalk or whatevs.
    People might avoid each other but generally stick
    to the sidewalk.
    
8. Add support for disconnected bones with offsets.

10. Add option to automatically set mass at each bone
     when a physics mesh is in place using vertex weights
     and volume of mesh in the area of each bone.

11. Wind and other effector forces.

12. Investigate single click walk cycle for legged things.
    For example, select the feet then make a rig, then add
    the walk motion and steering controls on a matching generated
    pose target rig?
    
13. Investigate using stretch-to constrants for each bone
    so the tails would exactly match the mix mesh instead
    of disconnecting parents.
    
14. Add gravity and velocity control for of each bone.

16. angular force needs to max out at 1 or have a soft max of 1

17. physics pose tools. Run the physics to pose a rig. More simply a button that
    updates the physics rig pose to the target rig pose.
    
18. linear solve per bone for example for squishy tires. This can also
    be affected by collision refresh between linear solves.
    
19. the visualisation empties at each node could serve as controls
    for head/tail size as well as the angular force at that hinge.
    
20. Set soft min on collision size to prevent negative.

21. Set soft max on angular force to 1.2 or whatever.

22. SNAP INFLUNCE TARGET.

23. A POSE TARGET BUTTON INSTEAD OF DUPLICATION OF RIG.

24. A no stretch feature using something like redistribute but
    that measures along each edge until the distance is right.
    (linear solve seems fast enough this might not be needed)

25. Need a way to make a zero influence base bone. So the head of this bone
    follows the tail with linear influence and angular but it is immune to gravity.
    Would be nice if it didn't influence the rotation of other bones. (maybe skip this if we are supporting offset bones?)

26. Need an influence target that can serve as a base bone so I don't have to
    worry about head and tail but I can connect several bones together.
    It needs to be influenced by the physics of the rig. Should also
    be able to do angular solve on this and use the matrix to plot
    connected bones, basically it's a bone with only a tail... 
    This way the head is not putting angle force on the other bones.
    Would also be nice if it supported offset bones.



CHECK OUT BLEND RIG

'''

if False:
    print("Make the physics mesh a child of the physics rig")
    print("Story Brand and Marketing Made Simple. Books reccomended by Nat")
    # task list:
    print("for no stretch try 'redistribute points'")
    
    print("make a modal operator that is a steering rig with springs\
    that will return the empties to their start location with parent\
    when you stop pressing the modal operator keys. WASD or whateves")

    print("When doing physics setup the physics mesh needs to be\
    generated in a rest state then moved to match the physics rig\
    matrix")

    print("recording could probably be done in a matrix file. Need\
    to make this for brian anyway.")

    print("could make a property for target rig. If this can be animated (might require python)\
    multiple target rigs could be switched for different actions then applied to recording\
    Might even be able to mix the target rigs or blend them.")

    print("auto ik setup would place empties at terminal tails with\
    chain length going back to where there is a split\
    these empties could also represent node sizes\
    these empties could also be set as pole targets.")
    
