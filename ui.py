import bpy
import os
import sys
import numpy as np
import importlib
from bpy.app.handlers import persistent

try:
    from character_physics import utils as U
    from character_physics import armatures as A
    from character_physics import character_engine as E
    importlib.reload(U)
    importlib.reload(A)
    importlib.reload(E)
    
except:
    U = bpy.data.texts['utils.py'].as_module()
    A = bpy.data.texts['armatures.py'].as_module()
    E = bpy.data.texts['character_engine.py'].as_module()

# task list:
if False:    
    print("make a modal operator that is a steering rig with springs\
    that will return the empties to their start location with parent\
    when you stop pressing the modal operator keys. WASD or whateves")

    print("When doing physics setup the physics mesh needs to be\
    generated in a rest state then moved to match the physics rig\
    matrix")

    print("influence check needs to be moved to a prop callback when\
    setting influence or creating or setting a new target")

    print("recording could probably be done in a matrix file. Need\
    to make this for brian anyway.")

    print("could make a property for target rig. If this can be animated (might require python)\
    multiple target rigs could be switched for different actions then applied to recording\
    Might even be able to mix the target rigs or blend them.")

    print("could make object friction based on vertex groups so\
    there can be slick spots")

    print("auto ik setup would place empties at terminal tails with\
    chain length going back to where there is a split\
    these empties could also represent node sizes\
    these empties could also be set as pole targets.")

    print("make the target rig an object property of the physics rig\
    and change the setup so you only need to select the physics rig")


#=======================#
# CALLBACK -------------#
#=======================#
def cb_test_prop(self, context):
    prop = self.test
    print("cb_test prop value", prop)


def cb_handler_update(self, context):
    ob = bpy.context.object
    live = ob.Ce_props.live
    animated = ob.Ce_props.animated
    E.install_handler(live=live, animated=animated)


def cb_update_influence(self, context):
    ob = bpy.context.object
    key = ob.Ce_props.data_key
    if ob.type == "EMPTY":    
        key = bpy.context.object.Ce_props.target_object.Ce_props.data_key
    ph = E.DATA[key]
    E.influence_check(ph)


def cb_update_head_mass(self, context):
    mix_mesh, target_rig, physics_rig, ob = get_relatives(bpy.context.object)
    
    head_mass = self.head_mass
    tail_mass = self.tail_mass

    if target_rig:
        for b in target_rig.pose.bones:
            if b.bone.select:
                b.Ce_props["head_mass"] = head_mass

    if physics_rig:
        for b in physics_rig.pose.bones:
            if b.bone.select:
                b.Ce_props["head_mass"] = head_mass

    if ob == target_rig:
        if physics_rig:
            for i in range(len(ob.pose.bones)):
                physics_rig.pose.bones[i].Ce_props["head_mass"] = ob.pose.bones[i].Ce_props["head_mass"]

    if ob == physics_rig:
        if target_rig:
            for i in range(len(ob.pose.bones)):
                target_rig.pose.bones[i].Ce_props["head_mass"] = ob.pose.bones[i].Ce_props["head_mass"]

    key = physics_rig.Ce_props.data_key
    if key not in E.DATA:
        return
    if physics_rig != E.DATA[key].physics_rig:
        return
    
    ph = E.DATA[key]
    E.distribute_mass(ph)


def cb_update_tail_mass(self, context):
    mix_mesh, target_rig, physics_rig, ob = get_relatives(bpy.context.object)
    
    head_mass = self.head_mass
    tail_mass = self.tail_mass

    if target_rig:
        for b in target_rig.pose.bones:
            if b.bone.select:
                b.Ce_props["tail_mass"] = tail_mass

    if physics_rig:
        for b in physics_rig.pose.bones:
            if b.bone.select:
                b.Ce_props["tail_mass"] = tail_mass    

    if ob == target_rig:
        if physics_rig:
            for i in range(len(ob.pose.bones)):
                physics_rig.pose.bones[i].Ce_props["tail_mass"] = ob.pose.bones[i].Ce_props["tail_mass"]

    if ob == physics_rig:
        if target_rig:
            for i in range(len(ob.pose.bones)):
                target_rig.pose.bones[i].Ce_props["tail_mass"] = ob.pose.bones[i].Ce_props["tail_mass"]
    
    key = physics_rig.Ce_props.data_key
    if key not in E.DATA:
        return
    if physics_rig != E.DATA[key].physics_rig:
        return


def get_relatives(ob):
    mix_mesh = None
    target_rig = None
    physics_rig = None
    
    if "mix_mesh" in ob:
        mix_mesh = ob["mix_mesh"]
    if "target_rig" in ob:
        target_rig = ob["target_rig"]    
    if "physics_rig" in ob:
        physics_rig = ob["physics_rig"]
    
    if mix_mesh is None:
        mix_mesh = ob
    if target_rig is None:
        target_rig = ob
    if physics_rig is None:
        physics_rig = ob    
    return mix_mesh, target_rig, physics_rig, ob
    

def reset_mass(ob):
    
    mix_mesh, target_rig, physics_rig, ob = get_relatives(ob)
    if target_rig:
        ar = target_rig
        for b in ar.pose.bones:
            b.Ce_props.head_mass = 1.0
            b.Ce_props.tail_mass = 1.0

    if physics_rig:
        ar = physics_rig
        for b in ar.pose.bones:
            b.Ce_props.head_mass = 1.0
            b.Ce_props.tail_mass = 1.0


def cb_bone_friction(self, context):
    physics_rig = bpy.context.object
    key = physics_rig.Ce_props.data_key
    if key not in E.DATA:
        return
    
    ph = E.DATA[key]
    if physics_rig != ph.physics_rig:
        return
    E.update_node_friction(ph)


def cb_update_collider(self, context):
    
    ob_list = []
    for ob in bpy.data.objects:
        if ob.name in bpy.context.scene.objects:
            if ob.type == "MESH":
                if ob.Ce_props.collider:
                    ob_list += [ob.id_data]
                    U.get_vertex_weights(ob, "Ce_friction")
            
    bpy.context.scene['Ce_collider_objects'] = ob_list

    dead = []
    for k, ph in E.DATA.items():
        try:            
            ph.physics_rig.name
        except ReferenceError:
            candidates = [ob for ob in bpy.data.objects if ob.Ce_props.data_key == ph.data_key]
            if len(candidates) == 2:
                ph.physics_rig = [ob for ob in candidates if ob.type == "ARMATURE"][0]
                ph.mix_mesh = [ob for ob in candidates if ob.type == "MESH"][0]
                ph.pose_target = ph.physics_rig.Ce_props.pose_target
                E.refresh_collision_objects(ph)
            else:
                dead += [k]

    for dk in dead:
        del E.DATA[dk]

    

def cb_update_node_size(self, context):
    for k, ph in E.DATA.items():
        E.refresh_collision_objects(ph)


#=======================#
# PROPERTIES -----------#
#=======================#
class CePropsPoseBone(bpy.types.PropertyGroup):

    head_node_size: bpy.props.FloatProperty(
        name="Head Node Size",
        description="Treat this like a sphere with a radius of this size",
        update=cb_update_node_size,
        default=0.3,
    )    

    tail_node_size: bpy.props.FloatProperty(
        name="Tail Node Size",
        description="Treat this like a sphere with a radius of this size",
        update=cb_update_node_size,
        default=0.3,
    )    

    target: bpy.props.StringProperty(
        name="target",
        description="Target This Guy",
        default="Skeletor",
    )

    target_object: bpy.props.PointerProperty(
        name="Target Object",
        type=bpy.types.Object,
        description="Select the target object",
        update=cb_update_influence,
    )

    tail_target_object: bpy.props.PointerProperty(
        name="Tail Target Object",
        type=bpy.types.Object,
        description="Select the tail target object",
        update=cb_update_influence,
    )
    
    # mass -------------
    head_mass: bpy.props.FloatProperty(
        name="Mass at head",
        description="Simulate the mass of the mesh at bone head",
        default=1.0,
        min=0.0,
        update=cb_update_head_mass,
    )    

    tail_mass: bpy.props.FloatProperty(
        name="Mass at tail",
        description="simulate the mass of the mesh at bone tail",
        default=1.0,
        min=0.0,
        update=cb_update_tail_mass,
    )

    # friction -------------
    head_friction: bpy.props.FloatProperty(
        name="Friction at head",
        description="Simulate the friction of the mesh at bone head",
        min=0.0,
        max=1.0,
        default=1.0,
        update=cb_bone_friction,
    )    

    tail_friction: bpy.props.FloatProperty(
        name="Friction at tail",
        description="simulate the friction of the mesh at bone tail",
        min=0.0,
        max=1.0,
        default=1.0,
        update=cb_bone_friction,
    )


def cb_node_size(self, context):
    rig = bpy.context.object
    for b in rig.pose.bones:
        b.Ce_props['head_node_size'] = rig.Ce_props.node_size
        b.Ce_props['tail_node_size'] = rig.Ce_props.node_size


def cb_object_friction(self, context):
    ob = bpy.context.object
    friction = ob.Ce_props.object_friction
    if ob.type == "MESH":
        v_list = np.arange(len(ob.data.vertices)).tolist()
        #U.assign_vert_group(ob, v_list, "Ce_friction", weight=friction)

        for k, ph in E.DATA.items():
            E.update_colliders(ph)
            E.refresh_collision_objects(ph)

    if ob.type == "ARMATURE":
        for b in ob.pose.bones:
            b.Ce_props['head_friction'] = friction
            b.Ce_props['tail_friction'] = friction
    
        key = bpy.context.object.Ce_props.data_key
        if key not in E.DATA:
            return
        if bpy.context.object != E.DATA[key].physics_rig:
            return

        ph = E.DATA[key]
        E.update_node_friction(ph)


def cb_update_pose_target(self, context):
    # come back here

    if self.pose_target is None:
        print("exited because target object was none")
        return

    key = self.data_key
    if key in E.DATA:
        if E.DATA[key].physics_rig != bpy.context.object:
            del E.DATA[key]

    bpy.ops.scene.ce_rig_setup()
    

test_array = np.zeros(12)
shape = (4,3)
test_array.shape = (4,3)
test_string = U.numpy_array_to_string(test_array)


# object properties -----------
class CePropsObject(bpy.types.PropertyGroup):

    record_to_keyframes: bpy.props.BoolProperty(
        name="Record",
        description="Record To Keyframes",
        default=False,
    )

    ticker: bpy.props.IntProperty(
        name="Frame Ticker",
        description="Keeps track for skip frames",
        default=0,
    )

    record_frame: bpy.props.IntProperty(
        name="Record Frame",
        description="Start or resume recording at this frame",
        default=1,
    )

    skip_frames: bpy.props.IntProperty(
        name="Skip Frames",
        description="Record Every Nth Frame",
        default=1,
    )

    target_object: bpy.props.PointerProperty(
        name="Target Object",
        type=bpy.types.Object,
        description="Select the target object",
    )
    
    velocity_array: bpy.props.StringProperty(
        name="Velocity Array",
        description="Store the velocity as a string",
        default=test_string,
    )

    current_co_array: bpy.props.StringProperty(
        name="Current Co Array",
        description="Store the current_co as a string",
        default=test_string,
    )

    vel_start_array: bpy.props.StringProperty(
        name="Vel Start Array",
        description="Store the vel_start as a string",
        default=test_string,
    )

    data_key: bpy.props.IntProperty(
        name="Data Key",
        description="Key for storing and retrieving data",
        default=0,
    )
    
    pose_target: bpy.props.PointerProperty(
        name="Pose Target",
        type=bpy.types.Object,
        description="Target the pose of this matching armature.",
        update=cb_update_pose_target,
    )

    node_size: bpy.props.FloatProperty(
        name="Node Size",
        description="Sphere size around nodes",
        default=0.3,
        update=cb_node_size
    )

    object_friction: bpy.props.FloatProperty(
        name="Friction",
        description="Friction on the surface of this object",
        default=1.0,
        min=0.0,
        max=1.0,
        update=cb_object_friction,
    )

    gravity: bpy.props.FloatProperty(
        name="gravity",
        description="The effect of gravity on this object",
        default=-1.0,
    )

    velocity: bpy.props.FloatProperty(
        name="velocity",
        description="velocity is multiplied by this value",
        default=0.98,
    )

    angular_force: bpy.props.FloatProperty(
        name="Angular Force",
        description="global angular strength",
        default=3.0,
    )

    animated: bpy.props.BoolProperty(
        name="Animated",
        description="Runs physics when animation is running",
        default=True,
    )

    live: bpy.props.BoolProperty(
        name="Live",
        description="Runs physics all the time even during the zombie apocalypse",
        default=False,
    )

    # iters ---------------
    linear_iters: bpy.props.IntProperty(
        name="Linear Iters",
        description="number of linear solves before angular",
        default=4,
    )

    # ITERS
    angular_iters: bpy.props.IntProperty(
        name="Angular Iters",
        description="number of angular solves",
        default=4,
    )

    # ITERS
    linear_post_iters: bpy.props.IntProperty(
        name="Linear Post Iters",
        description="number of linear solves after angular",
        default=4,
    )

    # ITERS
    sub_frames: bpy.props.IntProperty(
        name="Sub Frame",
        description="number all solves between frames",
        default=4,
    )

    # INFLUENCE ---------------
    influence_co_linear: bpy.props.FloatProperty(
        name="Inf co linear",
        description="number all solves between frames",
        default=0.0,
        precision=6,        
    )

    # INFLUENCE ---------------
    influence_spring_linear: bpy.props.FloatProperty(
        name="Inf spring linear",
        description="number all solves between frames",
        default=0.0,
        precision=6,
    )

    # INFLUENCE ---------------
    influence_co_angular: bpy.props.FloatProperty(
        name="Inf co_lin",
        description="number all solves between frames",
        default=0.0,
        precision=6,        
    )

    # INFLUENCE ---------------
    influence_spring_angular: bpy.props.FloatProperty(
        name="Inf co_lin",
        description="number all solves between frames",
        default=0.0,
        precision=6,        
    )
    
    # INFLUENCE ---------------
    influence_head_idx: bpy.props.IntProperty(
        name="Inf head idx",
        description="Mesh index for head at this target",
        default=-1,
    )

    # INFLUENCE ---------------
    influence_tail_idx: bpy.props.IntProperty(
        name="Inf tail idx",
        description="Mesh index for tail at this target",
        default=-1,
    )
    
    # INFLUENCE ---------------
    influence_is_tail: bpy.props.BoolProperty(
        name="Inf Is Tail",
        description="Tail otherwise head",
        default=False,
    )

    # INFLUENCE ---------------
    influence_is_target: bpy.props.BoolProperty(
        name="Is Target",
        description="Empties that are influence targets",
        default=False,
    )
    
    # INFLUENCE ---------------
    influence_spring: bpy.props.FloatProperty(
        name="Inf Spring",
        description="Spring To target",
        default=0.0,
        precision=6,
        soft_min=0.0,
        soft_max=1.0,
        update=cb_update_influence,
    )

    # INFLUENCE ---------------
    influence_directional: bpy.props.FloatProperty(
        name="Inf Spring",
        description="Move in target direction",
        default=0.0,
        precision=6,
        update=cb_update_influence,
    )

    # COLLISION ---------------
    collider: bpy.props.BoolProperty(
        name="Collider",
        description="Check collisions on this object",
        default=False,
        update=cb_update_collider,
    )
    

# scene properties ----------
class CePropsScene(bpy.types.PropertyGroup):
    test: bpy.props.BoolProperty(
        name="Test scene prop",
        description="Is this working?",
        default=False,
        update=cb_test_prop,
    )

    skip_handler: bpy.props.BoolProperty(
        name="Skip Handler",
        description="Skip the frame handler for recording",
        default=False,
    )




#=======================#
# OPERATORS ------------#
#=======================#
def set_rest_mode(rig):
    name = rig.data.name
    mode = bpy.data.armatures[name].pose_position
    if mode == "REST":
        return mode, name
    bpy.data.armatures[name].pose_position = "REST"
    bpy.context.view_layer.update()
    return mode, name


def rig_setup(rig=None):
    
    if rig is None:    
        rig = bpy.context.object
    if rig.type != "ARMATURE":
        msg = "Selct an armature"
        bpy.context.window_manager.popup_menu(U.oops, title=msg, icon='ARMATURE_DATA')
        return
    
    we_good = False
    pose_target = rig.Ce_props.pose_target
    if pose_target:
        if pose_target.type == "ARMATURE":
            target_rig = pose_target 
            we_good = True
        if not we_good:
            msg = "Pose target needs to be a matching armature."
            bpy.context.window_manager.popup_menu(U.oops, title=msg, icon='ARMATURE_DATA')
            rig.Ce_props['pose_target'] = None
            return
    else:
        return
    
    key = bpy.context.object.Ce_props.data_key
    
    running = False
    #if False:    
    if key in E.DATA:
        ph1 = E.DATA[key]
        if ph1.physics_rig == rig:
            #current = np.copy(ph1.current_co)    
            #velocity = np.copy(ph1.velocity)
            ph1.pose_target = rig.Ce_props.pose_target
            return

    rig_mode, rig_name = set_rest_mode(pose_target)
    p_rig_mode, p_rig_name = set_rest_mode(rig)
        
    ph = E.physics(rig, target_rig)
    
    M = rig.matrix_world
    ph.mix_mesh.matrix_world = M
    
    live = rig.Ce_props.live
    animated = rig.Ce_props.animated
    E.install_handler(live=live, animated=animated)

    rig["mix_mesh"] = ph.mix_mesh.id_data
    rig["target_rig"] = target_rig.id_data
    target_rig["mix_mesh"] = ph.mix_mesh.id_data
    target_rig["physics_rig"] = rig.id_data
    ph.mix_mesh.id_data["target_rig"] = target_rig.id_data
    ph.mix_mesh.id_data["physics_rig"] = rig.id_data
    
    bpy.data.armatures[rig_name].pose_position = rig_mode
    bpy.data.armatures[p_rig_name].pose_position = p_rig_mode
        
    cb_update_collider(None, None)
    ### ----- UPDATE RIG ----- ###
    E.vecs_to_matrix(ph)
    mesh_bone_co = ph.current_co[ph.mesh_bone_idx]
    A.set_ar_m3_world(ph.physics_rig, ph.physics_m3, locations=mesh_bone_co, return_quats=False)

    return ph
    

class CeRigSetup(bpy.types.Operator):
    """Setup physics data for rig."""

    bl_idname = "scene.ce_rig_setup"
    bl_label = "Ce Rig Setup"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        rig_setup()
        return {'FINISHED'}


def reset_pose(ar):
    loc = (0.0, 0.0, 0.0)    
    rot = (1.0, 0.0, 0.0, 0.0)
    scale = (1.0, 1.0, 1.0)
    for bo in ar.pose.bones:
        bo.location = loc
        bo.rotation_quaternion = rot
        bo.scale = scale
        

def rig_reset():
    
    # just a place to put this while testing
    bpy.context.scene.Ce_props.skip_handler = False
    # --------------------------------------
    
    ob = bpy.context.object
    if ob is None:
        print("No active object")
        return
    
    mix_mesh = None
    target_rig = None
    physics_rig = None
    
    if "mix_mesh" in ob:
        mix_mesh = ob["mix_mesh"]
    if "target_rig" in ob:
        target_rig = ob["target_rig"]    
    if "physics_rig" in ob:
        physics_rig = ob["physics_rig"]
    
    if mix_mesh is None:
        mix_mesh = ob
    if target_rig is None:
        target_rig = ob
    if physics_rig is None:
        physics_rig = ob
            
    reset_pose(target_rig)
    reset_pose(physics_rig)
    
    key = physics_rig.Ce_props.data_key
    if key not in E.DATA:
        return
    
    ph = E.DATA[key]
    if physics_rig != ph.physics_rig:
        return

    if mix_mesh is not None:
        U.co_to_shape(mix_mesh, co=None, key="Current")
        U.co_to_shape(mix_mesh, co=None, key="Target")
        U.co_to_shape(mix_mesh, co=None, key="Basis")
        
        vc = len(mix_mesh.data.vertices)
        ph.current_co = np.empty((vc, 3), dtype=np.float32)
        U.get_shape_co_mode(ob=mix_mesh, co=ph.current_co, key='Current')
        ph.velocity = np.zeros((vc, 3), dtype=np.float32)
        ph.vel_start = np.empty((vc, 3), dtype=np.float32)
        ph.vel_start[:] = ph.current_co
        ph.physics_m3 = A.get_ar_matrix_world(target_rig, m3=True)
        E.refresh_collision_objects(ph)

#import bpy
#from bpy import context




def make_influence_target(type='head', make_parent=False):
    
    exit = False
    physics_rig = bpy.context.object
    key = bpy.context.object.Ce_props.data_key
    if key in E.DATA:
        ph = E.DATA[key]
        if physics_rig != ph.physics_rig:
            exit = True
    else:
        exit = True

    if exit:
        print("!!! make_influence_target didn't work !!!")
        return
        
    sel_bool = [b.bone.select for b in physics_rig.pose.bones]
    selected = [b for b in physics_rig.pose.bones if b.bone.select]
    #matricies = np.array[b.matrix for b in selected]
    bidx = np.arange(len(physics_rig.pose.bones))
    sel_idx = bidx[sel_bool]
    co_idx = ph.mesh_bone_idx[sel_idx]
    co_idx_tial = ph.mesh_bone_tail_idx[sel_idx]
    display_type = "ARROWS"
    for e, b in enumerate(selected):
        name=b.name + "_inf_tar"
        em = U.create_empty(display_type, name, b.matrix, scene=None)
        em.Ce_props.influence_is_target = True
        em.show_in_front = True
        em.Ce_props.target_object = ph.physics_rig
        
        head_idx = int(co_idx[e])
        tail_idx = int(co_idx_tial[e])

        em.Ce_props.influence_head_idx = head_idx
        em.Ce_props.influence_tail_idx = tail_idx
        
        npm = np.array(b.matrix)
        if type == 'head':
            b.Ce_props.target_object = em
            em.location = ph.physics_rig.matrix_world @ b.head
        if type == 'tail':
            b.Ce_props.tail_target_object = em
            em.location = ph.physics_rig.matrix_world @ b.tail
            em.Ce_props.influence_is_tail = True
                
        if make_parent:            
            em.parent = ph.mix_mesh
            em.parent_type = 'VERTEX'
            em.parent_vertices = [head_idx] * 3
            em.location = np.zeros(3, dtype=np.float32)
            #em.matrix_parent_inverse = ph.mix_mesh.matrix_world.inverted()


class CeRigReset(bpy.types.Operator):
    """Clear location, rotation, and scale of rig"""

    bl_idname = "scene.ce_rig_reset"
    bl_label = "Ce Rig Reset"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        rig_reset()
        return {'FINISHED'}


class CeMakeInfluenceTarget(bpy.types.Operator):
    """Set up empty for selected bones
    to serve as influence target"""

    bl_idname = "scene.ce_make_influence_target"
    bl_label = "Ce Make Influence Target"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        make_influence_target(type='head')
        return {'FINISHED'}


class CeMakeInfluenceTargetTail(bpy.types.Operator):
    """Set up empty for selected bones
    to serve as influence target"""

    bl_idname = "scene.ce_make_influence_target_tail"
    bl_label = "Ce Make Influence Target Tail"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        make_influence_target(type='tail')
        return {'FINISHED'}


class CeResetMass(bpy.types.Operator):
    """Reset head and tail mass to 1.0
    for all bones in active object."""

    bl_idname = "scene.ce_reset_mass"
    bl_label = "Ce Reset Bone Mass"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        if not bpy.context.object:
            return {'FINISHED'}
            
        if bpy.context.object.type != "ARMATURE":
            return {'FINISHED'}
        
        reset_mass(bpy.context.object)
        return {'FINISHED'}

    

#=======================#
# UI -------------------#
#=======================#
class PANEL_PT_Character_Engine(bpy.types.Panel):
    """GLTF Panel"""

    bl_label = "Character Engine Panel"
    bl_idname = "PANEL_PT_Character_Engine"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Character Engine"
    
    def draw(self, context):
        layout = self.layout
        col = layout.column(align=True)
        ob = bpy.context.object
        if ob is None:
            return
        if ob.type == "MESH":
            col.prop(bpy.context.object.Ce_props, "collider", text="Collider")
            col.prop(bpy.context.object.Ce_props, "object_friction", text="Friction")
        
        if ob.type == "ARMATURE":
            col.label(text="Recording")
            col.prop(bpy.context.object.Ce_props, "record_to_keyframes", text="Record to Keyframes")
            col.prop(bpy.context.object.Ce_props, "skip_frames", text="Every Nth Frame")
            col.prop(bpy.context.object.Ce_props, "record_frame", text="Record Frame")

            col.label(text="Setup")
            col.prop(bpy.context.object.Ce_props, "pose_target", text="Pose Target")
            col.operator("scene.ce_rig_setup", text="Physics Setup", icon='ARMATURE_DATA')
            col.operator("scene.ce_rig_reset", text="Reset", icon='FILE_REFRESH')
            col.prop(bpy.context.object.Ce_props, "animated", text="Animated")
            col.prop(bpy.context.object.Ce_props, "live", text="Live")
                    
            col.label(text="Forces")
            col.prop(bpy.context.object.Ce_props, "gravity", text="Gravity")
            col.prop(bpy.context.object.Ce_props, "velocity", text="Velocity")
            #col.prop(bpy.context.object.Ce_props, "angular_force", text="Angular Force")
                    
            col.label(text="Solve Iterations")
            col.prop(bpy.context.object.Ce_props, "linear_iters", text="Linear")
            col.prop(bpy.context.object.Ce_props, "angular_iters", text="Angular")
            col.prop(bpy.context.object.Ce_props, "linear_post_iters", text="Lin Post")
            col.prop(bpy.context.object.Ce_props, "sub_frames", text="sub frame")
        
            col.label(text="bone targets")
            bone = bpy.context.active_pose_bone
            if bone:
                col.label(text="Mass")
                col.prop(bone.Ce_props, "head_mass", text="Mass Head")
                col.prop(bone.Ce_props, "tail_mass", text="Mass Tail")
                col.operator("scene.ce_reset_mass", text="Reset Mass")
                
                col.label(text="Friction")
                col.prop(ob.Ce_props, "object_friction", text="All Bones")
                col.prop(bone.Ce_props, "head_friction", text="Friction Head")
                col.prop(bone.Ce_props, "tail_friction", text="Friction Tail")
                
                col.label(text="Node Sizes")
                col.prop(bone.Ce_props, "head_node_size", text="Head Size")
                col.prop(bone.Ce_props, "tail_node_size", text="Tail Size")                    
                
                col.label(text="Influence Targets")
                col.operator("scene.ce_make_influence_target", text="Head Target", icon='OUTLINER_DATA_ARMATURE')
                col.operator("scene.ce_make_influence_target_tail", text="Tail Target", icon='OUTLINER_DATA_ARMATURE')
                #col.prop(bone.Ce_props, "target", text="Influence Target")
                #col.prop(bpy.context.object.Ce_props, "node_size", text="Node Size")
                
                
                col.label(text="Head")
                col.prop(bone.Ce_props, "target_object", text="Influence Target")
                target = bone.Ce_props.target_object
                
                if target:
                    col.prop(target.Ce_props, "influence_directional", text="Directional")
                    col.prop(target.Ce_props, "influence_spring", text="Spring")
                
                col.label(text="Tail")
                col.prop(bone.Ce_props, "tail_target_object", text="Influence Target")
                tail_target = bone.Ce_props.tail_target_object
                if tail_target:    
                    col.prop(tail_target.Ce_props, "influence_directional", text="Directional")
                    col.prop(tail_target.Ce_props, "influence_spring", text="Spring")
            else:
                col.label(text="Friction")
                col.prop(ob.Ce_props, "object_friction", text="All Bones Friction")            
            
        col.label(text="Influence Target")
        if ob:
            if bpy.context.object.Ce_props.influence_is_target:
                col.label(text="Influence")
                col.prop(bpy.context.object.Ce_props, "influence_directional", text="Directional")
                col.prop(bpy.context.object.Ce_props, "influence_spring", text="Spring")
                                    
        
        return
        
        # for reference:
        col.operator("test.select_folder", text="Batch Folder", icon='FILE_FOLDER')
        col.prop(bpy.context.scene.Gltf_props, "folder_path", text='', icon='FILEBROWSER')

        col.label(text="Single")
        col.operator("test.open_filebrowser", text="Browse", icon='FILE_FOLDER')
        col.prop(bpy.context.scene.Gltf_props, "append_path", text='', icon='FILEBROWSER')
        col.operator("scene.gltf_append_objects", text="Append Objects")  # , icon='RIGHTARROW')
        col2 = col.column()
        row2 = col2.row()
        row2.operator("scene.gltf_append_next", icon='TRIA_UP', text='')
        row2.prop(bpy.context.scene.Gltf_props, "convert_next", text="convert")
        row3 = col2.row()
        row3.prop(bpy.context.scene.Gltf_props, "view_update", text="view update")
        row3 = col2.row()
        row3.operator("scene.gltf_append_previous", icon='TRIA_DOWN', text='')
        row3.prop(bpy.context.scene.Gltf_props, "convert_previous", text="convert")


#=======================#
# SAVE/LOAD ------------#
#=======================#
@persistent
def Ce_save_handler(scene=None):
    
    for k, ph in E.DATA.items():
        ce = ph.physics_rig.Ce_props
        ce.velocity_array = U.numpy_array_to_string(ph.velocity)
        ce.current_co_array = U.numpy_array_to_string(ph.current_co)
        ce.vel_start_array = U.numpy_array_to_string(ph.vel_start)
        

@persistent
def Ce_load_handler(scene=None):
    bpy.context.view_layer.update()

    E.DATA = {}
    for ob in bpy.context.scene.objects:
        if U.check_object(ob):    
            if ob.type == "ARMATURE":
                if ob.Ce_props.pose_target:
                    rig_setup(rig=ob)

    for k, ph in E.DATA.items():
        ce = ph.physics_rig.Ce_props
        ph.current_co = U.string_to_numpy_array(ce.current_co_array)
        ph.velocity = U.string_to_numpy_array(ce.velocity_array)
        ph.vel_start = U.string_to_numpy_array(ce.vel_start_array)

        if ph.mix_mesh.data.is_editmode:
            for i, v in enumerate(ph.mix_obm.verts):
                if not v.select:    
                    v.co = ph.current_co[i]
                        
            bmesh.update_edit_mesh(ph.mix_mesh.data)

        else: 
            U.set_shape_co(ph.mix_mesh, "Current", ph.current_co)
            ph.mix_mesh.data.update()
        
        ### ----- UPDATE RIG ----- ###
        E.vecs_to_matrix(ph)
        mesh_bone_co = ph.current_co[ph.mesh_bone_idx]
        A.set_ar_m3_world(ph.physics_rig, ph.physics_m3, locations=mesh_bone_co, return_quats=False)
                

def save_handler(exit=False):
    """Handler for saving data.
    Current states like velocity
    need to be preserved."""

    # clean dead versions of the save handler
    handler_names = np.array([i.__name__ for i in bpy.app.handlers.save_pre])
    booly = [i == 'Ce_save_handler' for i in handler_names]
    idx = np.arange(handler_names.shape[0])
    idx_to_kill = idx[booly]
    for i in idx_to_kill[::-1]:
        del(bpy.app.handlers.save_pre[i])

    if exit:
        return
    bpy.app.handlers.save_pre.append(Ce_save_handler)


def load_handler(exit=False):
    """Handler for loading data.
    Current states like velocity
    need to be preserved."""

    # clean dead versions of the animated handler
    handler_names = np.array([i.__name__ for i in bpy.app.handlers.load_post])
    booly = [i == 'Ce_load_handler' for i in handler_names]
    idx = np.arange(handler_names.shape[0])
    idx_to_kill = idx[booly]
    for i in idx_to_kill[::-1]:
        del(bpy.app.handlers.load_post[i])

    if exit:
        return
    bpy.app.handlers.load_post.append(Ce_load_handler)


#=======================#
# REGISTER -------------#
#=======================#
classes = (
    PANEL_PT_Character_Engine,
    CePropsObject,
    CePropsScene,
    CePropsPoseBone,
    CeRigSetup,
    CeRigReset,
    CeMakeInfluenceTarget,
    CeMakeInfluenceTargetTail,
    CeResetMass,
)


def register():
    # classes
    from bpy.utils import register_class

    for cls in classes:
        register_class(cls)
    bpy.types.Scene.Ce_props = bpy.props.PointerProperty(type=CePropsScene)
    bpy.types.Object.Ce_props = bpy.props.PointerProperty(type=CePropsObject)
    bpy.types.PoseBone.Ce_props = bpy.props.PointerProperty(type=CePropsPoseBone)
    
    save_handler()
    load_handler()

    E.install_handler(live=False, animated=False)
#    for ob in bpy.data.objects:
#        if "physics_mesh" in ob:
#            del(ob["physics_mesh"])
#        if "target_rig" in ob:
#            del(ob["target_rig"])
#        if "physics_rig" in ob:
#            del(ob["physics_rig"])

#    bpy.context.scene.Ce_props.skip_handler = False


def unregister():
    # classes

    msg = "I guess you don't love me anymore . Goodbye cruel world. I die!"
    bpy.context.window_manager.popup_menu(U.oops, title=msg, icon='GHOST_ENABLED')

    from bpy.utils import unregister_class

    for cls in reversed(classes):
        unregister_class(cls)
    
    save_handler(exit=True)
    load_handler(exit=True)
    

if __name__ == "__main__":
    register()
