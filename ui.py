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


#=======================#
# CALLBACK -------------#
#=======================#

# subscribe and donate --------------------------
def cb_paypal(self, context):
    bpy.context.scene.CP_props["paypal"] = False
    print(dir(U))
    U.open_browser(link="paypal")


def cb_gumroad(self, context):
    bpy.context.scene.CP_props["gumroad"] = False
    U.open_browser(link="gumroad")


def cb_patreon(self, context):
    bpy.context.scene.CP_props["patreon"] = False
    U.open_browser(link="patreon")


def cb_donate(self, context):
    bpy.context.scene.CP_props["donate"] = False
    U.open_browser(link="donate")
#-----------------------------------------------


def cb_test_prop(self, context):
    prop = self.test
    print("cb_test prop value", prop)


def cb_handler_update(self, context):
    ob = bpy.context.object
    live = ob.CP_props.live
    animated = ob.CP_props.animated
    bpy.context.scene.CP_props.skip_handler = False
    if animated:    
        E.install_handler_animated()
    if live:
        E.install_handler_live()


def cb_update_influence(self, context):
    ob = bpy.context.object
    key = ob.CP_props.data_key
    if ob.type == "EMPTY":    
        key = bpy.context.object.CP_props.target_object.CP_props.data_key
    ph = E.DATA[key]
    E.influence_check(ph)


def cb_update_rot_force(self, context):
    ob = bpy.context.object
    key = ob.CP_props.data_key
    ph = E.DATA[key]    
    
    angle_multiplier = np.zeros(ph.yvc, dtype=np.float32)
    a_uni, a_inverse, a_counts = np.unique(ph.eidx, return_inverse=True, return_counts=True)
    bone_angle_forces = np.array([b.CP_props.angle_force for b in ph.physics_rig.pose.bones], dtype=np.float32)
    aw_counts = a_counts[a_inverse]
    aw_counts.shape = (aw_counts.shape[0] // 2, 2)
    rep_angle_mult = np.repeat(bone_angle_forces, 2)
    rep_angle_mult.shape = (rep_angle_mult.shape[0] // 2, 2)
    mix_angle_mult = rep_angle_mult / aw_counts
    np.add.at(angle_multiplier, ph.eidx, mix_angle_mult)
    ph.angle_multiplier = angle_multiplier[:, None]


#def cb_update_head_mass(self, context):
#    mix_mesh, target_rig, physics_rig, ob = get_relatives(bpy.context.object)
#    
#    head_mass = self.head_mass
#    tail_mass = self.tail_mass
#    
#    if False: # seems to be a problem accessing bone props by their key...
#        if target_rig:
#            for b in target_rig.pose.bones:
#                if b.bone.select:
#                    b.CP_props["head_mass"] = head_mass

#        if physics_rig:
#            for b in physics_rig.pose.bones:
#                if b.bone.select:
#                    b.CP_props["head_mass"] = head_mass

#        if ob == target_rig:
#            if physics_rig:
#                for i in range(len(ob.pose.bones)):
#                    physics_rig.pose.bones[i].CP_props["head_mass"] = ob.pose.bones[i].CP_props["head_mass"]

#        if ob == physics_rig:
#            if target_rig:
#                for i in range(len(ob.pose.bones)):
#                    target_rig.pose.bones[i].CP_props["head_mass"] = ob.pose.bones[i].CP_props["head_mass"]

#    key = physics_rig.CP_props.data_key
#    if key not in E.DATA:
#        return
#    if physics_rig != E.DATA[key].physics_rig:
#        return
#    
#    ph = E.DATA[key]
#    E.distribute_mass(ph)


#def cb_update_tail_mass(self, context):
#    mix_mesh, target_rig, physics_rig, ob = get_relatives(bpy.context.object)
#    
#    head_mass = self.head_mass
#    tail_mass = self.tail_mass
#    
#    if False: # seems to be a problem accessing bone props by their key...
#        if target_rig:
#            for b in target_rig.pose.bones:
#                if b.bone.select:
#                    b.CP_props["tail_mass"] = tail_mass

#        if physics_rig:
#            for b in physics_rig.pose.bones:
#                if b.bone.select:
#                    b.CP_props["tail_mass"] = tail_mass    

#        if ob == target_rig:
#            if physics_rig:
#                for i in range(len(ob.pose.bones)):
#                    physics_rig.pose.bones[i].CP_props["tail_mass"] = ob.pose.bones[i].CP_props["tail_mass"]

#        if ob == physics_rig:
#            if target_rig:
#                for i in range(len(ob.pose.bones)):
#                    target_rig.pose.bones[i].CP_props["tail_mass"] = ob.pose.bones[i].CP_props["tail_mass"]
#    
#    key = physics_rig.CP_props.data_key
#    if key not in E.DATA:
#        return
#    if physics_rig != E.DATA[key].physics_rig:
#        return


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
    

def reset_mass(ob, value=1.0):
    
    mix_mesh, target_rig, physics_rig, ob = get_relatives(ob)
    if target_rig:
        ar = target_rig
        for b in ar.pose.bones:
            b.CP_props.head_mass = value
            #b.CP_props.tail_mass = value

    if physics_rig:
        ar = physics_rig
        for b in ar.pose.bones:
            b.CP_props.head_mass = value
            #b.CP_props.tail_mass = value


def reset_angular(ob):
    
    mix_mesh, target_rig, physics_rig, ob = get_relatives(ob)
    if target_rig:
        ar = target_rig
        for b in ar.pose.bones:
            b.CP_props.angle_force = 1.0

    if physics_rig:
        ar = physics_rig
        for b in ar.pose.bones:
            b.CP_props.angle_force = 1.0


def cb_bone_head_friction(self, context):
    physics_rig = bpy.context.object
    key = physics_rig.CP_props.data_key
    if key not in E.DATA:
        return
    
    ph = E.DATA[key]
    if physics_rig != ph.physics_rig:
        return

    bone = bpy.context.active_pose_bone
    prop = bone.CP_props.head_friction
    for b in physics_rig.pose.bones:
        if b.bone.select:
            b.CP_props['head_friction'] = prop

    E.update_node_friction(ph)


def cb_bone_tail_friction(self, context):
    physics_rig = bpy.context.object
    key = physics_rig.CP_props.data_key
    if key not in E.DATA:
        return
    
    ph = E.DATA[key]
    if physics_rig != ph.physics_rig:
        return

    bone = bpy.context.active_pose_bone
    prop = bone.CP_props.tail_friction
    for b in physics_rig.pose.bones:
        if b.bone.select:
            b.CP_props['tail_friction'] = prop

    E.update_node_friction(ph)


def cb_update_collider(self, context):
    
    ob_list = []
    for ob in bpy.data.objects:
        if ob.name in bpy.context.scene.objects:
            if ob.type == "MESH":
                if ob.CP_props.collider:
                    ob_list += [ob.id_data]
                    U.get_vertex_weights(ob, "CP_friction")
            
    bpy.context.scene['Ce_collider_objects'] = ob_list
    
    bad_keys = []
    
    for k, ph in E.DATA.items():
        U.validate_references(ph)
        if ph.physics_rig:
            U.refresh_collision_objects(ph)
        else:
            bad_keys += [k]

    for k in bad_keys:
        del(E.DATA[k])
                    

def cb_update_head_node_size(self, context):
    for k, ph in E.DATA.items():
        rig = ph.physics_rig
        bone = bpy.context.active_pose_bone
        prop = bone.CP_props.head_node_size
        for b in rig.pose.bones:
            if b.bone.select:
                b.CP_props['head_node_size'] = prop
        
        U.refresh_collision_objects(ph)


def cb_update_tail_node_size(self, context):
    for k, ph in E.DATA.items():
        rig = ph.physics_rig
        bone = bpy.context.active_pose_bone
        prop = bone.CP_props.tail_node_size
        for b in rig.pose.bones:
            if b.bone.select:
                b.CP_props['tail_node_size'] = prop
        
        U.refresh_collision_objects(ph)


def cb_update_mocap_head_influence(self, context):
    for k, ph in E.DATA.items():
        rig = ph.physics_rig
        bone = bpy.context.active_pose_bone
        prop = bone.CP_props.head_mocap_influence

        for b in rig.pose.bones:
            if b.bone.select:
                b.CP_props['head_mocap_influence'] = prop
        
        E.update_mocap_influence(ph)


def cb_update_mocap_tail_influence(self, context):
    for k, ph in E.DATA.items():
        rig = ph.physics_rig
        bone = bpy.context.active_pose_bone
        prop = bone.CP_props.tail_mocap_influence
        for b in rig.pose.bones:
            if b.bone.select:
                b.CP_props['tail_mocap_influence'] = prop
        
        E.update_mocap_influence(ph)
        
        
def cb_update_mocap_influence(self, context):        
    for k, ph in E.DATA.items():    
        E.update_mocap_influence(ph)


def cb_update_mass_and_force(self, context):
    
    rig = bpy.context.object
    selected = [b for b in rig.pose.bones if b.bone.select]
    
    if len(selected) > 1:
        bone = bpy.context.active_pose_bone
        value = bone.CP_props.head_mass
        for b in selected:
            b.CP_props['head_mass'] = value       
    
    for k, ph, in E.DATA.items():
        E.mass_and_force(ph)


def cb_update_angle_force(self, context):
    
    rig = bpy.context.object
    selected = [b for b in rig.pose.bones if b.bone.select]
    
    if len(selected) > 1:
        bone = bpy.context.active_pose_bone
        value = bone.CP_props.angle_force
        for b in selected:
            b.CP_props['angle_force'] = value        

    for k, ph, in E.DATA.items():
        E.mass_and_force(ph)


#def cb_update_selected_mass(self, context):

#    rig = bpy.context.object
#    selected = [b for b in rig.pose.bones if b.bone.select]
#    if len(selected) > 1:
#        bone = bpy.context.active_pose_bone
#        value = bone.CP_props.selected_bone_mass
#        bone.CP_props.skip_mass_update = True
#            
#        for b in selected:
#            b.CP_props.head_mass = value
#            b.CP_props['selected_bone_mass'] = value
#            
#        for k, ph, in E.DATA.items():
#            E.mass_and_force(ph)
#        bone.CP_props.skip_mass_update = False


#=======================#
# PROPERTIES -----------#
#=======================#
class CpPropsPoseBone(bpy.types.PropertyGroup):

    angle_force: bpy.props.FloatProperty(
        name="Angle Force",
        description="Strength of influence of angle forces on this bone.",
        #update=cb_update_rot_force,
        precision=4,
        min=0.0,
        soft_max=2,
        update=cb_update_angle_force,
        default=1.0,
    )

    parent_influence: bpy.props.FloatProperty(
        name="Parent Influence",
        description="Let parent rotate this bone",
        #update=cb_update_head_node_size,
        default=0.0,
    )

    head_node_size: bpy.props.FloatProperty(
        name="Head Node Size",
        description="Treat this like a sphere with a radius of this size",
        update=cb_update_head_node_size,
        default=0.3,
    )

    tail_node_size: bpy.props.FloatProperty(
        name="Tail Node Size",
        description="Treat this like a sphere with a radius of this size",
        update=cb_update_tail_node_size,
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
    
    rotation_target_object: bpy.props.PointerProperty(
        name="Rotation Target Object",
        type=bpy.types.Object,
        description="Select the rotation target object",
        update=cb_update_influence,
    )
    
    # mass -------------
    head_mass: bpy.props.FloatProperty(
        name="Mass at head",
        description="Simulate the mass of the mesh at bone head",
        default=1.0,
        min=0.0,
        #update=cb_update_head_mass,
        update=cb_update_mass_and_force,
    )

    head_mocap_influence: bpy.props.FloatProperty(
        name="Bone Mocap Influence",
        description="Influence of motion capture on this bone",
        default=1.0,
        soft_min=0.0,
        soft_max=1.0,
        update=cb_update_mocap_head_influence,
    )

    tail_mocap_influence: bpy.props.FloatProperty(
        name="Bone Mocap Influence",
        description="Influence of motion capture on this bone",
        default=1.0,
        soft_min=0.0,
        soft_max=1.0,
        update=cb_update_mocap_tail_influence,
    )

#    selected_bone_mass: bpy.props.FloatProperty(
#        name="Selected bone mass",
#        description="Sets the mass for all selected bones when more than one is selected",
#        default=1.0,
#        min=0.0,
#        update=cb_update_selected_mass,
#    )
    
    skip_mass_update: bpy.props.BoolProperty(
        name="Skip Mass Update",
        description="When updating selected bones we use this to override bone mass callbacks",
        default=False,
    )
    
    skip_selected_mass_update: bpy.props.BoolProperty(
        name="Skip Selected Mass Update",
        description="When updating bone mass we use this to override selected bone mass callbacks",
        default=False,
    )

#    tail_mass: bpy.props.FloatProperty(
#        name="Mass at tail",
#        description="simulate the mass of the mesh at bone tail",
#        default=1.0,
#        min=0.0,
#        update=cb_update_tail_mass,
#    )

    # friction -------------
    head_friction: bpy.props.FloatProperty(
        name="Friction at head",
        description="Simulate the friction of the mesh at bone head",
        min=0.0,
        max=1.0,
        default=1.0,
        update=cb_bone_head_friction,
    )

    tail_friction: bpy.props.FloatProperty(
        name="Friction at tail",
        description="simulate the friction of the mesh at bone tail",
        min=0.0,
        max=1.0,
        default=1.0,
        update=cb_bone_tail_friction,
    )


def cb_node_size(self, context):
    rig = bpy.context.object
    for b in rig.pose.bones:
        b.CP_props['head_node_size'] = rig.CP_props.node_size
        b.CP_props['tail_node_size'] = rig.CP_props.node_size


def cb_object_friction(self, context):
    ob = bpy.context.object
    friction = ob.CP_props.object_friction
    if ob.type == "MESH":
        v_list = np.arange(len(ob.data.vertices)).tolist()
        #U.assign_vert_group(ob, v_list, "CP_friction", weight=friction)

        for k, ph in E.DATA.items():
            E.update_colliders(ph)
            U.refresh_collision_objects(ph)

    if ob.type == "ARMATURE":
        for b in ob.pose.bones:
            b.CP_props['head_friction'] = friction
            b.CP_props['tail_friction'] = friction
    
        key = bpy.context.object.CP_props.data_key
        if key not in E.DATA:
            return
        if bpy.context.object != E.DATA[key].physics_rig:
            return

        ph = E.DATA[key]
        E.update_node_friction(ph)


def cb_update_pose_target(self, context):

    if self.pose_target is None:
        print("exited because target object was none")
        return

    key = self.data_key
    if key in E.DATA:
        if E.DATA[key].physics_rig == bpy.context.object:
            del E.DATA[key]
        else:
            self.data_key = max([k for k in E.DATA.keys()]) + 1

    bpy.ops.scene.ce_rig_setup()
    

def cb_update_alternate_target(self, context):

    if self.alternate_target is None:
        print("exited because alternate target was none")
    print("Update alternate target here.")
    

def cb_animated(self, context):
    bpy.context.scene.CP_props.skip_handler = False
    print("set skip handler to false")


def cb_control_rig_on_off(self, context):
    rig = bpy.context.object
    bones = rig.pose.bones
    for b in bones:
        for c in b.constraints:
            if c.name.startswith("CP_CP_"):
                c.enabled = self.control_rig_on
                
                
def cb_control_rig_influence(self, context):
    rig = bpy.context.object
    bones = rig.pose.bones
    for b in bones:
        for c in b.constraints:
            if c.name.startswith("CP_CP_"):
                c.influence = self.control_rig_influence
    

# Define a callback function to update the dropdown menu items
def update_shape_keys(self, context):
    obj = bpy.context.object
    items = [(key.name, key.name, '') for key in obj.data.shape_keys.key_blocks]
    obj.CP_props.my_enum_prop_items = items


def get_animation_actions(self, context):
    #return ((a.name, a.name, 'Action to use for target') for a in bpy.data.actions)
    return ((a.name, a.name, 'Action to use for target') for a in bpy.data.actions)


def enum_callback_example(self, context):
    return (
        ('NONE', 'None', "Flat geometry"),
        ('GEOM', 'Geometry', "Use z value from shape geometry if exists"),
        ('FIELD', 'Field', "Extract z elevation value from an attribute field"),
        ('OBJ', 'Object', "Get z elevation value from an existing ground mesh"),
    )


# object properties -----------
class CpPropsObject(bpy.types.PropertyGroup):
    
    test_prop: bpy.props.FloatProperty(
        name="Test prop",
        description="Used for whatever",
        default=1.0,
    )

    control_rig_on: bpy.props.BoolProperty(
        name="Control Rig On",
        description="Turn control rig constraints on or off.",
        update=cb_control_rig_on_off,
        default=True,
    )
    
    copy_roll: bpy.props.BoolProperty(
        name="Copy Roll",
        description="Copy roll of active bone to matching control rig bone.",
        default=True,
    )
    
    control_rig_influence: bpy.props.FloatProperty(
        name="Control Rig Influence",
        description="Set the influence of all control rig constraints.",
        update=cb_control_rig_influence,
        default=1.0,
        min=0.0,
        max=1.0,
    )
    
    rot_reducer: bpy.props.FloatProperty(
        name="Rot Reducer",
        description="Decrease rotation force",
        default=1.0,
    )

    advanced_mode: bpy.props.BoolProperty(
        name="Advanced Target Mode",
        description="Set action, start and end frame for pose target.",
        default=False,
    )
    
    target_frame_start: bpy.props.IntProperty(
        name="Target Frame Start",
        description="Loop back to this frame in with the pose target",
        default=1,
    )    

    target_frame_end: bpy.props.IntProperty(
        name="Target Frame End",
        description="Loop back to start when this matchs Target Frame Current",
        default=1,
    )    

    target_frame_current: bpy.props.IntProperty(
        name="Target Frame Current",
        description="Keeps track of the pose target current from for looping using Target Frame Start and Target Frame End",
        default=1,
    )

    target_frame_reset: bpy.props.IntProperty(
        name="Target Reset at Frame",
        description="Set the current frame back to the start frame",
        default=1,
    )

    target_action: bpy.props.EnumProperty(
        items=get_animation_actions,
        name="Target Action",
        description="Use this action with the pose target",
        default=None,
    )
        #options={'ANIMATABLE'},
        #update=None,
        #get=None,
        #set=None)

#    target_action: bpy.props.EnumProperty(
#        items=[i.name for i in bpy.data.actions],
#        description="Use this action with the pose target",
#        name="Target Action",
#    )    
    
    reset_to_active_shape: bpy.props.BoolProperty(
        name="Shape Reset",
        description="Use the active shape key for reset",
        default=True,
    )

    reset_at_frame: bpy.props.IntProperty(
        name="Reset at Frame",
        description="If the current frame matches reset",
        default=1,
    )

    record_to_keyframes: bpy.props.BoolProperty(
        name="Record",
        description="Record To Keyframes",
        default=False,
    )
    
    record_selection_only: bpy.props.BoolProperty(
        name="Record Selection Only",
        description="Record only bones selected in pose mode",
        default=False,
    )

    record_location: bpy.props.BoolProperty(
        name="Record Location",
        description="Keyframe Location",
        default=True,
    )

    record_rotation: bpy.props.BoolProperty(
        name="Record Rotation",
        description="Keyframe Rotation As Quaternion",
        default=True,
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
    
    end_frame: bpy.props.IntProperty(
        name="End Frame",
        description="Stop recording when Record Frame hits this number",
        default=-1,
    )    

    skip_frames: bpy.props.IntProperty(
        name="Skip Frames",
        description="Record Every Nth Frame",
        default=0,
    )
    
    action_name: bpy.props.StringProperty(
        name="Action Name",
        description="Record keyframes to this action.",
        default="KITTENS WILL RULE!!!",
    )

    target_object: bpy.props.PointerProperty(
        name="Target Object",
        type=bpy.types.Object,
        description="Select the target object",
    )
    
    test_array = np.zeros(12)
    shape = (4,3)
    test_array.shape = (4,3)
    test_string = U.numpy_array_to_string(test_array)
    
    velocity_array: bpy.props.StringProperty(
        name="Velocity Array",
        description="Store the velocity as a string",
        default=test_string,
    )
    
    vel_start_array: bpy.props.StringProperty(
        name="Velocity Start Array",
        description="Store the start velocity as a string",
        default=test_string,
    )

    current_co_array: bpy.props.StringProperty(
        name="Current Co Array",
        description="Store the current_co as a string",
        default=test_string,
    )

    data_key: bpy.props.IntProperty(
        name="Data Key",
        description="Key for storing and retrieving data",
        default=0,
    )
    
    id_key: bpy.props.IntProperty(
        name="ID Key",
        description="Because references disappear after undo etc.",
        default=0,
    )    
    
    pose_target: bpy.props.PointerProperty(
        name="Pose Target",
        type=bpy.types.Object,
        description="Target the pose of this matching armature.",
        update=cb_update_pose_target,
    )
    
    control_target: bpy.props.PointerProperty(
        name="Control Target",
        type=bpy.types.Object,
        description="Target the physics rig you want to influence your existing rig. For making it easier to set up constraints by filling in the armature target.",
        #update=cb_update_control_target,
    )
    
    mocap_target: bpy.props.PointerProperty(
        name="Mocap Target",
        type=bpy.types.Object,
        description="Influenced by this rig's motion",
        update=cb_update_pose_target,
    )    
    
    mocap_influence: bpy.props.FloatProperty(
        name="Motion Capture Influence",
        description="Amount of influence we get from the mocap target.",
        default=0,
        soft_min=0,
        soft_max=1,
        update=cb_update_mocap_influence,
    )
    
    gravity_object: bpy.props.PointerProperty(
        name="Gravity Object",
        type=bpy.types.Object,
        description="Gravity direction from this object's Z axis.",
        update=cb_update_pose_target,
    )

    advanced_anim_action: bpy.props.PointerProperty(
        name="Target Action",
        type=bpy.types.Action,
        description="Action used by advanced animation settings",
        #default=None
    )
    
    physics_rig: bpy.props.PointerProperty(
        name="Physics Rig",
        type=bpy.types.Object,
        description="Points to the physics rig from the pose_target and mix_mesh",
    )        
    
    mix_mesh: bpy.props.PointerProperty(
        name="Mix Mesh",
        type=bpy.types.Object,
        description="Points to a rig's mix mesh",
    )    
    
    alternate_target: bpy.props.PointerProperty(
        name="Alternate Target",
        type=bpy.types.Object,
        description="Write to this rig instead of the physics rig.",
        update=cb_update_alternate_target,
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
        default=0.2,
        min=0.0,
        max=1.0,
        update=cb_object_friction,
    )

    object_static_friction: bpy.props.FloatProperty(
        name="Static Friction",
        description="This much distance required to break friction",
        default=0.03,
        min=0.0,
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
        soft_min=0.0,
        soft_max=1.0,
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
        update=cb_handler_update,
    )

    live: bpy.props.BoolProperty(
        name="Live",
        description="Runs physics all the time even during the zombie apocalypse",
        default=False,
        update=cb_handler_update,
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
        default=1,
    )

    # ITERS
    sub_frames: bpy.props.IntProperty(
        name="Sub Frame",
        description="number all solves between frames",
        default=1,
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
    influence_is_rotation: bpy.props.BoolProperty(
        name="Is Rotation",
        description="Empties that are rotation targets",
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
    influence_rotation: bpy.props.FloatProperty(
        name="Inf Rotation",
        description="Rotation spring",
        default=0.0,
        precision=6,
        soft_min=0.0,
        soft_max=1.0,
        update=cb_update_influence,
    )
    
    # INFLUENCE ---------------
    stabilizer: bpy.props.BoolProperty(
        name="Stabilizer",
        description="For using rotation influence targets to stabilize a rig.",
        default=True,
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
class CpPropsScene(bpy.types.PropertyGroup):
    
    # subscribe and donate -------------------------------------
    subscribe:\
    bpy.props.BoolProperty(name="Subscribe",
    description="Check out options to subscribe or support Character Physics",
    default=False)

    paypal:\
    bpy.props.BoolProperty(name="Paypal Subscribe",
    description="Support Character Physics through paypal subscription",
    default=False,
    update=cb_paypal)

    gumroad:\
    bpy.props.BoolProperty(name="Gumroad Subscribe",
    description="Support Character Physics through gumroad subscription",
    default=False,
    update=cb_gumroad)

    patreon:\
    bpy.props.BoolProperty(name="Patreon Subscribe",
    description="Support Character Physics through patreon subscription",
    default=False,
    update=cb_patreon)

    donate:\
    bpy.props.BoolProperty(name="Donate",
    description="Support Character Physics through paypal donation",
    default=False,
    update=cb_donate)
    # -----------------------------------------------------------


    test: bpy.props.BoolProperty(
        name="Test scene prop",
        description="Is this working?",
        default=False,
        update=cb_test_prop,
    )

    skip_load: bpy.props.BoolProperty(
        name="Skip Auto Load",
        description="If load is crashing bleder try this",
        default=False,
    )

    skip_handler: bpy.props.BoolProperty(
        name="Skip Handler",
        description="Skip the frame handler for recording",
        default=False,
    )

    collision_holdback: bpy.props.FloatProperty(
        name="Collision Holdback",
        description="Recover Collisions by this amount",
        default=1.0,
    )

    collision_margin: bpy.props.FloatProperty(
        name="Collision Holdback",
        description="Detect collisions below this amount",
        default=0.0,
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


def clear_references(ob):
    if "mix_mesh" in ob:
        del ob["mix_mesh"]
    if "target_rig" in ob:
        del ob["target_rig"]
    if "physics_rig" in ob:
        del ob["physics_rig"]


def rig_setup(rig=None):
        
    dead_keys = []
    for k, ph in E.DATA.items():
        if ph.physics_rig:
            continue
        else:
            dead_keys += [k]
            print("dead key was", k)
    
    for k in dead_keys:    
        print("deleted dead key:", k)
        del(E.DATA[k])
    
    if rig is None:    
        rig = bpy.context.object
    if rig.type != "ARMATURE":
        msg = "Selct an armature"
        bpy.context.window_manager.popup_menu(U.oops, title=msg, icon='ARMATURE_DATA')
        return

    keys = [k for k in E.DATA.keys()]
    rig_key = rig.CP_props.data_key
    
    if rig_key in keys:
        ph = E.DATA[rig_key]
        if ph.physics_rig == rig:
            del(E.DATA[rig_key])
            print("deleted key", rig_key)
        else:
            rig_key = max(keys) + 1
            rig.CP_props.data_key = rig_key
            print("added key", rig_key)
            
    pose_target = rig.CP_props.pose_target
    
    exit = False
    if pose_target is None:
        exit = True
    if pose_target.type != "ARMATURE":
        exit = True
    if exit:
        msg = "Pose target needs to be a matching armature."
        bpy.context.window_manager.popup_menu(U.oops, title=msg, icon='ARMATURE_DATA')
        rig.CP_props['pose_target'] = None
        del(E.DATA[rig_key])
        return

    rig_mode, rig_name = set_rest_mode(pose_target)
    p_rig_mode, p_rig_name = set_rest_mode(rig)
    
    try:    
        ph = E.physics(rig, pose_target)
    except:
        msg = "Problem with setup. Probably the number of bones didn't match the pose target"
        bpy.context.window_manager.popup_menu(U.oops, title=msg, icon='GHOST_ENABLED')
        del(E.DATA[rig.CP_props.data_key])
        bpy.data.armatures[rig_name].pose_position = rig_mode
        bpy.data.armatures[p_rig_name].pose_position = p_rig_mode
        return
        
    rig.CP_props.mix_mesh = ph.mix_mesh
    pose_target.CP_props.physics_rig = rig
    ph.mix_mesh.CP_props.physics_rig = rig
    ph.physics_rig = rig
    
    M = rig.matrix_world
    ph.mix_mesh.matrix_world = M
    
    live = rig.CP_props.live
    animated = rig.CP_props.animated
    if live:
        E.install_handler_live()
    if animated:
        E.install_handler_animated()
    
    clear_references(rig)
    clear_references(pose_target)
    clear_references(ph.mix_mesh)
    
    bpy.data.armatures[rig_name].pose_position = rig_mode
    bpy.data.armatures[p_rig_name].pose_position = p_rig_mode
     
    rig_id = U.set_id_key(rig)
    target_id = U.set_id_key(pose_target)
    mix_mesh_id = U.set_id_key(ph.mix_mesh)
    
    ph.physics_rig_id = rig_id
    ph.pose_target_id = target_id
    ph.mix_mesh_id = mix_mesh_id
        
    cb_update_collider(None, None)
    return ph


class CpRigSetup(bpy.types.Operator):
    """Setup physics data for rig."""

    bl_idname = "scene.ce_rig_setup"
    bl_label = "Ce Rig Setup"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        rig_setup()
        return {'FINISHED'}
                            

class CpTestLoad(bpy.types.Operator):
    """Debugging the load handler."""

    bl_idname = "scene.cp_test_load"
    bl_label = "CP Test Load"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):

        load = bpy.context.scene.CP_props.skip_load
        bpy.context.scene.CP_props.skip_load = False
        Ce_load_handler()
        bpy.context.scene.CP_props.skip_load = load
        return {'FINISHED'}


def make_influence_target(type='head', make_parent=False):
    
    exit = False
    physics_rig = bpy.context.object
    key = bpy.context.object.CP_props.data_key
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
        em.CP_props.influence_is_target = True
        em.show_in_front = True
        em.CP_props.target_object = ph.physics_rig
        
        head_idx = int(co_idx[e])
        tail_idx = int(co_idx_tial[e])

        em.CP_props.influence_head_idx = head_idx
        em.CP_props.influence_tail_idx = tail_idx
        
        npm = np.array(b.matrix)
        if type == 'head':
            b.CP_props.target_object = em
            em.location = ph.physics_rig.matrix_world @ b.head
        
        if type == 'tail':
            b.CP_props.tail_target_object = em
            em.location = ph.physics_rig.matrix_world @ b.tail
            em.CP_props.influence_is_tail = True
            
        if type == 'rotation':
            b.CP_props.rotation_target_object = em
            loc1 = ph.physics_rig.matrix_world @ b.tail
            loc2 = ph.physics_rig.matrix_world @ b.head
            em.location = np.mean([loc1, loc2], axis=0)
            em.CP_props.influence_is_rotation = True    
                
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
        U.rig_reset(E.DATA, ph=None)
        return {'FINISHED'}


def delete_all_cp_constraints(rig):
    for b in rig.pose.bones:
        for c in b.constraints:
            if c.name.startswith("CP_CP_"):
                b.constraints.remove(c)


def manage_constraints(c):
    c.name = "CP_CP_" + c.name
    rig = bpy.context.object
    target = rig.CP_props.control_target
    if target:
        c.target = target
        close_idx = A.compare_rig_spaces(rig, target)
        bone = target.pose.bones[close_idx]
        c.subtarget = bone.name
        if rig.CP_props.copy_roll:
            A.copy_roll(target, close_idx)


class CpControlRigNew(bpy.types.Operator):
    """Add a control rig with matching transforms."""

    bl_idname = "scene.cp_control_rig_new"
    bl_label = "Cp Control Rig New"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):

        rig = bpy.context.object
        new_rig = U.link_armature(rig=rig)
        new_rig.name = "Control_" + rig.name
        rig.CP_props.control_target = new_rig
                            
        return {'FINISHED'}


class CpControlRigCopy(bpy.types.Operator):
    """Add a CP copy transforms constraint"""

    bl_idname = "scene.cp_control_rig_copy"
    bl_label = "Cp Control Rig Copy"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        c = bpy.context.active_pose_bone.constraints.new("COPY_TRANSFORMS")
        manage_constraints(c)
        c.target_space = "LOCAL_WITH_PARENT"
        c.owner_space = "LOCAL_WITH_PARENT"
                    
        return {'FINISHED'}


class CpControlRigIk(bpy.types.Operator):
    """Add a CP IK constraint"""

    bl_idname = "scene.cp_control_rig_ik"
    bl_label = "Cp Control Rig IK"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        c = bpy.context.active_pose_bone.constraints.new("IK")
        manage_constraints(c)
            
        return {'FINISHED'}


class CpControlRigChild(bpy.types.Operator):
    """Add a CP Child Of constraint"""

    bl_idname = "scene.cp_control_rig_child_of"
    bl_label = "Cp Control Rig Child Of"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        c = bpy.context.active_pose_bone.constraints.new("CHILD_OF")
        manage_constraints(c)
        
        return {'FINISHED'}


class CpControlRigDeleteAll(bpy.types.Operator):
    """Delete all CP constraints"""

    bl_idname = "scene.cp_control_rig_delete_all"
    bl_label = "Cp Control Rig Delete All"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        delete_all_cp_constraints(bpy.context.object)
        return {'FINISHED'}


class CeDisconnectBones(bpy.types.Operator):
    """Set the connect state of all bones
    in a rig. Usefull for ropes or things
    that stretch"""

    bl_idname = "scene.ce_disconnect_bones"
    bl_label = "Ce Disonnect Bones"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        A.connect_bones(bpy.context.object, connected=False)
        return {'FINISHED'}


class CeConnectBones(bpy.types.Operator):
    """Set the connect state of all bones
    in a rig. Usefull for ropes or things
    that stretch"""

    bl_idname = "scene.ce_connect_bones"
    bl_label = "Ce Connect Bones"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        ob = bpy.context.object
        A.connect_bones(bpy.context.object, connected=True)
        for k, ph in E.DATA.items():    
            E.update_display(ph)
        return {'FINISHED'}


class CeClearParents(bpy.types.Operator):
    """Clear parents and connect state
    of all bones"""

    bl_idname = "scene.ce_clear_parents"
    bl_label = "Ce Clear Parents"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        A.connect_bones(bpy.context.object, connected=False, clear_parents=True)
        for k, ph in E.DATA.items():    
            E.update_display(ph)
        return {'FINISHED'}


class CeMakeInfluenceTarget(bpy.types.Operator):
    """Set up empty for selected bones
    to serve as head target"""

    bl_idname = "scene.ce_make_influence_target"
    bl_label = "Ce Make Influence Target"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        make_influence_target(type='head')
        return {'FINISHED'}


class CeMakeInfluenceTargetTail(bpy.types.Operator):
    """Set up empty for selected bones
    to serve as tail target"""

    bl_idname = "scene.ce_make_influence_target_tail"
    bl_label = "Ce Make Influence Target Tail"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        make_influence_target(type='tail')
        return {'FINISHED'}
    
    
class CeMakeInfluenceTargetRotation(bpy.types.Operator):
    """Set up empty for selected bones
    to serve as rotation target"""

    bl_idname = "scene.ce_make_influence_target_rotation"
    bl_label = "Ce Make Influence Target Rotation"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        make_influence_target(type='rotation')
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


class CpResetAngular(bpy.types.Operator):
    """Reset bone angular force to 1.0
    for all bones in active object."""

    bl_idname = "scene.ce_reset_angular"
    bl_label = "Ce Reset Bone Angular"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        if not bpy.context.object:
            return {'FINISHED'}
            
        if bpy.context.object.type != "ARMATURE":
            return {'FINISHED'}
        
        reset_angular(bpy.context.object)
        return {'FINISHED'}

    
#=======================#
# UI -------------------#
#=======================#
class PANEL_PT_Character_Engine_Colliders(bpy.types.Panel):
    """CP Colliders Panel"""

    bl_label = "Colliders"
    bl_idname = "PANEL_PT_Character_Engine_Colliders"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Character Physics"
    
    def draw(self, context):
        layout = self.layout
        col = layout.column(align=True)

        ob = bpy.context.object
        if ob:    
            if ob.type == "MESH":
                col.prop(ob.CP_props, "collider", text="Collider")
                col.prop(ob.CP_props, "object_friction", text="Friction")
                return
        col.label(text="Select Mesh Object")


class PANEL_PT_Character_Engine_Setup(bpy.types.Panel):
    """CP Setup Panel"""

    bl_label = "Setup"
    bl_idname = "PANEL_PT_Character_Engine_Setup"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Character Physics"
    
    def draw(self, context):
        layout = self.layout
        col = layout.column(align=True)
        
        sc = bpy.context.scene
        heart = 'ORPHAN_DATA'
        if sc.CP_props.subscribe:
            heart = 'FUND'
        col.prop(sc.CP_props, "subscribe", text="Subscribe", icon=heart)
        if sc.CP_props.subscribe:    
            col.prop(sc.CP_props, "paypal", text="Paypal", icon='HEART')
            col.prop(sc.CP_props, "gumroad", text="Gumroad", icon='HEART')
            col.prop(sc.CP_props, "patreon", text="Patreon", icon='HEART')
            col.prop(sc.CP_props, "donate", text="Donate", icon='HEART')
        col.separator()

        ob = bpy.context.object
        if ob:
            if ob.type == "ARMATURE":
                box = layout.box()
                box.label(text="Pose Target:")
                box.prop(bpy.context.object.CP_props, "pose_target", text="")
                
                if ob.CP_props.pose_target:
                    box.label(text="Alt Target:")
                    box.prop(bpy.context.object.CP_props, "alternate_target", text="")
                    col = box.column()
                    col.scale_y = 1.7    
                    col.operator("scene.ce_rig_setup", text="Physics Setup", icon='ARMATURE_DATA')

                    box = layout.box()
                    box.operator("scene.ce_connect_bones", text="Connet Bones", icon='CONSTRAINT_BONE')
                    #col.operator("scene.ce_disconnect_bones", text="Disconnect Bones", icon='GROUP_BONE')
                    box.operator("scene.ce_clear_parents", text="Clear Parents", icon='GROUP_BONE')
                    box = layout.box()
                    box.prop(bpy.context.object.CP_props, "reset_to_active_shape", text="Shape Reset")
                    box.prop(bpy.context.object.CP_props, "reset_at_frame", text="Frame Reset")

                #---
                col = box.column()
                col.scale_y = 1.7
                col.operator("scene.ce_rig_reset", text="Reset", icon='FILE_REFRESH')
            
            #box.prop(bpy.context.object.CP_props, "test_prop", text="Test Prop")

                #---
       
                return
        col.label(text="Select Armature Object")
                
                
class PANEL_PT_Character_Engine_Tools(bpy.types.Panel):
    """Cp Tools Panel"""

    bl_label = "Influence Targets"
    bl_idname = "PANEL_PT_Character_Engine"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Character Physics"
    
    def draw(self, context):
        layout = self.layout

        ob = bpy.context.object
        if ob:

            if bpy.context.object.CP_props.influence_is_target:
                box = layout.box()
                col = box.column()                
                #col.label(text="Influence Baby")
                col.prop(bpy.context.object.CP_props, "influence_directional", text="Directional")
                col.prop(bpy.context.object.CP_props, "influence_spring", text="Spring")
            
            if bpy.context.object.CP_props.influence_is_rotation:
                col.prop(bpy.context.object.CP_props, "influence_rotation", text="Rotation")
                col.prop(bpy.context.object.CP_props, "stabilizer", text="Stabilizer")
            
#            if not ob.type == "ARMATURE":
#                col = layout.column()
#                col.label(text="Select Armature Object")
#                return
#        
#            box = layout.box()
#            box.operator("scene.ce_rig_reset", text="Reset", icon='FILE_REFRESH')
#            
#            if ob.CP_props.pose_target is None:
#                col = layout.column()
#                col.label(text="Setup Pose Target")
#                return

#            box.prop(bpy.context.object.CP_props, "reset_at_frame", text="Frame Reset")
#            box.prop(bpy.context.object.CP_props, "reset_to_active_shape", text="Use Active Shape")
#            #box.prop(bpy.context.object.CP_props, "test_prop", text="Test Prop")

            phys_rig = (ob.type == "ARMATURE") & (ob.CP_props.pose_target is not None)
            if phys_rig:
            
                bone = bpy.context.active_pose_bone
                if bone:                
                    box = layout.box()
                    col = box.column()
                    #col.label(text="Influence Targets")
                    col.operator("scene.ce_make_influence_target", text="Head Target", icon='OUTLINER_DATA_ARMATURE')
                    col.operator("scene.ce_make_influence_target_tail", text="Tail Target", icon='OUTLINER_DATA_ARMATURE')
                    col.operator("scene.ce_make_influence_target_rotation", text="Rotation Target", icon='OUTLINER_DATA_ARMATURE')
                    #col.prop(bone.CP_props, "target", text="Influence Target")
                    #col.prop(bpy.context.object.CP_props, "node_size", text="Node Size")
                    
                    col.label(text="Head")
                    col.prop(bone.CP_props, "target_object", text="Influence Target")
                    target = bone.CP_props.target_object
                    
                    if target:
                        col.prop(target.CP_props, "influence_directional", text="Directional")
                        col.prop(target.CP_props, "influence_spring", text="Spring")
                        #col.prop(target.CP_props, "influence_rotation", text="Rotation")
                    
                    col.label(text="Tail")
                    col.prop(bone.CP_props, "tail_target_object", text="Influence Target")
                    
                    col.label(text="Rotation")
                    col.prop(bone.CP_props, "rotation_target_object", text="Influence Target")
                    tail_target = bone.CP_props.tail_target_object
                    if tail_target:    
                        col.prop(tail_target.CP_props, "influence_directional", text="Directional")
                        col.prop(tail_target.CP_props, "influence_spring", text="Spring")
                        #col.prop(tail_target.CP_props, "influence_rotation", text="Rotation")                
                    rotation_target = bone.CP_props.rotation_target_object
                    if rotation_target:
                        col.prop(rotation_target.CP_props, "influence_rotation", text="Rotation")
                    return

                col = layout.column()
                col.label(text="No Active Bone")
                return

            col = layout.column()
            col.label(text="Select Physic Rig")
                    

class PANEL_PT_Character_Engine_Physics(bpy.types.Panel):
    """Cp Physics Panel"""

    bl_label = "Physics"
    bl_idname = "PANEL_PT_Character_Engine_Physics"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Character Physics"
    
    def draw(self, context):
        layout = self.layout

        ob = bpy.context.object
        
        if ob:            
            phys_rig = (ob.type == "ARMATURE") & (ob.CP_props.pose_target is not None)
            pose_target = (ob.type == "ARMATURE") & (ob.CP_props.physics_rig is not None)
            
            if phys_rig:
                rig = ob
                
            if pose_target:
                rig = ob.CP_props.physics_rig
                                        
            if phys_rig | pose_target:
                box = layout.box()
                col = box.column()
                col.label(text="Forces")
                col.prop(rig.CP_props, "gravity", text="Gravity")
                col.prop(rig.CP_props, "gravity_object", text="Gravity Object")
                col.prop(rig.CP_props, "velocity", text="Velocity")
                #col.prop(bpy.context.object.CP_props, "angular_force", text="Angular Force")
                        
                col.label(text="Solve Iterations")
                col.prop(rig.CP_props, "linear_iters", text="Linear")
                col.prop(rig.CP_props, "angular_iters", text="Angular")
                #col.prop(bpy.context.object.CP_props, "rot_reducer", text="A Force")
                #col.prop(bpy.context.object.CP_props, "linear_post_iters", text="Lin Post")
                col.prop(rig.CP_props, "sub_frames", text="sub frame")
            
                #col.label(text="bone targets")
                if phys_rig:    
                    bone = bpy.context.active_pose_bone
                    if bone:
                        box = layout.box()
                        col = box.column()
                        col.label(text="Mass")
                        selected = [b for b in ob.pose.bones if b.bone.select]
                        col.prop(bone.CP_props, "head_mass", text="Mass")
                        #if len(selected) == 1:
                        #if len(selected) > 1:
                            #col.prop(bone.CP_props, "selected_bone_mass", text="Mass Selected")
                        col.operator("scene.ce_reset_mass", text="Reset Mass", icon='FILE_REFRESH')
                        #col.prop(bone.CP_props, "tail_mass", text="Mass Tail")
                        col.prop(bone.CP_props, "angle_force", text="Angle Force")
                        col.operator("scene.ce_reset_angular", text="Reset Angular", icon='FILE_REFRESH')
                        #col.prop(bone.CP_props, "parent_influence", text="Parent Influence")
                        
                        col.label(text="Friction")
                        col.prop(bone.CP_props, "head_friction", text="Friction Head")
                        col.prop(bone.CP_props, "tail_friction", text="Friction Tail")
                        col.prop(ob.CP_props, "object_friction", text="All Bones")
                        col.prop(ob.CP_props, "object_static_friction", text="Static Friction Distance")
                        
                        col.label(text="Collide Radius")
                        col.prop(bone.CP_props, "head_node_size", text="Head Radius")
                        col.prop(bone.CP_props, "tail_node_size", text="Tail Radius")
                            
                    else:
                        col.label(text="Friction")
                        col.prop(ob.CP_props, "object_friction", text="All Bones Friction")            
                        col.prop(ob.CP_props, "object_static_friction", text="Static Friction Distance")
                    
#            if bpy.context.object.CP_props.influence_is_target:
#                col.label(text="Influence Targets")
#                col.prop(bpy.context.object.CP_props, "influence_directional", text="Directional")
#                col.prop(bpy.context.object.CP_props, "influence_spring", text="Spring")
#            
#            if bpy.context.object.CP_props.influence_is_rotation:
#                col.prop(bpy.context.object.CP_props, "influence_rotation", text="Rotation")


class PANEL_PT_Character_Engine_Record(bpy.types.Panel):
    """Cp Record/Playback Panel"""

    bl_label = "Record/Playback"
    bl_idname = "PANEL_PT_Character_Engine_Record"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Character Physics"
    
    def draw(self, context):
        layout = self.layout

        ob = bpy.context.object
        if ob:
            phys_rig = (ob.type == "ARMATURE") & (ob.CP_props.pose_target is not None)
            if phys_rig:
                box = layout.box()
                box.prop(bpy.context.object.CP_props, "record_to_keyframes", text="Record to Keyframes")
                box.prop(bpy.context.object.CP_props, "skip_frames", text="Skip Frames")
                box.prop(bpy.context.object.CP_props, "record_frame", text="Record Frame")
                box.prop(bpy.context.object.CP_props, "end_frame", text="End Frame")
                box.prop(bpy.context.object.CP_props, "action_name", text="Action Name")
                box.label(text="Settings")
                box.prop(bpy.context.object.CP_props, "record_selection_only", text="Selected Only")
                box.prop(bpy.context.object.CP_props, "record_location", text="Location")
                box.prop(bpy.context.object.CP_props, "record_rotation", text="Rotation")
                
                
                box = layout.box()
                box.label(text="Playback")
                box.prop(bpy.context.object.CP_props, "advanced_mode", text="Advanced")
                if bpy.context.object.CP_props.advanced_mode:
                    #col.prop(bpy.context.object.CP_props, "target_action_name", text="Action")
                    box.prop(bpy.context.object.CP_props, "target_frame_start", text="Start Frame")
                    box.prop(bpy.context.object.CP_props, "target_frame_end", text="End Frame")            
                    box.prop(bpy.context.object.CP_props, "target_frame_current", text="Current Frame")            
                    box.prop(bpy.context.object.CP_props, "target_frame_reset", text="Reset Frame")            
                    #box.prop(bpy.context.object.CP_props, "target_action", text="Action")
                    box.prop(bpy.context.object.CP_props, "advanced_anim_action", text="Action")
                
                box.prop(bpy.context.object.CP_props, "animated", text="Animated", toggle=True)
                box.prop(bpy.context.object.CP_props, "live", text="Live", toggle=True)
                return
            pose_target = (ob.type == "ARMATURE") & (ob.CP_props.physics_rig is not None)
            if pose_target:
                box = layout.box()
                phys_rig = ob.CP_props.physics_rig
                box.prop(phys_rig.CP_props, "animated", text="Animated", toggle=True)
                box.prop(phys_rig.CP_props, "live", text="Live", toggle=True)       
            

class PANEL_PT_Character_Engine_Settings(bpy.types.Panel):
    """Cp Settings Panel"""

    bl_label = "Settings"
    bl_idname = "PANEL_PT_Character_Engine_Settings"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Character Physics"
    
    def draw(self, context):
        layout = self.layout
        settings = True
        if settings:
            col = layout.column()
            col.label(text="settings")
            col.prop(bpy.context.scene.CP_props, "skip_load", text="Skip Auto Load")
            col.operator("scene.cp_test_load", text="Reload", icon='FILE_REFRESH')

            #col.prop(bpy.context.scene.CP_props, "collision_holdback", text="Collision Holdback")
            #col.prop(bpy.context.scene.CP_props, "collision_margin", text="Collision Margin")


class PANEL_PT_Character_Engine_Mocap(bpy.types.Panel):
    """Cp Motion Capture Panel"""

    bl_label = "Motion Capture Tools"
    bl_idname = "PANEL_PT_Character_Engine_Mocap"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Character Physics"
    
    def draw(self, context):
        layout = self.layout
        col = layout.column()

        #col.label(text="Mocap Tools")
        ob = bpy.context.object
        if ob:
            if ob.type == "ARMATURE":    
                col.prop(ob.CP_props, "mocap_target", text="Mocap Target")
                col.prop(ob.CP_props, "mocap_influence", text="Mocap influe")
            bone = bpy.context.active_pose_bone
            if bone:
                col.prop(bone.CP_props, "head_mocap_influence", text="Head Mocap Influence")
                col.prop(bone.CP_props, "tail_mocap_influence", text="Tail Mocap Influence")
            
        else:
            col.label(text="Select Physics Rig")
            

class PANEL_PT_Character_Engine_Control_Rig(bpy.types.Panel):
    """Cp Control Rig Panel"""

    bl_label = "Control Rig Tools"
    bl_idname = "PANEL_PT_Character_Control_Rig"
    bl_space_type = 'VIEW_3D'
    bl_region_type = 'UI'
    bl_category = "Character Physics"
    
    def draw(self, context):
        layout = self.layout
        col = layout.column()

        #col.label(text="Mocap Tools")
        ob = bpy.context.object
        if ob:
            if ob.type == "ARMATURE":    
                col.operator("scene.cp_control_rig_new", text="Create Control Rig")
                col.prop(ob.CP_props, "control_target", text="Control Target")
                col.prop(ob.CP_props, "control_rig_on", text="On/Off")
                col.prop(ob.CP_props, "control_rig_influence", text="Influence")
                col.prop(ob.CP_props, "copy_roll", text="Bone Roll to Target")
                col.operator("scene.cp_control_rig_copy", text="Add Copy Transforms")
                col.operator("scene.cp_control_rig_ik", text="Add IK")
                col.operator("scene.cp_control_rig_child_of", text="Add Child Of")
                col.operator("scene.cp_control_rig_delete_all", text="Remove All CP Constraints")
                #col.prop(ob.CP_props, "mocap_influence", text="Mocap influe")
            #bone = bpy.context.active_pose_bone
            #if bone:
                #col.prop(bone.CP_props, "head_mocap_influence", text="Head Mocap Influence")
                #col.prop(bone.CP_props, "tail_mocap_influence", text="Tail Mocap Influence")
            
        else:
            col.label(text="Select Character Rig")

#=======================#
# w/LOAD ------------#
#=======================#
@persistent
def Ce_save_handler(scene=None):
    for k, ph in E.DATA.items():        
        print("Saving CP data for: ", ph.physics_rig.name)
        rig = ph.physics_rig
        rig.CP_props.velocity_array = U.numpy_array_to_string(ph.velocity)
        rig.CP_props.current_co_array = U.numpy_array_to_string(ph.current_co)
        rig.CP_props.vel_start_array = U.numpy_array_to_string(ph.vel_start)


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
            if phys_parents[i] is not None:
                phys.data.edit_bones[i].parent = phys.data.edit_bones[phys_parents[i]]
            else:
                phys.data.edit_bones[i].parent = None
            
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
        else:
            phys.data.edit_bones[i].parent = None
        
    bpy.ops.object.mode_set(mode='OBJECT')

    bpy.ops.object.select_all(action='DESELECT')
    for ob in original_selected:
        ob.select_set(True)
        bpy.context.view_layer.objects.active = ob
        bpy.ops.object.mode_set(mode=original_object_modes[ob])
        
    return phys_parents


@persistent
def Ce_load_handler(scene=None):
    """Runs on load to update physics
    objects back to their saved state."""
    if bpy.context.scene.CP_props.skip_load:
        return
    
    for ob in bpy.data.objects:
        if ob.type == "ARMATURE":
            if ob.CP_props.pose_target:
                if ob.name in bpy.context.scene.objects:
                    if ob.CP_props.pose_target.name in bpy.context.scene.objects:
                        ph = rig_setup(rig=ob)                        
                        rig = ph.physics_rig
                        cco = rig.CP_props.current_co_array
                        ph.current_co = U.string_to_numpy_array(cco, shape=None, dtype=np.float32)
                        vel = rig.CP_props.velocity_array
                        ph.velocity = U.string_to_numpy_array(vel, shape=None, dtype=np.float32)
                        vel_start = rig.CP_props.vel_start_array
                        ph.vel_start = U.string_to_numpy_array(vel_start, shape=None, dtype=np.float32)
                        E.update_display(ph)
    

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
    PANEL_PT_Character_Engine_Colliders,
    PANEL_PT_Character_Engine_Setup,
    PANEL_PT_Character_Engine_Tools,
    PANEL_PT_Character_Engine_Physics,
    PANEL_PT_Character_Engine_Record,
    PANEL_PT_Character_Engine_Settings,
    PANEL_PT_Character_Engine_Mocap,
    PANEL_PT_Character_Engine_Control_Rig,
    CpPropsObject,
    CpPropsScene,
    CpPropsPoseBone,
    CpRigSetup,
    CeRigReset,
    CeConnectBones,
    CeDisconnectBones,
    CeClearParents,
    CeMakeInfluenceTarget,
    CeMakeInfluenceTargetTail,
    CeMakeInfluenceTargetRotation,
    CeResetMass,
    CpResetAngular,
    CpTestLoad,
    CpControlRigNew,
    CpControlRigCopy,
    CpControlRigIk,
    CpControlRigChild,
    CpControlRigDeleteAll,
)


def register():
    # classes
    from bpy.utils import register_class

    for cls in classes:
        register_class(cls)
    bpy.types.Scene.CP_props = bpy.props.PointerProperty(type=CpPropsScene)
    bpy.types.Object.CP_props = bpy.props.PointerProperty(type=CpPropsObject)
    bpy.types.PoseBone.CP_props = bpy.props.PointerProperty(type=CpPropsPoseBone)
    
    save_handler()
    load_handler()

    print("!!! clearing handler here !!!")
    E.install_handler_animated(skip=True)
    E.install_handler_live(skip=True)
    

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
