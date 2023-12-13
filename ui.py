import bpy
import os
import sys
import numpy as np
psep = os.path.sep
path = '/home/rich/Desktop/cloth/character_engine'
sys.path.append(path)

try:
    U = bpy.data.texts['utils.py'].as_module()
    Q = bpy.data.texts['quaternions.py'].as_module()
    C = bpy.data.texts['coordinates.py'].as_module()
    A = bpy.data.texts['armatures.py'].as_module()
    E = bpy.data.texts['character_engine.py'].as_module()
    
except:
    import utils as U
    import quaternions as Q
    import coordinates as C
    import armatures as A
    import character_engine as E
    importlib.reload(U)
    importlib.reload(Q)
    importlib.reload(C)
    importlib.reload(A)
    importlib.reload(E)



# task list:
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

print()
print()
print("to monetize:")
print("multiple rigs")
print("collistion objects")
print("No X collision")
print("publish addon")
print("record animation")


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
    ph = E.DATA['ph']
    E.influence_check(ph)


def cb_update_head_mass(self, context):
    mix_mesh, target_rig, physics_rig, ob = get_relatives(bpy.context.object)
    
    print("is this a bone?")
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

    if 'ph' in E.DATA:
        ph = E.DATA['ph']
        E.distribute_mass(ph)


def cb_update_tail_mass(self, context):
    mix_mesh, target_rig, physics_rig, ob = get_relatives(bpy.context.object)
    
    print("is this a bone?")
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
    
    if 'ph' in E.DATA:
        ph = E.DATA['ph']
        E.distribute_mass(ph)


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


def cb_update_friction(self, context):
    ph = E.DATA['ph']
    #ph.head_friction = np.array([b.Ce_props.head_friction for b in ph.target_rig.pose.bones], dtype=np.float32)
    #ph.tail_friction = np.array([b.Ce_props.tail_friction for b in ph.target_rig.pose.bones], dtype=np.float32)
    


#=======================#
# PROPERTIES -----------#
#=======================#
class CePropsPoseBone(bpy.types.PropertyGroup):

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
        default=1.0,
        update=cb_update_friction,
    )    

    tail_friction: bpy.props.FloatProperty(
        name="Friction at tail",
        description="simulate the friction of the mesh at bone tail",
        default=1.0,
        update=cb_update_friction,
    )


# object properties -----------
class CePropsObject(bpy.types.PropertyGroup):

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
        # update=cb_handler_update,
    )

    live: bpy.props.BoolProperty(
        name="Live",
        description="Runs physics all the time even during the zombie apocalypse",
        default=False,
        # update=cb_handler_update,
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
    )

    # INFLUENCE ---------------
    influence_spring_linear: bpy.props.FloatProperty(
        name="Inf spring linear",
        description="number all solves between frames",
        default=0.0,
    )

    # INFLUENCE ---------------
    influence_co_angular: bpy.props.FloatProperty(
        name="Inf co_lin",
        description="number all solves between frames",
        default=0.0,
    )

    # INFLUENCE ---------------
    influence_spring_angular: bpy.props.FloatProperty(
        name="Inf co_lin",
        description="number all solves between frames",
        default=0.0,
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
        update=cb_update_influence,
    )

    # INFLUENCE ---------------
    influence_directional: bpy.props.FloatProperty(
        name="Inf Spring",
        description="Move in target direction",
        default=0.0,
        update=cb_update_influence,
    )



# scene properties ----------
class CePropsScene(bpy.types.PropertyGroup):
    test: bpy.props.BoolProperty(
        name="Test scene prop",
        description="Is this working?",
        default=False,
        update=cb_test_prop,
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


def rig_setup():
    target_rig = bpy.context.object
    #rig_name = target_rig.data.name
    #rig_mode = bpy.data.armatures[rig_name].pose_position
    #bpy.data.armatures[rig_name].pose_position = "REST"
    #bpy.context.view_layer.update()
    
    rig_mode, rig_name = set_rest_mode(target_rig)
    
    print(target_rig.type, ": Should be an armature")
    if not target_rig.type == 'ARMATURE':
        print("select an armature goofball")
        return
  
    phr = None
    selected = [i for i in bpy.context.selected_objects if (i != target_rig) & (i.type == "ARMATURE")]
    
    if len(selected) == 1:
        phr = selected[0]
    
    physics_rig = bpy.context.object    
    if phr is not None:
        physics_rig = phr

    p_rig_mode, p_rig_name = set_rest_mode(physics_rig)



    ph = E.physics(physics_rig, target_rig)
    #print("Warning! Set a global physics object")
    #E.ph = ph
    #look for ph as global in E    
    
    M = physics_rig.matrix_world
    ph.mix_mesh.matrix_world = M
    
    live = physics_rig.Ce_props.live
    animated = physics_rig.Ce_props.animated
    E.install_handler(live=live, animated=animated)

    physics_rig["mix_mesh"] = ph.mix_mesh.id_data
    physics_rig["target_rig"] = target_rig.id_data
    target_rig["mix_mesh"] = ph.mix_mesh.id_data
    target_rig["physics_rig"] = physics_rig.id_data
    ph.mix_mesh.id_data["target_rig"] = target_rig.id_data
    ph.mix_mesh.id_data["physics_rig"] = physics_rig.id_data
    
    bpy.data.armatures[rig_name].pose_position = rig_mode
    bpy.data.armatures[p_rig_name].pose_position = p_rig_mode
    

class CeRigSetup(bpy.types.Operator):
    """Setup physics data for rig."""

    bl_idname = "scene.ce_rig_setup"
    bl_label = "Ce Rig Setup"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        print("when child bones have the same parent then need to be evaluated as relative to each other.")
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
    ph = E.DATA['ph']
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
    
    if mix_mesh is not None:
        C.co_to_shape(mix_mesh, co=None, key="Current")
        C.co_to_shape(mix_mesh, co=None, key="Target")
        C.co_to_shape(mix_mesh, co=None, key="Basis")
        
        vc = len(mix_mesh.data.vertices)
        ph.current_co = np.empty((vc, 3), dtype=np.float32)
        C.get_shape_co_mode(ob=mix_mesh, co=ph.current_co, key='Current')
        ph.velocity = np.zeros((vc, 3), dtype=np.float32)
        ph.vel_start = np.empty((vc, 3), dtype=np.float32)
        ph.vel_start[:] = ph.current_co
        ph.physics_m3 = A.get_ar_matrix_world(target_rig, m3=True)
        


#import bpy
#from bpy import context




def make_influence_target(ph=None, type='head', make_parent=False):
    
    if ph is None:
        ph = E.DATA['ph']
    sel_bool = [b.bone.select for b in ph.physics_rig.pose.bones]
    selected = [b for b in ph.physics_rig.pose.bones if b.bone.select]
    #matricies = np.array[b.matrix for b in selected]
    bidx = np.arange(len(ph.physics_rig.pose.bones))
    sel_idx = bidx[sel_bool]
    co_idx = ph.mesh_bone_idx[sel_idx]
    co_idx_tial = ph.mesh_bone_tail_idx[sel_idx]
    display_type = "ARROWS"
    for e, b in enumerate(selected):
        name=b.name + "_inf_tar"
        em = U.create_empty(display_type, name, b.matrix, scene=None)
        em.Ce_props.influence_is_target = True
        em.show_in_front = True
        print()
        print("========")
        print(ph.mesh_bone_idx[e], "what idx is this?")
        print("========")
        print()
        
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
        if bpy.context.object is None:
            return
        col.operator("scene.ce_rig_setup", text="Physics Setup", icon='ARMATURE_DATA')
        col.operator("scene.ce_rig_reset", text="Reset", icon='FILE_REFRESH')
        col.prop(bpy.context.object.Ce_props, "animated", text="Animated")
        col.prop(bpy.context.object.Ce_props, "live", text="Live")
                
        col.label(text="Forces")
        col.prop(bpy.context.object.Ce_props, "gravity", text="Gravity")
        col.prop(bpy.context.object.Ce_props, "velocity", text="Velocity")
        col.prop(bpy.context.object.Ce_props, "angular_force", text="Angular Force")
                
        col.label(text="Solve Iterations")
        col.prop(bpy.context.object.Ce_props, "linear_iters", text="Linear")
        col.prop(bpy.context.object.Ce_props, "angular_iters", text="Angular")
        col.prop(bpy.context.object.Ce_props, "linear_post_iters", text="Lin Post")
        col.prop(bpy.context.object.Ce_props, "sub_frames", text="sub frame")

        col.label(text="Future")
        col.prop(bpy.context.scene.Ce_props, "test", text="Test Callback")
        
        if bpy.context.object is not None:
            if bpy.context.object.type == "ARMATURE":
                col.label(text="bone targets")
                bone = bpy.context.active_pose_bone
                if bone:
                    col.label(text="Mass")
                    col.prop(bone.Ce_props, "head_mass", text="Mass Head")
                    col.prop(bone.Ce_props, "tail_mass", text="Mass Tail")
                    col.operator("scene.ce_reset_mass", text="Reset Mass")
                    
                    col.label(text="Friction")
                    col.prop(bone.Ce_props, "head_friction", text="Friction Head")
                    col.prop(bone.Ce_props, "tail_friction", text="Friction Tail")
                    
                    col.label(text="Influence Targets")
                    col.operator("scene.ce_make_influence_target", text="Head Target", icon='OUTLINER_DATA_ARMATURE')
                    col.operator("scene.ce_make_influence_target_tail", text="Tail Target", icon='OUTLINER_DATA_ARMATURE')
                    #col.prop(bone.Ce_props, "target", text="Influence Target")
                    
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


        
        col.label(text="Influence Target")
        if bpy.context.object is not None:
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


def unregister():
    # classes

    msg = "I guess you don't love me anymore . Goodbye cruel world. I die!"
    bpy.context.window_manager.popup_menu(U.oops, title=msg, icon='GHOST_ENABLED')

    from bpy.utils import unregister_class

    for cls in reversed(classes):
        unregister_class(cls)


if __name__ == "__main__":
    register()
