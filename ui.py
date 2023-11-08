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


#=======================#
# PROPERTIES -----------#
#=======================#
# object properties -----------
class CePropsObject(bpy.types.PropertyGroup):

    gravity: bpy.props.FloatProperty(
        name="gravity",
        description="The effect of gravity on this object",
        default=-0.01,
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
def rig_setup():
    target_rig = bpy.context.object
    print(target_rig.type, ": Should be an armature")
    if not target_rig.type == 'ARMATURE':
        print("select an armature goofball")
        return
  
    phr = None
    selected = [i for i in bpy.context.selected_objects if i != target_rig]
    if len(selected) == 1:
        phr = selected[0]
    
    physics_rig = bpy.context.object    
    if phr is not None:
        physics_rig = phr

    ph = E.physics(physics_rig, target_rig)
    #print("Warning! Set a global physics object")
    #E.ph = ph
    #look for ph as global in E    
    
    M = physics_rig.matrix_world
    ph.physics_mesh.matrix_world = M
    
    live = physics_rig.Ce_props.live
    animated = physics_rig.Ce_props.animated
    E.install_handler(live=live, animated=animated)

    physics_rig["physics_mesh"] = ph.physics_mesh.id_data
    physics_rig["x_mesh"] = ph.x_mesh.id_data
    physics_rig["target_rig"] = target_rig.id_data
    target_rig["physics_mesh"] = ph.physics_mesh.id_data
    target_rig["x_mesh"] = ph.x_mesh.id_data
    target_rig["physics_rig"] = physics_rig.id_data
    ph.physics_mesh.id_data["target_rig"] = target_rig.id_data
    ph.physics_mesh.id_data["physics_rig"] = physics_rig.id_data
    ph.physics_mesh.id_data["x_mesh"] = ph.x_mesh.id_data
    ph.x_mesh["target_rig"] = target_rig.id_data
    ph.x_mesh["physics_rig"] = physics_rig.id_data
    ph.x_mesh["physics_mesh"] = ph.physics_mesh.id_data
    

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
    
    physics_mesh = None
    target_rig = None
    physics_rig = None
    x_mesh = None
    
    if "physics_mesh" in ob:
        physics_mesh = ob["physics_mesh"]
    if "target_rig" in ob:
        target_rig = ob["target_rig"]    
    if "physics_rig" in ob:
        physics_rig = ob["physics_rig"]
    if "x_mesh" in ob:
        x_mesh = ob["x_mesh"]
    
    if physics_mesh is None:
        physics_mesh = ob
    if target_rig is None:
        target_rig = ob
    if physics_rig is None:
        physics_rig = ob    
    if x_mesh is None:
        x_mesh = ob
            
    reset_pose(target_rig)
    reset_pose(physics_rig)
    
    if x_mesh is not None:
        C.co_to_shape(x_mesh, co=None, key="Current")
        C.co_to_shape(x_mesh, co=None, key="Target")
        C.co_to_shape(x_mesh, co=None, key="Basis")

        vc = len(x_mesh.data.vertices)
        ph.x_co = np.empty((vc, 3), dtype=np.float32)
        C.get_shape_co_mode(ob=x_mesh, co=ph.x_co, key='Current')
    
    if physics_mesh is not None:
        C.co_to_shape(physics_mesh, co=None, key="Current")
        C.co_to_shape(physics_mesh, co=None, key="Target")
        C.co_to_shape(physics_mesh, co=None, key="Basis")
        
        vc = len(physics_mesh.data.vertices)
        ph.current_co = np.empty((vc, 3), dtype=np.float32)
        C.get_shape_co_mode(ob=physics_mesh, co=ph.current_co, key='Current')
        ph.velocity = np.zeros((vc, 3), dtype=np.float32)
        ph.vel_start = np.empty((vc, 3), dtype=np.float32)
        ph.vel_start[:] = ph.current_co
        ph.physics_m3 = A.get_ar_matrix_world(target_rig, m3=True)


class CeRigReset(bpy.types.Operator):
    """Clear location, rotation, and scale of rig"""

    bl_idname = "scene.ce_rig_reset"
    bl_label = "Ce Rig Reset"
    bl_options = {'REGISTER', 'UNDO'}

    def execute(self, context):
        rig_reset()
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
        col.operator("scene.ce_rig_setup", text="Physics Setup", icon='ARMATURE_DATA')
        col.operator("scene.ce_rig_reset", text="Reset", icon='FILE_REFRESH')
        col.prop(bpy.context.object.Ce_props, "animated", text="Animated")
        col.prop(bpy.context.object.Ce_props, "live", text="Live")
                
        col.label(text="Forces")
        col.prop(bpy.context.object.Ce_props, "gravity", text="Gravity")
        col.prop(bpy.context.object.Ce_props, "velocity", text="Velocity")
        col.prop(bpy.context.object.Ce_props, "angular_force", text="Angular Force")

        col.label(text="Future")
        col.prop(bpy.context.scene.Ce_props, "test", text="Test Callback")

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

        col.operator("scene.gltf_test_function", text="Test Function")  # , icon='RIGHTARROW')
        col.label(text="Convert")
        col.operator("scene.gltf_convert", text="GLTF Convert")  # , icon='RIGHTARROW')
        col.operator("scene.gltf_make_markers", text="Markers Only")  # , icon='RIGHTARROW')
        col.prop(bpy.context.scene.Gltf_props, "caliper_scale", text="mm")  # , icon='CON_SIZELIKE')
        col.prop(bpy.context.scene.Gltf_props, "scale_factor", text="Scale Factor")  # , icon='CON_SIZELIKE')
        col.prop(bpy.context.scene.Gltf_props, "subdiv_levels", text="Subdiv Levels")  # , icon='CON_SIZELIKE')

        col.label(text="Cleanup")
        col.operator("scene.gltf_verify_objects", text="Verify Meshes")  # , icon='RIGHTARROW')
        col.prop(
            bpy.context.scene.Gltf_props, "symmetrize_markers", text="Symmetrize Markers"
        )  # , icon='CON_SIZELIKE')
        col.prop(bpy.context.scene.Gltf_props, "marker_names", text="Marker Names")  # , icon='CON_SIZELIKE')
        col.prop(bpy.context.scene.Gltf_props, "show_normals", text="Show Face Orientation")  # , icon='CON_SIZELIKE')
        col.prop(bpy.context.scene.Gltf_props, "self_collide_fix", text="Self Collide Fix")  # , icon='CON_SIZELIKE')
        col.operator("scene.gltf_self_collide_debug", text="SC Debug")  # , icon='RIGHTARROW')

        col.label(text="Developer")
        col.prop(bpy.context.scene.Gltf_props, "make_debug_material", text="Debug Material")  # , icon='CON_SIZELIKE')
        # col.prop(bpy.context.scene.Gltf_props, "debug_mode", text="Debug Mode")
        col.prop(bpy.context.scene.Gltf_props, "debug_view", text="Debug Viewer")  # , icon='CON_SIZELIKE')
        col.prop(bpy.context.scene.Gltf_props, "debug_time", text="Viewer Time")  # , icon='CON_SIZELIKE')
        col.prop(bpy.context.scene.Gltf_props, "batch_count", text="Batch Number")  # , icon='CON_SIZELIKE')
        col.prop(bpy.context.scene.Gltf_props, "debug_view_offset", text="Viewer Offset")  # , icon='CON_SIZELIKE')
        col = layout.column(align=True)
        col.scale_y = 1.5
        msg = "RUN BATCH"
        if bpy.context.scene.Gltf_props.run_batch:
            msg = "PAUSE BATCH"
        col.prop(bpy.context.scene.Gltf_props, "run_batch", text=msg)  # , icon='CON_SIZELIKE')
        col = layout.column(align=True)
        col.scale_y = 1.0
        col.prop(bpy.context.scene.Gltf_props, "render_debug_image", text="Marker Image")  # , icon='CON_SIZELIKE')
        col.prop(bpy.context.scene.Gltf_props, "subdiv_for_markers", text="Marker Subdiv")  # , icon='CON_SIZELIKE')
        col.prop(
            bpy.context.scene.Gltf_props,
            "fix_interior_faces",
            text="Interior Faces Removed",
        )  # , icon='CON_SIZELIKE')

        # baking ---------
        col = layout.column(align=True)
        col.label(text="UV/Bake")
        col.prop(bpy.context.scene.Gltf_props, "keep_uv_maps", text="Keep UV maps")  # , icon='CON_SIZELIKE')
        col.prop(bpy.context.scene.Gltf_props, "update_maps", text="Update gltf map")  # , icon='CON_SIZELIKE')
        # if bpy.context.scene.Gltf_props.run_bakes:
        col.label(text="Maps")
        col.prop(bpy.context.scene.Gltf_props, "diffuse_bake", text="DIFFUSE")  # , icon='CON_SIZELIKE')
        if bpy.context.scene.Gltf_props.diffuse_bake:
            col.prop(
                bpy.context.scene.Gltf_props, "diffuse_size", text="Size"
            )  # , text="DIFFUSE")  # , icon='CON_SIZELIKE')
            col.prop(
                bpy.context.scene.Gltf_props, "diffuse_colorspace", text=""
            )  # , text="DIFFUSE")  # , icon='CON_SIZELIKE')
        col.prop(bpy.context.scene.Gltf_props, "thickness_bake", text="THICKNESS")  # , icon='CON_SIZELIKE')
        if bpy.context.scene.Gltf_props.thickness_bake:
            col.prop(
                bpy.context.scene.Gltf_props, "thickness_size", text="Size"
            )  # , text="THICKNESS")  # , icon='CON_SIZELIKE')
            col.prop(
                bpy.context.scene.Gltf_props, "thickness_samples", text="Samples"
            )  # , text="AO")  # , icon='CON_SIZELIKE')
        col.prop(bpy.context.scene.Gltf_props, "ao_bake", text="AO")  # , icon='CON_SIZELIKE')
        if bpy.context.scene.Gltf_props.ao_bake:
            col.prop(bpy.context.scene.Gltf_props, "ao_size", text="Size")  # , text="AO")  # , icon='CON_SIZELIKE')
            col.prop(
                bpy.context.scene.Gltf_props, "ao_samples", text="Samples"
            )  # , text="AO")  # , icon='CON_SIZELIKE')
        col.prop(bpy.context.scene.Gltf_props, "roughness_bake", text="ROUGHNESS")  # , icon='CON_SIZELIKE')
        if bpy.context.scene.Gltf_props.roughness_bake:
            col.prop(
                bpy.context.scene.Gltf_props, "roughness_size", text="Size"
            )  # , text="ROUGHNESS")  # , icon='CON_SIZELIKE')
        col = layout.column(align=True)
        col.operator("scene.gltf_bake_manual", text="Bake", icon='OUTPUT')
        col.prop(bpy.context.scene.Gltf_props, "run_bakes", text="Bake Auto")  # , icon='CON_SIZELIKE')

        # baking end -----

        col = layout.column(align=True)
        col.label(text="Verify Groups")
        col.operator("scene.gltf_caliper", text="Caliper", icon='NOCURVE')
        col.operator("scene.gltf_left_temple", text="Left Temple", icon='TRIA_LEFT')
        col.operator("scene.gltf_right_temple", text="Right Temple", icon='TRIA_RIGHT')
        col.operator("scene.gltf_frame", text="Frame")  # , icon='RIGHTARROW')
        col.operator("scene.gltf_lenses", text="Lenses")  # , icon='RIGHTARROW')
        col.operator("scene.gltf_select_by_material", text="OUTPUT")  # , icon='RIGHTARROW')

        col = layout.column(align=True)
        col.alert = bpy.context.scene.Gltf_props.explode_view
        col.prop(bpy.context.scene.Gltf_props, "explode_view", text="Explode View")

        col = layout.column(align=True)
        col.label(text="Convert")
        col.prop(bpy.context.scene.Gltf_props, "glb_only", text=".glb only")  # , icon='CON_SIZELIKE')
        col.prop(bpy.context.scene.Gltf_props, "glb_path", text=".glb path")  # , icon='CON_SIZELIKE')
        col.operator("scene.gltf_save_gltf", text="Save")  # , icon='RIGHTARROW')
        col.operator("scene.gltf_reset", text="Reset")  # , icon='RIGHTARROW')


#=======================#
# REGISTER -------------#
#=======================#
classes = (
    PANEL_PT_Character_Engine,
    CePropsObject,
    CePropsScene,
    CeRigSetup,
    CeRigReset,
)


def register():
    # classes
    from bpy.utils import register_class

    for cls in classes:
        register_class(cls)
    bpy.types.Scene.Ce_props = bpy.props.PointerProperty(type=CePropsScene)
    bpy.types.Object.Ce_props = bpy.props.PointerProperty(type=CePropsObject)


def unregister():
    # classes

    msg = "I guess you don't love me anymore . Goodbye cruel world. I die!"
    bpy.context.window_manager.popup_menu(U.oops, title=msg, icon='GHOST_ENABLED')

    from bpy.utils import unregister_class

    for cls in reversed(classes):
        unregister_class(cls)


if __name__ == "__main__":
    register()
