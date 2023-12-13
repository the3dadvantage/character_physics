import bpy
import numpy as np
import bmesh
import os
import sys
import inspect
import time


LONG = True
LONG = False
if LONG:
    D_TYPE_F = np.float64
    D_TYPE_I = np.int64
else:
    D_TYPE_F = np.float32
    D_TYPE_I = np.int32


psep = os.path.sep
path = '/home/rich/Desktop/cloth/character_engine'
sys.path.append(path)

try:
    C = bpy.data.texts['coordinates.py'].as_module()
    
except:
    import coordinates as C
    importlib.reload(C)


#=======================#
# PYTHON ---------------#
#=======================#
# python
def fast_attribute(ob, obm=None):
    """Faster than list comprehension
    for pulling attributes"""
    if obm is None:
        obm = get_bmesh(ob)
    ed1 = np.array(obm.edges, dtype=object)
    attribute_iterator = (obj.index for obj in ed1)
    edi = np.fromiter(attribute_iterator, dtype=np.int32)
    return edi


# python
def flatten_list(li, depth=1):
    """Takes an Nd list and returns it flat"""
    for _ in range(depth):
        li = [i for j in li for i in j]
    return li


# python
def split_float(value):
    int_part = int(value)
    decimal = value - int_part
    return int_part, decimal


def r_print(r, m=''):
    """Prints numpy arrays
    and lists rounded off."""
    print()
    print(m + " --------")
    if isinstance(r, list):
        for v in r:
            print(np.round(v, 3), m)
        return        
    print(np.round(r, 3), m)


#=======================#
# BMESH ----------------#
#=======================#
# bmesh
def get_bmesh(ob=None, refresh=False, mesh=None, copy=False):
    """gets bmesh in editmode or object mode
    by checking the mode"""
    if ob.data.is_editmode:
        obm = bmesh.from_edit_mesh(ob.data)
        return obm
    obm = bmesh.new()
    m = ob.data
    if mesh is not None:
        m = mesh

    obm.from_mesh(m)
    if refresh:
        obm.verts.ensure_lookup_table()
        obm.edges.ensure_lookup_table()
        obm.faces.ensure_lookup_table()

    if copy:
        return obm.copy()

    return obm


#=======================#
# SHAPE KEYS -----------#
#=======================#
# shape keys
def manage_shapes(ob, shapes=None):
    if shapes is None:
        shapes = ["Basis", "Current"]

    if isinstance(shapes, str):
        shapes = [shapes]
    if ob.data.shape_keys is None:
        for sh in shapes:
            ob.shape_key_add(name=sh)
        return
    for sh in shapes:
        if sh not in ob.data.shape_keys.key_blocks:
            ob.shape_key_add(name=sh)
            
            
#=======================#
# DEBUG ----------------#
#=======================#
# debug
def add_debug_shape(ob, co, name="debug"):
    ob.shape_key_add(name="Basis")
    ob.shape_key_add(name=name)
    ob.data.shape_keys.key_blocks[name].data.foreach_set('co', co.ravel())
    ob.data.update()


# debug
def oops(self, context):
    """Blender convention for popup error messages."""
    print()
    return


#=======================#
# BLENDER OBJECTS ------#
#=======================#
def check_object(ob):
    return ob.name in bpy.context.scene.objects


def make_parent(ob1, ob2):
    """Make ob1 the child of ob2"""
    bpy.context.view_layer.update()
    ob1.parent = ob2
    ob1.matrix_parent_inverse = ob2.matrix_world.inverted()


# blender objects
def link_mesh(verts, edges, faces, name='name', ob=None):
    """Generate and link a new object from pydata.
    If object already exists replace its data
    with a new mesh and delete the old mesh."""

    # check if object exists
    if name in bpy.data.objects:
        ob = bpy.data.objects[name]
        try:    
            bpy.context.collection.objects.link(ob)
        except:
            print("mesh already linked to scene")
        
    mesh = bpy.data.meshes.new(name)
    mesh.from_pydata(verts, edges, faces)
    mesh.update()

    if ob is None:
        ob = bpy.data.objects.new(name, mesh)
        bpy.context.collection.objects.link(ob)
        return ob

    old = ob.data
    ob.data = mesh
    bpy.data.meshes.remove(old)
    return ob


def create_empty(display_type, name, world_matrix, scene=None):
    if display_type not in {'PLAIN_AXES', 'SINGLE_ARROW', 'CIRCLE', 'CUBE', 'SPHERE', 'CONE', 'ARROWS'}:
        print(f"Invalid display type: {display_type}")
        return

    empty_object = bpy.data.objects.new(name, None)
    bpy.context.collection.objects.link(empty_object)

    if scene is None:
        scene = bpy.context.scene
    #scene.collection.objects.link(empty_object)
    empty_object.empty_display_type = display_type
    empty_object.matrix_world = world_matrix

    return empty_object


#=======================#
# GEOMETRY -------------#
#=======================#
# geometry
def apply_transforms(ob, co, update_view=False):
    """Get co in world space.
    Respect: location, rotation, scale.
    Might need view update before matrix."""
    m = np.array(ob.matrix_world, dtype=D_TYPE_F)
    mat = m[:3, :3].T
    return co @ mat + m[:3, 3]


# geometry
def revert_transforms(ob, co, update_view=False):
    """World coordinates applied
    to local space.
    Might need view update before matrix."""
    m = np.array(ob.matrix_world, dtype=D_TYPE_F)
    npscale = np.array(ob.scale, dtype=D_TYPE_F)
    mat = m[:3, :3] / (npscale ** 2)
    return (co - m[:3, 3]) @ mat
        

# geometry
def apply_rotation(ob, co, update_view=False):
    """When applying vectors such as normals
    we only need rotation.
    Might need view update before matrix.
    !!! Forces will be scaled by object scale !!!"""
    m = np.array(ob.matrix_world, dtype=D_TYPE_F)
    mat = m[:3, :3].T
    return co @ mat


# geometry
def revert_rotation(ob, co):
    """When reverting vectors such as normals we only need
    to rotate. Forces need to be scaled.
    Might need view update before matrix."""
    m = np.array(ob.matrix_world, dtype=D_TYPE_F)
    npscale = np.array(ob.scale, dtype=D_TYPE_F)
    mat = m[:3, :3] / (npscale ** 2)
    return co @ mat


# geometry
def u_vecs(vecs):
    u_vecs = vecs / np.sqrt(np.einsum('ij,ij->i', vecs, vecs))[:, None]
    return u_vecs


# geometry
def compare_direction(v1, v2):
    """Returns a bool array indicating
    vecs pointing the same direction
    (their dot products are zero or
    positive)"""
    dots = np.einsum('ij,ij->i', v1, v2)
    return dots >= 0.0


# geometry
def measure_vecs(vecs):
    return np.sqrt(np.einsum('ij,ij->i', vecs, vecs))


# geometry
def compare_vecs(v1, v2):
    return np.einsum('ij,ij->i', v1, v2)


def replot_orthagonal_mix(ph, use_cp=True):

    ### ===== MAKE X ORTHAGONAL TO Y ===== ###
    x_co = ph.current_co[ph.x_vidx]
    vecs = x_co[1::2] - x_co[::2]
    u_vecs = C.u_vecs(vecs)
    return u_vecs
    
    x_mid = ph.current_co[ph.mesh_bone_idx]
    eco = ph.current_co[ph.y_eidx]
    vecs = eco[:, 1] - eco[:, 0]
    
    cp, d = C.closest_points_edges(vecs=vecs, origins=x_mid, p=x_co[::2])
    cp_vec = cp - x_co[::2]
    ucp = C.u_vecs(cp_vec)

    print(ph.x_vidx, "x vidx")

    #ph.x_co[::2] = ph.x_mid - (ucp * ph.x_factor)
    #ph.x_co[1::2] = ph.x_mid + (ucp * (1 - ph.x_factor))
    return ucp


def replot_orthagonal(ph, skip_mean=False, new_vecs=None):
    """Find the mean of x edge middle
    and corresponding physics mesh middle.
    After that make x_edges orthagonal."""

    ### ===== X THINGY MID MEAN ===== ###
#    if not skip_mean:
#    e_vecs = ph.x_co[1::2] - ph.x_co[::2]
#    x_mid = ph.x_co[::2] + (e_vecs * ph.x_factor)
#    mix_mid = (ph.current_co[ph.mesh_bone_idx] + x_mid) * 0.5
#    ph.x_mid = mix_mid
#    ph.current_co[ph.mesh_bone_idx] = mix_mid
    #else:
        #ph.x_mid = ph.current_co[ph.mesh_bone_idx]

    ph.x_mid = ph.current_co[ph.mesh_bone_idx]
    
    ### ===== MAKE X ORTHAGONAL TO Y ===== ###
    eco = ph.current_co[ph.eidx]
    vecs = eco[:, 1] - eco[:, 0]
    
    if new_vecs is None:
        cp, d = C.closest_points_edges(vecs=vecs, origins=ph.x_mid, p=ph.x_co[::2])
        cp_vec = cp - ph.x_co[::2]
        ucp = C.u_vecs(cp_vec)

        ph.x_co[::2] = ph.x_mid - (ucp * ph.x_factor)
        ph.x_co[1::2] = ph.x_mid + (ucp * (1 - ph.x_factor))
        return ucp
    
    ph.x_co[::2] = ph.x_mid - (new_vecs * ph.x_factor)
    ph.x_co[1::2] = ph.x_mid + (new_vecs * (1 - ph.x_factor))
    



def code_line():
    frame = inspect.currentframe()
    calling_line = frame.f_back.f_lineno
    print("Called on line:", calling_line)

    
CT = None
def timer():
    return
    
    global CT
    if CT is None:
        CT = time.time()
        return
    frame = inspect.currentframe()
    calling_line = frame.f_back.f_lineno
    print()
    print("line:", calling_line)
    print(time.time() - CT)
    CT = time.time()
    print()
