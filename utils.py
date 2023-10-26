import bpy
import numpy as np
import bmesh


#=======================#
# PYTHON ---------------#
#=======================#
# python
def flatten_list(li, depth=1):
    """Takes an Nd list and returns it flat"""
    for _ in range(depth):
        li = [i for j in li for i in j]
    return li


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


#=======================#
# GEOMETRY -------------#
#=======================#
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
