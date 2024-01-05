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


#=======================#
# PYTHON ---------------#
#=======================#
# python
def code_line():
    """Prints the line of code where it's called"""
    frame = inspect.currentframe()
    calling_line = frame.f_back.f_lineno
    print("Called on line:", calling_line)


# python
def numpy_array_to_string(numpy_array):
    return ','.join(map(str, numpy_array.ravel()))


# python
def string_to_numpy_array(string, shape=None, dtype=np.float32):
    ar = np.fromstring(string, dtype=dtype, sep=',')
    if shape is None:
        shape = (ar.shape[0]//3, 3)
    ar.shape = shape
    return ar


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


# python
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


### ==================== ###
#      VERTEX GROUPS       #
### ==================== ###
# vertex groups
def assign_vert_group(ob, v_list, name, weight=1.0):
    """Does what you might imagine
    (makes me a sandwich)."""
    if name not in ob.vertex_groups:
        nvg = ob.vertex_groups.new(name=name)
    else:
        nvg = ob.vertex_groups[name]
    nvg.add(v_list, weight, 'ADD')


# vertex groups
def get_vertex_weights(ob, group_name, default=1.0):
    """Get weights of the group. if it's not
    in the group set it to default"""
    if group_name not in ob.vertex_groups:
        ob.vertex_groups.new(name=group_name)

    vertex_group = ob.vertex_groups[group_name]
    all_vertices = range(len(ob.data.vertices))
    weights = []

    for idx in all_vertices:
        try:
            weight = vertex_group.weight(idx)
            weights.append(weight)
        except RuntimeError:
            weights.append(default)

    vertex_weights = np.array(weights, dtype=np.float32)
    return vertex_weights


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


# bmesh
def get_tridex(ob, teidx=False, tmesh=False, tobm=None, refresh=False, cull_verts=None, free=False):
    """Return an index for viewing the
    verts as triangles using a mesh and
    foreach_get.
    Return tridex and triobm"""

    if tobm is None:
        tobm = get_bmesh(ob)
        if cull_verts is not None:
            tobm.verts.ensure_lookup_table()
            cv = [tobm.verts[v] for v in cull_verts]
            bmesh.ops.delete(tobm, geom=cv, context='VERTS')

    bmesh.ops.triangulate(tobm, faces=tobm.faces)
    me = bpy.data.meshes.new('tris')
    tobm.to_mesh(me)
    p_count = len(me.polygons)
    tridex = np.empty((p_count, 3), dtype=D_TYPE_I)
    me.polygons.foreach_get('vertices', tridex.ravel())
    if free:
        tobm.free()
        return tridex
    
    if tmesh:
        return tridex, tobm, me
    if teidx:
        eidx = np.array(me.edge_keys, dtype=D_TYPE_I)
        # clear unused tri mesh
        bpy.data.meshes.remove(me)
        return tridex, eidx, tobm

    # clear unused tri mesh
    bpy.data.meshes.remove(me)

    if refresh:
        tobm.verts.ensure_lookup_table()
        tobm.edges.ensure_lookup_table()
        tobm.faces.ensure_lookup_table()

    return tridex, tobm, None


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


# debug
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


#=======================#
# BLENDER OBJECTS ------#
#=======================#
def check_object(ob):
    return ob.name in bpy.context.scene.objects


# blender objects
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


# blender objects
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
def prox_object(ob):
    """Returns object including modifier and
    shape key effects."""
    dg = bpy.context.evaluated_depsgraph_get()
    return ob.evaluated_get(dg)


# geometry
def absolute_co(ob, ar=None, world=True, prox=None):
    """Get proxy vert coords in world space.
    !!! Also returns prox object !!!
    Might need view update before matrix."""

    if prox is None:    
        prox = prox_object(ob)

    if ar is None:
        count = len(prox.data.vertices)
        ar = np.zeros((count, 3), dtype=D_TYPE_F)
    prox.data.vertices.foreach_get("co", ar.ravel())
    
    if world:    
        return apply_transforms(ob, ar), prox

    return ar


# geometry
def co_to_shape(ob, co=None, key="current"):
    if co is None:
        co = get_co(ob)
    ob.data.shape_keys.key_blocks[key].data.foreach_set('co', co.ravel())
    ob.data.update()


# geometry
def get_shape_co_mode(ob=None, co=None, key='current'):
    """Edit or object mode"""

    if ob.data.is_editmode:
        ob.update_from_editmode()

    if co is None:
        co = np.empty((len(ob.data.vertices), 3), dtype=np.float32)
        
    ob.data.shape_keys.key_blocks[key].data.foreach_get('co', co.ravel())
    return co


# geometry
def get_shape_co(ob, shape='Key_1', co=None):
    if co is None:
        co = np.empty((len(ob.data.vertices), 3), dtype=D_TYPE_F)
    ob.data.shape_keys.key_blocks[shape].data.foreach_get('co', co.ravel())
    return co


# geometry
def set_shape_co(ob, shape, co):
    ob.data.shape_keys.key_blocks[shape].data.foreach_set('co', co.ravel())


# geometry
def get_bmesh_co(obm, co=None):
    """To get the grabbed location of a vertex we
    need to get the data from the bmesh coords."""
    if co is None:
        return np.array([v.co for v in cloth.obm.verts], dtype=np.float32)

    for i in range(co.shape[0]):
        co[i] = obm.verts[i].co


# geometry
def get_co_edit(ob, ar=None, key='current'):
    ob.update_from_editmode()
    if ar is None:
        c = len(ob.data.vertices)
        ar = np.empty((c, 3), dtype=np.float32)
    ob.data.shape_keys.key_blocks[key].data.foreach_get('co', ar.ravel())
    return ar


# geometry
def get_co(ob):
    """Returns Nx3 cooridnate set as numpy array"""
    co = np.empty((len(ob.data.vertices), 3), dtype=D_TYPE_F)
    ob.data.vertices.foreach_get("co", co.ravel())
    return co


# geometry
def get_edge_centers(co, eidx, scale=0.5):
    """Start and end of edge
    has to match head and tail
    of bone if center of mass
    is not at 0.5."""
    head = co[eidx[:, 0]]
    vec = (co[eidx[:,1]] - head) * scale
    return head + vec


# geometry
def slide_points_to_planes(e1, e2, origins, normals, intersect=False):
    '''Takes the start and end of an edge.
    Returns where it intersects the planes with a bool array for the
    edges that pass through the plane'''
    e_vecs = e2 - e1
    e1ors = e1 - origins
    edge_dots = np.einsum('ij,ij->i', e_vecs, normals)
    dots = np.einsum('ij,ij->i', normals, e1ors)
    scale = np.nan_to_num(dots / edge_dots)
    
    drop = (e1ors - e_vecs * scale[:, None]) + origins
    if intersect:
        intersect = (scale < 0) & (scale > -1)
        return drop, intersect, scale
    return drop


# geometry
def inside_triangles(tris, points, margin=0.0):  # , cross_vecs):
    """Checks if points are inside triangles"""
    origins = tris[:, 0]
    cross_vecs = tris[:, 1:] - origins[:, None]

    v2 = points - origins

    # ---------
    v0 = cross_vecs[:, 0]
    v1 = cross_vecs[:, 1]

    d00_d11 = np.einsum('ijk,ijk->ij', cross_vecs, cross_vecs)
    d00 = d00_d11[:, 0]
    d11 = d00_d11[:, 1]
    d01 = np.einsum('ij,ij->i', v0, v1)
    d02 = np.einsum('ij,ij->i', v0, v2)
    d12 = np.einsum('ij,ij->i', v1, v2)

    div = 1 / (d00 * d11 - d01 * d01)
    u = (d11 * d02 - d01 * d12) * div
    v = (d00 * d12 - d01 * d02) * div

    w = 1 - (u + v)
    # !!!! needs some thought
    # margin = 0.0
    # !!!! ==================
    weights = np.array([w, u, v]).T
    check = (u >= margin) & (v >= margin) & (w >= margin)
    return check, weights


# geometry
def get_edge_bounds(edges):
    """Takes an NxNx3 set of coords and returns
    the min and max for each edge/tri"""
    min = np.min(edges, axis=1)
    max = np.max(edges, axis=1)
    return [min, max]


# geometry
def get_tri_normals(tco, normalize=True):
    """"""
    tv1 = tco[:, 1] - tco[:, 0]
    tv2 = tco[:, 2] - tco[:, 0]
    cross = np.cross(tv1, tv2)
    if normalize:    
        return u_vecs(cross)    
    return cross
    

# geometry
def get_vertex_normals(ob):
    """Vertex normals from object using blender
    calculated normals."""
    normals = np.zeros((len(ob.data.vertices), 3), dtype=D_TYPE_F)
    ob.data.vertices.foreach_get('normal', normals.ravel())
    return normals


# geometry
def xyz_world(ob, normalize=False):
    """Get the xyz axis vectors of an object.
    Assumes a scale of 1, 1, 1"""
    xyz = np.array(ob.matrix_world, dtype=D_TYPE_F)[:3, :3].T
    if normalize:
        xyz /= np.array(ob.scale, dtype=np.float32)[:, None]
    return xyz


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
def co_to_matrix_space(co, ob1, ob2):
    """Takes the world coordinates of object
    1 and puts them in object coordinate space"""
        
    local_matrix = np.linalg.inv(ob2.matrix_world) @ np.array(ob1.matrix_world, dtype=np.float32)
    local_co = co @ local_matrix[i][:3, :3].T
    local_co += local_matrix[i][:3, 3]
    return local_co


# geometry
def u_vecs(vecs):
    u_v = vecs / np.sqrt(np.einsum('ij,ij->i', vecs, vecs))[:, None]
    return u_v


# geometry
def closest_points_edges(vecs, origins, p):
    '''Returns the location of the points on the edge'''
    vec2 = p - origins
    d = np.einsum('ij,ij->i', vecs, vec2) / np.einsum('ij,ij->i', vecs, vecs)
    cp = origins + vecs * d[:, None]
    return cp, d


# geometry
def vec_to_vec_pivot(edges, vecs, factor=0.5):
    """Rotate edges to match vecs respecting
    a pivot defined by a factor that plots
    the pivot along the edge. The factor
    should be defined by the bone center
    of mass.
    !!! Assumes vecs are normalized !!!"""
    
    edge_vecs = (edges[:, 1] - edges[:, 0])
    
    rot_edges = np.empty_like(edges)
    vecs_1 = edge_vecs * factor
    vecs_2 = edge_vecs * (1 - factor)
    edge_mid = edges[:, 0] + vecs_1
    dist_1 = measure_vecs(vecs_1)[:, None]
    dist_2 = measure_vecs(vecs_2)[:, None]

    rot_edges[:, 0] = edge_mid - (vecs * dist_1)
    rot_edges[:, 1] = edge_mid + (vecs * dist_2)
    return rot_edges


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
    u_v = u_vecs(vecs)
    return u_v


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
        cp, d = closest_points_edges(vecs=vecs, origins=ph.x_mid, p=ph.x_co[::2])
        cp_vec = cp - ph.x_co[::2]
        ucp = u_vecs(cp_vec)

        ph.x_co[::2] = ph.x_mid - (ucp * ph.x_factor)
        ph.x_co[1::2] = ph.x_mid + (ucp * (1 - ph.x_factor))
        return ucp
    
    ph.x_co[::2] = ph.x_mid - (new_vecs * ph.x_factor)
    ph.x_co[1::2] = ph.x_mid + (new_vecs * (1 - ph.x_factor))
    





    
