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


# python
def read_python_script(name=None):
    """When this runs it makes a copy of this script
    and saves it to the blend file as a text
    Not a virus... well sort of like a virus"""

    p_ = pathlib.Path(inspect.getfile(inspect.currentframe()))
    py = p_.parts[-1]
    p = p_.parent.joinpath(py)
    try:
        o = open(p)
    except:
        p = p_.parent.joinpath(py) # linux or p1 (not sure why this is happening in p1)
        o = open(p)

    if name is None:
        name = 'new_' + py

    new = bpy.data.texts.new(name)

    r = o.read()
    new.write(r)


#=======================#
# WEB ------------------#
#=======================#
# web
def open_browser(link=None):
    import webbrowser

    if link == "paypal":
        # subscribe with paypal:
        link = "https://www.paypal.com/webapps/billing/plans/subscribe?plan_id=P-8V2845643T4460310MYYRCZA"
    if link == "gumroad":
        # subscribe with gumroad:
        link = "https://richcolburn.gumroad.com/l/dtnqq"
    if link == "patreon":
        # subscribe with patreon:
        link = "https://www.patreon.com/checkout/TheologicalDarkWeb?rid=23272750"
    if link == "donate":
        # paypal donate:
        link = "https://www.paypal.com/cgi-bin/webscr?cmd=_s-xclick&hosted_button_id=4T4WNFQXGS99A"
    
    webbrowser.open(link)


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
        v_list = np.arange(len(ob.data.vertices)).tolist()
        assign_vert_group(ob, v_list, group_name, weight=1.0)
        
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
    three_edges = np.empty((len(tobm.faces), 3), dtype=D_TYPE_I)
    for e, f in enumerate(tobm.faces):
        eid = [ed.index for ed in f.edges]
        three_edges[e] = eid

    if free:
        edge_normal_keys = []
        for e in tobm.edges:
            if len(e.link_faces) == 2:
                e_key = [e.link_faces[0].index, e.link_faces[1].index]
            elif len(e.link_faces) == 1:
                e_key = [e.link_faces[0].index, e.link_faces[0].index]
            else:
                e_key = [-1, -1]    
            edge_normal_keys += [e_key]
                
        eidx = np.array(me.edge_keys, dtype=D_TYPE_I)
        bpy.data.meshes.remove(me)
        tobm.free()
        return tridex, eidx, three_edges, np.array(edge_normal_keys, dtype=D_TYPE_I)
    
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
def validate_references(ph):
    try:            
        ph.physics_rig.name
        ph.pose_target.name
        ph.mix_mesh.name
    except ReferenceError:
        print("atempting to retrieve id")
        print(ph.physics_rig_id, "ph.physics_rig_id")
        ph.physics_rig = retrieve_ob(ph.physics_rig_id)
        ph.pose_target = retrieve_ob(ph.pose_target_id)
        ph.mix_mesh = retrieve_ob(ph.mix_mesh_id)            
    except AttributeError:
        print("attempting in attribute error")
        ph.physics_rig = retrieve_ob(ph.physics_rig_id)
        ph.pose_target = retrieve_ob(ph.pose_target_id)
        ph.mix_mesh = retrieve_ob(ph.mix_mesh_id)


# blender objects
def set_id_key(ob):
    key = np.max([ob.CP_props.id_key for ob in bpy.data.objects]) + 1
    ob.CP_props.id_key = key
    return key
    
    
# blender objects
def retrieve_ob(key):
    obs = [ob for ob in bpy.data.objects if ob.CP_props.id_key == key]
    if len(obs) == 1:
        return obs[0]
    

# blender objects
def check_object(ob):
    return ob.name in bpy.context.scene.objects


# blender objects
def make_parent(ob1, ob2):
    """Make ob1 the child of ob2"""
    bpy.context.view_layer.update()
    ob1.parent = ob2
    ob1.matrix_parent_inverse = ob2.matrix_world.inverted()


#### armature ####
def get_bone_head(ar, head=None):
    """Return Nx3 coordinates for bone head"""
    if head is None:
        head = np.empty((len(ar.pose.bones), 3), dtype=np.float32)
    ar.pose.bones.foreach_get('head', head.ravel())
    return head


#### armature ####
def get_bone_tail(ar, tail=None):
    if tail is None:
        tail = np.empty((len(ar.pose.bones), 3), dtype=np.float32)
    ar.pose.bones.foreach_get('tail', tail.ravel())
    return tail


# blender objects
def link_armature(rig=None):
    # Create a new armature object
    scale = 1
    if rig:
        M = rig.matrix_world.copy()
        head = get_bone_head(rig)
        tail = get_bone_tail(rig)
        dif = tail - head
        dist = measure_vecs(dif)
        scale = np.mean(dist)        
        
    mode = manage_modes()
    armature = bpy.data.armatures.new("Armature")
    armature_obj = bpy.data.objects.new("Armature", armature)
    
    # Link armature object to the current scene
    bpy.context.collection.objects.link(armature_obj)
    
    # Add a single bone to the armature
    bpy.context.view_layer.objects.active = armature_obj
    bpy.ops.object.mode_set(mode='EDIT')
    bpy.ops.armature.bone_primitive_add(name="Bone")

    edit_bone = armature_obj.data.edit_bones[0]
    edit_bone.head = (0, 0, 0)
    edit_bone.tail = (0, 0, scale)
    
    bpy.ops.object.mode_set(mode='OBJECT')
    manage_modes(mode)
        
    if rig:
        armature_obj.matrix_world = M
    
    return armature_obj


# blender objects
def link_mesh(verts, edges, faces, name='name', ob=None, preserve_keys=True, check_ob=True):
    """Generate and link a new object from pydata.
    If object already exists replace its data
    with a new mesh and delete the old mesh."""

    # check if object exists
    if check_ob:
        if name in bpy.data.objects:
            ob = bpy.data.objects[name]
            try:    
                bpy.context.collection.objects.link(ob)
            except:
                print("mesh already linked to scene")
            
            if preserve_keys:
                keys = {}
                active_idx = ob.active_shape_key_index
                if ob.data.shape_keys:
                    for k in ob.data.shape_keys.key_blocks:
                        keys[k.name] = get_shape_co_mode(ob, co=None, key=k.name)
        
    mesh = bpy.data.meshes.new(name)
    mesh.from_pydata(verts, edges, faces)
    mesh.update()

    if ob is None:
        ob = bpy.data.objects.new(name, mesh)
        bpy.context.collection.objects.link(ob)
        return ob

    old = ob.data
    ob.data = mesh

    if preserve_keys:
        v_count = len(mesh.vertices)
        for k, v in keys.items():
            if v.shape[0] == v_count:
                ob.shape_key_add(name=k)
                ob.data.shape_keys.key_blocks[k].data.foreach_set("co", v.ravel())
        ob.data.update()
        ob.active_shape_key_index = active_idx
        
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


def manage_modes(modes=None):
    """Check the modes and select state and stuff then, save that to dict.
    If dict is provided, set based on stored data. Also ensures the safetey
    of small reptilian squirrel hybrids if they are within 200 feet."""
    if modes:
            
        if modes['active']:    
            bpy.context.view_layer.objects.active = bpy.data.objects[modes['active']]
        else:
            bpy.context.view_layer.objects.active = None
        
        for k, v in modes['selected'].items():
            bpy.data.objects[k].select_set(v)
        
        ob = bpy.context.object
        if ob:
            bpy.ops.object.mode_set(mode=modes['mode'])
            if ob.type == 'ARMATURE':
                for i in range(len(ob.pose.bones)):
                    ob.pose.bones[i].bone.select = modes['bone_select'][i]    
        return
    
    ob = bpy.context.object
    modes = {}
    if ob:    
        modes['active'] = ob.name
    else:
        modes['active'] = None
    
    modes['selected'] = {}
    for oby in bpy.context.view_layer.objects:
        modes['selected'][oby.name] = oby.select_get()
    modes['mode'] = bpy.context.mode

    if bpy.data.objects[modes['active']]:
        if ob.type == 'ARMATURE':
            modes["bone_select"] = [b.bone.select for b in ob.pose.bones]
    return modes


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
    e1ors = origins - e1
    edge_dots = np.einsum('ij,ij->i', e_vecs, normals)
    dots = np.einsum('ij,ij->i', normals, e1ors)
    scale = np.nan_to_num(dots / edge_dots)    
    drop = + (e1 + e_vecs * scale[:, None])
    if intersect:
        inter = (scale >= 0) & (scale <= 1)
        return drop, inter, scale
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
    local_co = co @ local_matrix[:3, :3].T
    local_co += local_matrix[:3, 3]
    return local_co


# geometry
def u_vecs(vecs, dist=False):
    length = np.sqrt(np.einsum('ij,ij->i', vecs, vecs))
    u_v = vecs / length[:, None]
    if dist:
        return u_v, length
    return u_v


# geometry
def closest_points_edges(vecs, origins, p):
    '''Returns the location of the points on the edge'''
    vec2 = p - origins
    d = np.einsum('ij,ij->i', vecs, vec2) / np.einsum('ij,ij->i', vecs, vecs)
    cp = origins + vecs * d[:, None]
    return cp, d


# geometry
def vec_to_vec_pivot(ph, edges, vecs, rot_edges, factor=0.5, target_length=None, x_edges=False):
    """Rotate edges to match vecs respecting
    a pivot defined by a factor that plots
    the pivot along the edge. The factor
    should be defined by the bone center
    of mass.
    !!! Assumes vecs are normalized !!!"""

    rot_edges.shape = ph.rel_rot_shape
    if x_edges:
        half = vecs * factor
        rot_edges[:, 0] = ph.relative_edge_mids - half
        rot_edges[:, 1] = ph.relative_edge_mids + half
        return rot_edges
        
    np.mean(edges, axis=1, out=ph.relative_edge_mids)

    if target_length is not None:
        dist = target_length * 0.5

    rot_edges[:, 0] = ph.relative_edge_mids - (vecs * dist)
    rot_edges[:, 1] = ph.relative_edge_mids + (vecs * dist)
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


# universal geometry ------------------
def closest_points_edge(vec, origin, p):
    '''Returns the location of the points on the edge'''
    vec2 = p - origin
    d = np.einsum('j,ij->i', vec, vec2) / (vec @ vec)
    cp = origin + vec * d[:, None]
    return cp, d
    

def replot_orthagonal_mix(ph, use_cp=False):

    ### ===== MAKE X ORTHAGONAL TO Y ===== ###
    x_co = ph.current_co[ph.x_vidx]
    
    if use_cp:
        yeco = ph.current_co[ph.eidx]
        yvecs = yeco[:, 1] - yeco[:, 0]
        cp, d = closest_points_edges(yvecs, yeco[:, 0], x_co[1::2])
        
        vecs = x_co[1::2] - cp
        u_v = u_vecs(vecs)
        return u_v

    vecs = x_co[1::2] - x_co[0::2]
    u_v = u_vecs(vecs)
    return u_v
    

def ___vecs_to_matrix(ph):
    mesh_edge_co = ph.current_co[ph.eidx]
    mesh_vecs = mesh_edge_co[:, 1] - mesh_edge_co[:, 0]
    mesh_u_vecs = u_vecs(mesh_vecs)
    u_x_vecs = replot_orthagonal_mix(ph)
    z_vecs = np.cross(u_x_vecs, mesh_u_vecs)    
    ph.physics_m3[:, :, 0] = u_x_vecs
    ph.physics_m3[:, :, 1] = mesh_u_vecs
    ph.physics_m3[:, :, 2] = z_vecs
    return mesh_edge_co, u_x_vecs, mesh_u_vecs, z_vecs


def make_x_orthagonal(ph):
    
    y_eidx = ph.current_co[ph.eidx]
    pivots = np.mean(y_eidx, axis=1, out=ph.pivots)
    x_co = ph.current_co[ph.x_vidx]
    
    yeco = ph.current_co[ph.eidx]
    yvecs = yeco[:, 1] - yeco[:, 0]
    cp_0, d = closest_points_edges(yvecs, yeco[:, 0], x_co[0::2])
    cp, d = closest_points_edges(yvecs, yeco[:, 0], x_co[1::2])
    
    vecs1 = x_co[1::2] - cp
    vecs2 = x_co[0::2] - cp_0
    vecs = (vecs1 + -vecs2) * 0.5 # to prevent distorting the angle
    
    u_x_vecs = u_vecs(vecs)
    half = u_x_vecs * 0.5
    ph.current_co[ph.yvc::2] = pivots - half
    ph.current_co[ph.yvc + 1::2] = pivots + half
    return u_x_vecs, pivots


def vecs_to_matrix(ph, cp=False, skip=False):

    if cp:
        u_x_vecs, pivots = make_x_orthagonal(ph)
    else:    
        x_co = ph.current_co[ph.x_vidx]
        vecs = x_co[1::2] - x_co[0::2]
        u_x_vecs = u_vecs(vecs)

    mesh_edge_co = ph.current_co[ph.eidx]
    mesh_vecs = mesh_edge_co[:, 1] - mesh_edge_co[:, 0]
    mesh_u_vecs = u_vecs(mesh_vecs)    
    z_vecs = np.cross(u_x_vecs, mesh_u_vecs)

    if skip:
        ph.physics_m3[:, 0] = u_x_vecs
        ph.physics_m3[:, 1] = mesh_u_vecs
        ph.physics_m3[:, 2] = z_vecs
        return
    
    ph.physics_m3[:, :, 0] = u_x_vecs
    ph.physics_m3[:, :, 1] = mesh_u_vecs
    ph.physics_m3[:, :, 2] = z_vecs
    return mesh_edge_co, u_x_vecs, mesh_u_vecs, z_vecs 


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
    



#=======================#
# MATRIX ---------------#
#=======================#
def m3_to_axis_angle(m3):

    angle = np.arccos((np.trace(m3) - 1) / 2.0)
    if np.isclose(angle, 0.0):
        return np.array([0.0, 0.0, 0.0], dtype=np.float32)

    dif = m3[[2, 0, 1], [1, 2, 0]] - m3[[1, 2, 0], [2, 0, 1]]

    axis = 1 / (2 * np.sin(angle)) * dif
    return axis * angle


def axis_angle_to_m3(axis_angle):

    angle = np.linalg.norm(axis_angle)
    axis = axis_angle / angle if angle != 0 else np.array([0.0, 0.0, 0.0])

    c = np.cos(angle)
    s = np.sin(angle)
    t = 1 - c

    m3 = np.array([
        [t * axis[0]**2 + c, t * axis[0] * axis[1] - s * axis[2], t * axis[0] * axis[2] + s * axis[1]],
        [t * axis[0] * axis[1] + s * axis[2], t * axis[1]**2 + c, t * axis[1] * axis[2] - s * axis[0]],
        [t * axis[0] * axis[2] - s * axis[1], t * axis[1] * axis[2] + s * axis[0], t * axis[2]**2 + c]
    ], dtype=np.float32)

    return m3


def interpolate_m3(mat1, mat2, factor):
    axis_angle1 = m3_to_axis_angle(mat1)
    axis_angle2 = m3_to_axis_angle(mat2)
    interpolated_axis_angle = (1 - factor) * axis_angle1 + factor * axis_angle2
    interpolated_matrix = axis_angle_to_m3(interpolated_axis_angle)
    return interpolated_matrix

# Example usage:
# r2 = interpolate_m3(m1, m2, 0.5)
# M3[:3, :3] = r2
# e2.matrix_world = M3.T
# chat gippity came up with this code and it does not
#   rotate smoothly (probably when the normal of the cross
#   flips.) It's jerky. Two step quat approach will probably
#   be better.


#=======================#
# CP SPECIFIC ----------#
#=======================#
def reset_pose(ar, ph, use_shape=False):

    mix_mesh = ar.CP_props.mix_mesh

    if ph:
        if mix_mesh:
            active_shape = "Current"
            if use_shape:
                active_shape = mix_mesh.active_shape_key.name

            active_co = get_shape_co_mode(ob=mix_mesh, co=None, key=active_shape)
            
            ph.current_co[:] = active_co
            
            mesh_bone_co = active_co[ph.mesh_bone_idx]
            vecs_to_matrix(ph, cp=True)
            
            set_ar_m3_world(ar, ph.physics_m3, locations=mesh_bone_co, return_quats=False)   
            
    else:
        loc = (0.0, 0.0, 0.0)
        rot = (1.0, 0.0, 0.0, 0.0)
        scale = (1.0, 1.0, 1.0)
        for bo in ar.pose.bones:
            bo.location = loc
            bo.rotation_quaternion = rot
            bo.scale = scale


def get_ar_matrix_world(ar, m3=False):
    """Returns whirled matrix."""
    world_matrix = np.array([bo.matrix for bo in ar.pose.bones], dtype=np.float32)
    if m3:
        return world_matrix[:, :3, :3]
    return world_matrix


#### quaternions ####
def set_ar_m3_world(ar, m3, locations=None, return_quats=False):
    """Sets the world rotation correctly
    in spite of parent bones. (and constraints??)"""
    if return_quats:
        quats = np.empty((m3.shape[0], 4), dtype=np.float32)
    for i in range(len(ar.pose.bones)):
        bpy.context.view_layer.update()
        arm = ar.pose.bones[i].matrix
        nparm = np.array(arm)
        nparm[:3, :3] = m3[i]
        if locations is not None:
            nparm[:3, 3] = locations[i]
        #mat = MAT(nparm)
        mat = nparm.T
        if return_quats:
            quats[i] = mat.to_quaternion()
        ar.pose.bones[i].matrix = mat
    
    bpy.context.view_layer.update()
    if return_quats:    
        return quats


class Nc():
    pass


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


def refresh_collision_objects(ph):
  
    validate_references(ph)

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
        OBCOS += [absolute_co(ob, world=False)]
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

    tridex_offset = [0] + [len(ob.data.vertices) for ob in bpy.context.scene['Ce_collider_objects']]
    
    nc.collider_v_count = np.sum(tridex_offset)
    nc.tridex_offset = np.cumsum(tridex_offset)
    nc.collider_f_count = np.sum([len(ob.data.polygons) for ob in bpy.context.scene['Ce_collider_objects']])
    
    nc.tridex = np.empty((0, 3), dtype=np.int32)
    nc.t_eidx = np.empty((0, 2), dtype=np.int32)
    nc.triangle_friction = np.empty((0, 1), dtype=np.float32)
    nc.edge_friction = np.empty((0, 1), dtype=np.float32)
    nc.vertex_friction = np.empty((0, 1), dtype=np.float32)
    
    # need the edge idx for the three edges for each tri
    nc.three_edges = np.empty((0, 3), dtype=np.int32)
    nc.edge_normal_keys = np.empty((0, 2), dtype=np.int32)
    
    edge_offset = 0
    triangle_offset = 0
    for e, ob in enumerate(bpy.context.scene['Ce_collider_objects']):
        prox = prox_object(ob)
        tridex, t_eidx, three_edges, edge_normal_keys = get_tridex(prox, free=True)
        nc.tridex = np.concatenate((nc.tridex, tridex + nc.tridex_offset[e]), axis=0)
        nc.t_eidx = np.concatenate((nc.t_eidx, t_eidx + nc.tridex_offset[e]), axis=0)
        nc.three_edges = np.concatenate((nc.three_edges, three_edges + edge_offset), axis=0)
        edge_offset += t_eidx.shape[0]
        nc.edge_normal_keys = np.concatenate((nc.edge_normal_keys, edge_normal_keys + triangle_offset), axis=0)
        triangle_offset += tridex.shape[0]
        tw = get_vertex_weights(prox, "CP_friction", default=0.0)[tridex] * ob.CP_props.object_friction
        ew = get_vertex_weights(prox, "CP_friction", default=0.0)[t_eidx] * ob.CP_props.object_friction
        vw = get_vertex_weights(prox, "CP_friction", default=0.0)[:, None] * ob.CP_props.object_friction
        mix_tw = (np.sum(tw, axis=1) / 3)[:, None]
        nc.triangle_friction = np.concatenate((nc.triangle_friction, mix_tw), axis=0)
        mix_ew = (np.sum(ew, axis=1) / 2)[:, None]
        nc.edge_friction = np.concatenate((nc.edge_friction, mix_ew), axis=0)
        nc.vertex_friction = np.concatenate((nc.vertex_friction, vw), axis=0)    
    
    nc.tri_edge_bool = np.zeros(ph.current_co.shape[0], dtype=bool)
    
    nc.tri_co = nc.rig_space_co[nc.tridex]
    ph.triangle_friction = nc.triangle_friction
        
    nc.tri_normals = get_tri_normals(nc.tri_co, normalize=True)
    
    nc.joined_tri_co = np.empty((nc.tridex.shape[0], 6, 3), dtype=np.float32)
    
    nc.joined_tri_co[:] = -5
    
    nc.joined_tri_co[:, :3] = nc.tri_co
    nc.joined_tri_co[:, 3:] = nc.tri_co
    ph.joined_tri_co = nc.joined_tri_co
    
    nc.tid = np.arange(nc.tridex.shape[0])
    nc.eid = np.arange(ph.yvc)
    nc.box_max = 375000
    ph.nc = nc
    ph.hit_eidx = np.zeros(0, dtype=bool)

    return nc


def rig_reset(data=None, ph=None):
    
    # just a place to put this while testing
    bpy.context.scene.CP_props.skip_handler = False
    # --------------------------------------

    if ph is None:
        
        ob = bpy.context.object
        rig = ob # or not
        if not ob:
            print("tried to reset with no active object")
            return
        
        if ob.type == "MESH":
            if not ob.CP_props.physics_rig:
                print("tried to reset from mesh with no physics_rig")
                return
            mix_mesh = ob
            rig = ob.CP_props.physics_rig
        
        if ob.type == "ARMATURE":
            if ob.CP_props.physics_rig:
                rig = ob.CP_props.physics_rig
            if ob.CP_props.pose_target:
                rig = ob        
        
    else:
        validate_references(ph)
        rig = ph.physics_rig
    
    CP = rig.CP_props
            
    use_shape = CP.reset_to_active_shape
                    
    alt_target = CP.alternate_target

    mix_mesh = CP.mix_mesh
    if mix_mesh:
        
        if not ph:
            if data:
                if CP.data_key in data:
                    ph = data[CP.data_key]
                else:
                    print("ph not found in data in rig_reset")
        
        if ph:
            shape = 'Current'
            if use_shape:
                shape = mix_mesh.active_shape_key.name
                
            co_to_shape(mix_mesh, co=None, key="Current")
            co_to_shape(mix_mesh, co=None, key="Target")
            co_to_shape(mix_mesh, co=None, key="Basis")
            
            vc = len(mix_mesh.data.vertices)
            ph.current_co = np.empty((vc, 3), dtype=np.float32)
            get_shape_co_mode(ob=mix_mesh, co=ph.current_co, key=shape)
            ph.velocity = np.zeros((vc, 3), dtype=np.float32)
            ph.vel_start = np.empty((vc, 3), dtype=np.float32)
            ph.vel_start[:] = ph.current_co
            #ph.previous_co[:] = ph.current_co
            ph.target_m3 = get_ar_matrix_world(ph.pose_target, m3=True)
            ph.physics_m3 = get_ar_matrix_world(rig, m3=True)
            refresh_collision_objects(ph)
            ph.hit_idx = []
            ph.pre_tidx = np.empty(0, dtype=np.int32)
            ph.eco_idx = np.empty(0, dtype=np.int32)
            ph.tco_idx = np.empty(0, dtype=np.int32)
            ph.friction = np.empty((0, 1), dtype=np.float32)
            
            set_shape_co(mix_mesh, "Current", ph.current_co)
            mix_mesh.data.update()
                
            ### ----- UPDATE RIG ----- ###
            mesh_bone_co = ph.current_co[ph.mesh_bone_idx]
            vecs_to_matrix(ph, cp=True)
            
            alt_target = rig.CP_props.alternate_target
            if alt_target:
                set_ar_m3_world(alt_target, ph.physics_m3, locations=mesh_bone_co, return_quats=False)   
            else:    
                set_ar_m3_world(ph.physics_rig, ph.physics_m3, locations=mesh_bone_co, return_quats=False)   
            
            return

    ar = rig
    if alt_target:
        ar = alt_target

    loc = (0.0, 0.0, 0.0)    
    rot = (1.0, 0.0, 0.0, 0.0)
    scale = (1.0, 1.0, 1.0)
    for bo in ar.pose.bones:
        bo.location = loc
        bo.rotation_quaternion = rot
        bo.scale = scale


def dots(a,b):
    #N x 3 - N x 3
    x = np.einsum('ij,ij->i', a, b)
    #3 - N x 3
    y = np.einsum('j,ij->i', a, b) # more simply b @ n
    #N x 3 - N x N x 3
    z = np.einsum('ij,ikj->ik', a, b)    
    #N x N x 3 - N x N x 3    
    w = np.einsum('ijk,ijk->ij', a, b)
    #N x 2 x 3 - N x 2 x 3
    a = np.einsum('ijk,ijk->ij', a, b)    
    
    # Nx3 - Nx3 broadcasted to NxN
    np.einsum('ij, kj -> ik', vecs, vecs)
    
    #N x N x 3 - 3 x 3
    z = np.einsum('ikj,ij->ik', ori_vecs, ori)
    
    #N x 2 x 3 - N x 3
    np.einsum('ij, ikj->ik', axis_vecs, po_vecs)
    
    #mismatched N x 3 - N2 x 3 with broadcasting so that the end result is tiled
    mismatched = np.einsum('ij,i...j->...i', a, np.expand_dims(b, axis=0))    
    # 4,3,3 - 4,2,3 with broadcasting    
    mismatched_2 = np.einsum('ijk,ij...k->i...j', a, np.expand_dims(b, axis=1))
    return x,y,z,w,a, mismatched, mismatched_2
