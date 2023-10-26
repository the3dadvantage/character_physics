import bpy
import numpy as np
from mathutils import Matrix as MAT
D_TYPE_F = np.float32


def set_matrix(ob, m3=None, location=None):
    npm = np.array(ob.matrix_world)
    if m3 is not None:
        npm[:3, :3] = m3
    if location is not None:
        npm[:3, 3] = location
    mat = MAT(npm)
    ob.matrix_world = mat


def get_co(ob):
    """Returns Nx3 cooridnate set as numpy array"""
    co = np.empty((len(ob.data.vertices), 3), dtype=D_TYPE_F)
    ob.data.vertices.foreach_get("co", co.ravel())
    return co


def co_to_shape(ob, co=None, key="current"):
    if co is None:
        co = get_co(ob)
    ob.data.shape_keys.key_blocks[key].data.foreach_set('co', co.ravel())
    ob.data.update()


def get_co_edit(ob, ar=None, key='current'):
    ob.update_from_editmode()
    if ar is None:
        c = len(ob.data.vertices)
        ar = np.empty((c, 3), dtype=np.float32)
    ob.data.shape_keys.key_blocks[key].data.foreach_get('co', ar.ravel())
    return ar


def get_shape_co_mode(ob=None, co=None, key='current'):
    """Edit or object mode"""

    if ob.data.is_editmode:
        ob.update_from_editmode()

    if co is None:
        co = np.empty((len(ob.data.vertices), 3), dtype=np.float32)
        
    ob.data.shape_keys.key_blocks[key].data.foreach_get('co', co.ravel())
    return co


def get_shape_co(ob, shape='Key_1', co=None):
    if co is None:
        co = np.empty((len(ob.data.vertices), 3), dtype=D_TYPE_F)
    ob.data.shape_keys.key_blocks[shape].data.foreach_get('co', co.ravel())
    return co


def set_shape_co(ob, shape, co):
    ob.data.shape_keys.key_blocks[shape].data.foreach_set('co', co.ravel())


def get_bmesh_co(obm, co=None):
    """To get the grabbed location of a vertex we
    need to get the data from the bmesh coords."""
    if co is None:
        return np.array([v.co for v in cloth.obm.verts], dtype=np.float32)

    for i in range(co.shape[0]):
        co[i] = obm.verts[i].co


def get_edge_centers(co, eidx, scale=0.5):
    """Start and end of edge
    has to match head and tail
    of bone if center of mass
    is not at 0.5."""
    head = co[eidx[:, 0]]
    vec = (co[eidx[:,1]] - head) * scale
    return head + vec
