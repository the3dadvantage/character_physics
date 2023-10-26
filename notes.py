
# for overloading operators:
'''
Operator	Magic Method
+	__add__(self, other)
–	__sub__(self, other)
*	__mul__(self, other)
/	__truediv__(self, other)
//	__floordiv__(self, other)
%	__mod__(self, other)
**	__pow__(self, other)
>>	__rshift__(self, other)
<<	__lshift__(self, other)
&	__and__(self, other)
|	__or__(self, other)
^	__xor__(self, other)
'''


'''
Demo:

Snake with wave animation crawling through obstacles
Man doing pushups
Man doing situps
Man doing chinups
Body falling on a cloth sheet or being held up
    by cloth sheet
Placing hand on table
Waving
Throwing objects
Getting hit by car

spider jumping
spider looking at target
spider following a target with friction keeping
the lifted legs moving separate from the down legs
spider with target forcing the local gravity
    down so the spider can crawl on a branch.

Rig a cat. Auto generate phys stuff. Have a single
    button that also does auto weight paint
    Auto generate colliders for the feet
    Auto gen targets for termial bones (bones that have no children)

Meausring the mass at each bone and plugging it into
    the linear stuff could have the effect of making it
    much more realistin when pulling on the hand for example.
    Because the torso will pull more heavily on the arm than
    the arm will pull on the hand

Consider a "chain" property for each target.
    this will control the falloff of influence 
    based on connected bones. Think spider leg
    versus spider body. It might be a matter
    of how strongly the rotation of the chain
    is trying to match the target versus relaxing
    to allow the terminal to point at the target.
    
For targets:
    At least two properties. Location follow and
    rotating to point at target.
    For location, a property for strength will
    add to the velocity vector. Maybe put in
    A clipping value on the distance of the vec
    so at some distance the force will stop increasing
    This way we can have the spider walk at a given speed.
    
Imagine doing emotes like in dark soles trying to animate
    without motion capture. Now try doing it with our system
    Could potentially eliminate the need for motion capture
    in several cases like this.

'''

# ==================== #
# BRAND----------------#
# =====================#

1. 



# grow by link edges
# normalize spring diffs and multiply for weights
# duplicate index for each vert based on number
#   of connected springs to simplify the math
# use weighted average location from
#   the target location of each spring

#a bunch of points going in the same direction
#the final move direction
#compare each move to the final move

#if a bunch of points all point the same
#direction I want to divide by the
#number of points but with respect
#to compounding direction.

#so what if I take the dot of every edge move
#with the final edge move.
#I get a value for all the moves.

#what if I simply move the max dist at each
#point?


#    so what if I get the basic verlet move
#    then I take the dot of all the moves
#    against the verlet move.
#    
#    
#    so like if there were five that move
#    in basically the same direction I want...
#    
#    could move the point based on
#    the average total distance it
#    needs to move... so get the direction,
#    normalize it, then multiply by the
#    average distance of all the points.
    
'''
I could get a quaternion from the movement of an edge...
basically the move vec, the edge and the cross of those
so... start loc of points on both ends...
end loc of points on both ends.
arccos of the angle between them.
cross of move and edge gives axis
arccos gives quat w
can apply rot around center of mass.
can track edge move as linear vel

I can move the bone relative to the edge taking
the bone center mass and related edge center mass
then have a force to make the bone match the edge
rot by comparing the vecs from one to the other...
'''


'''
so what if I keep track of linear velocity at the center of
mass of each bone?
rotational forces happen then we iterate with stretch solve
and rotational back and forth right?

# def get_pose():
so is everything a quaternion force? No because we will have
linear velocity of bones. maybe of head and tail?
maybe a good starting place would be to do angular velocity
how do I set the rotation of a child bone ahhhh!!!!

if I want the linear effects to have an impact on the
rotation force... would it be as simple as taking
the linear force direction with the cross product of the
bone's' y axis for the quaternion rotation axis?
I could then get the angle of the linear force
against the y axis vector for the magnitude of
the quaternion.
'''

# the non-rotated status of a quat is based on its
#   rotation relative to the edit bones relationship to
#   its parent.
# !!! the bones of the slave rig all need an edit mode
#   !!! rotation that's neutral (so pointing up) in order
#   !!! to use the basic get_quat without matrix.to_quaternion()


#print(tail)
#so what if I used a combination of
#matrix and quat.
#I could rotate the 3x3 by the quat using
#slerping. Then I take that location (a fraction of the move)
#and target it from the 3x3 of a bone end.

#So... the head and tail of each bone
#gets its own matrix.

#the connected bones will push that
#matrix and rotate it around.


#the target pose will contain a matrix relative
#to each bone


#each bone at each target pose will be
#trying to get to a particular rotation
#relative to connected bones.

#So I generate a target rotation
#relative 


#so... do I want the relative quaternions or the world ones...
#if I do world I'll have to do quat subtract between bones to get
#the difference.
#if I just use the parent child relationship I'll already have the relative
#so like if a bone is at zero from it's parent, the parent is influencing it
#toward it's own global quaternion which will be applied to the slave rig


#When I apply a rotation to a bone I think I need to rotate the bone
#with the origin in the center of mass.
#The linear solve will then apply forces that will counter the rotation.

    # notes:
    # I can store angular velocity as a quaternion.
    # Need to be able to multiply the quat velocity
    #   by a decimal to reduce the size of the quaternion.
    # When it's applied a standard quat is added to current
    #   rotation so... need a partial quat.
