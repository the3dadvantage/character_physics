import importlib
from character_physics import ui as UI
importlib.reload(UI)

print("One day kittens will rule all. Meow ah ha ha!")
bl_info = {
    "name": "Character Physics",
    "author": "Rich Colburn, email: CharacterPhysics101@gmail.com",
    "version": (1, 0),
    "blender": (4, 1, 0),
    "location": "View3D >  > Tools",
    "description": "The most powerfol tool in the history of blender addons! \
                    Used for animating characters with physics.",
    "warning": "A man is standing right behind you planning to lick the back \
                of your head. If you turn around he will vanish.",
    "wiki_url": "https://characterphysics.com/",
    "category": '3D View',
}


def oops(self, context):
    """Blender convention for popup error messages."""
    print()
    return


def register():
    UI.register()

def unregister():
    UI.unregister()

#-----------------------------------------------
# |       Commercial Use License Agreement     |
#-----------------------------------------------

# This Commercial Use License Agreement ("Agreement") is entered into
#    by and between CharacterPhysics.com ("Licensor") and the individual
#    or entity agreeing to these terms ("Licensee").

# 1. Grant of License: Licensor hereby grants to Licensee a non-exclusive,
#    non-transferable license to use the Blender addon created by Licensor
#    ("Addon") for commercial purposes.

# 2. Permitted Use: Licensee may use the Addon to create,
#    modify, and distribute derivative works for commercial use.

# 3. Restrictions: Licensee shall not sublicense, sell, or distribute the
#    Addon or any part of it without prior written consent from Licensor.
#    Licensee shall not reverse engineer, decompile, or disassemble the Addon.

# 4. Ownership: Licensor retains all right, title, and interest in and to the
#    Addon, including all intellectual property rights.

# 5. Warranty: The Addon is provided "as is," without warranty of any kind,
#    express or implied. Licensor disclaims all warranties, including but not
#    limited to the implied warranties of merchantability and fitness for a
#    particular purpose.

# 6. Limitation of Liability: In no event shall Licensor be liable for any
#    direct, indirect, incidental, special, exemplary, or consequential damages
#    arising out of the use or inability to use the Addon.

# 7. Governing Law: This Agreement shall be governed by and construed in
#    accordance with the laws of the United States.

# 8. Entire Agreement: This Agreement constitutes the entire agreement between
#    the parties concerning the subject matter hereof and supersedes all prior
#    or contemporaneous agreements, understandings, and negotiations, whether
#    written or oral.

# By using the Addon, Licensee agrees to be bound by the terms and conditions
#    of this Agreement.
