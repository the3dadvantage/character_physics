import importlib
from character_physics import ui as UI
importlib.reload(UI)

print("One day kittens will rule all. Meow ah ha ha!")
bl_info = {
    "name": "Character Physics",
    "author": "Rich Colburn, email: characterPhysics101@gmail.com",
    "version": (1, 0),
    "blender": (4, 0, 0),
    "location": "View3D >  > Tools",
    "description": "The most powerfol tool in the history of blender addons! \
                    Used for animating characters with physics.",
    "warning": "A man is standing right behind you planning to lick the back \
                of your head. If you turn around he will vanish.",
    "wiki_url": "",
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
