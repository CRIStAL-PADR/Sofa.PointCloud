import os
def get(target):
    return os.path.join(os.path.join(os.path.dirname(__file__), "assets"), target)

