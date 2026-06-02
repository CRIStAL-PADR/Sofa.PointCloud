import Sofa
from Sofa.Types import RGBAColor
from splib3 import animation
from splib3 import numerics
import math
from psl import SofaScene, SofaPrefab, Object, Node, this_node, set
import os
import SofaRuntime

def Container(name, position, splat):
    with Node(name) as spot:
        with Node("geometry") as geometry:
            Object("PointCloudContainer", name="splats", filename=splat)
            
        with Node("mechanical") as mechanical:
            Object("MechanicalObject", name="state", template="Rigid3", 
                                       position=position)
            
        with Node("visual"):
            geometry.splats.init()
            indices = [0]*len(geometry.splats.indices.value)
            Object("PointCloudVisualModel", name="renderer", 
                                        indices=geometry.splats.indices.linkpath, 
                                        geometry=geometry.splats.linkpath, 
                                        frames=mechanical.state.position.linkpath,
                                        frameIndices=indices)
    return spot


def OcTree(name, position, splats, distances):
    with Node(name) as quad:
        with Node("LOD") as lod:
            Object("PointCloudOctree", name="octree", 
                    filenames=splats,
                    distances=distances,
                    maxSplats=100)

        with Node("mechanical") as mechanical:
            Object("MechanicalObject", name="state", template="Rigid3", 
                                       position=position)
        
        with Node("visual"):
            lod.octree.init()
            indices = [0]*len(lod.octree.indices.value)
            Object("PointCloudOctreeVisualModel", name="renderer", 
                            indices=lod.octree.indices.linkpath,
                            geometry=lod.octree.linkpath,
                            frames=mechanical.state.position.linkpath,
                            frameIndices=indices)
    return quad



@SofaScene
def createScene(root):
    """Demonstrate the control of a point cloud by a sofa rigid Frame
    """  
    useOcTree = True
    Object("RequiredPlugin", name="Sofa.PointCloud")
    Object("BackgroundSetting", name="settings", color=[0.0,0.0,0.0,1.0])    
    Object("InteractiveCamera", name="camera", position=[0, 0, 1, 0, 0, 0, 1], computeZClip=False, zFar=1000)
    if useOcTree:
        Object("PointCloudOctreeRenderer", name="renderer", camera=root.camera.linkpath)    
    else:
        Object("PointCloudRenderer", name="renderer", camera=root.camera.linkpath)

    Object(animation.AnimationManager(root))
    with Node("Modelling"):
        if useOcTree:
            OcTree("ocTree", position=[0,0,0,0,0,0,1], splats=["splats/garden/garden.ply", "splats/garden/garden_r2.ply", "splats/garden/garden_r4.ply", "splats/garden/garden_r8.ply", "splats/garden/garden_r16.ply"], distances=[10, 15, 50, 100, 1000])

        else:
            Container("container", position=[0,0,0,0,0,0,1], splat="splats/garden/garden_r4.ply")

