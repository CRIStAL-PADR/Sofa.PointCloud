import Sofa
from Sofa.Types import RGBAColor
from splib3 import animation
from splib3 import numerics
import math
from psl import SofaScene, SofaPrefab, Object, Node, this_node, set
import os
import SofaRuntime

class DofManipulator(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.target = kwargs.get("target", None)
        self.direction = kwargs.get("direction", 1)

        def myAnimate1(target, factor):
            angle=self.direction*factor*3.14*2.0
            ref = target.mechanical.state.rest_position.value
            with target.mechanical.state.position.writeableArray() as w:
                for i in range(w.shape[0]):
                    q = numerics.Quat.createFromAxisAngle([0,0,1],angle+float(i)*angle*0.2)
                    x = numerics.Quat(ref[i,3:])
                    r = numerics.Quat.product(q,x)
                    w[i,3:]=r

        animation.animate(myAnimate1, {"target" : self.target}, 1, mode="loop")

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
                    maxSplats=100,
                    showLod=True)

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
            files = ["splats/spot/spot_r4.ply", "splats/spot/spot_r10.ply", "splats/spot/spot_r18.ply"]
            dist_thresholds = [10, 30, 1000] 
            with OcTree("ocTree", position=[0,0,0,0,0,0,1], splats=files, distances=dist_thresholds) as octree:
                octree.addObject(DofManipulator(name="manipulator", direction=1, target=octree))

        else:
            with Container("container", position=[0,0,0,0,0,0,1], splat="splats/spot/spot.ply") as conatainer:
                conatainer.addObject(DofManipulator(name="manipulator", direction=1, target=conatainer))

