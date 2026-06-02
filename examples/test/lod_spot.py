import Sofa
import numpy as np
from psl import Object, Node, SofaScene
from splib3 import animation
from splib3 import numerics


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
    
    def onStart(self):
        self.renderer.indices.value = self.splats_nodes[0].indices.value

class LODController(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.camera = kwargs.get("camera")
        self.mechanical_state = kwargs.get("mechanical_state")
        self.visuals = kwargs.get("visuals")
        self.distances = kwargs.get("distances")
        self.current_idx = -1

    def onAnimateBeginEvent(self, event):
        cam_pos = np.array(self.camera.position.value[:3])
        obj_pos = np.array(self.mechanical_state.position.value[0][:3])
        distance = np.linalg.norm(cam_pos - obj_pos)

        selected_idx = len(self.distances) - 1 
        for i, dist_threshold in enumerate(self.distances):
            if distance < dist_threshold:
                selected_idx = i
                break
        offset = 0
        for i in range(selected_idx):
            offset += self.visuals[i].indices.value.shape[0]


        if selected_idx != self.current_idx:
            Sofa.msg_info("LOD", f"LOD {selected_idx} (Dist: {distance:.1f})")
            self.visuals[self.current_idx].enable.value = False
            self.visuals[selected_idx].enable.value = True
            self.current_idx = selected_idx



def Container(name, position, splats, distances, camera):
    with Node(name) as spot:
        splats_nodes = []
        with Node("geometry"):
            for i, s_file in enumerate(splats):
                node = Object("PointCloudContainer", name=f"splat_{i}", filename=s_file)
                splats_nodes.append(node)

        with Node("mechanical"):
            state = Object("MechanicalObject", name="state", template="Rigid3", position=position)

        visuals = []
        with Node("visual") as  visual:
            for i, splat_node in enumerate(splats_nodes):
                with Node(f"lod_level_{i}") as lod_node:
                    visual = Object("PointCloudVisualModel", 
                                        name=f"splat_{i}",
                                        indices=splat_node.indices.getLinkPath(), 
                                        geometry=splat_node.getLinkPath(), 
                                        frames=state.position.getLinkPath(),
                                        frameIndices=[0] * len(splat_node.indices.value),
                                        enable=i==0)
                    visuals.append(visual)
                
        spot.addObject(LODController(name="LODManager",
                                     camera=camera, 
                                     mechanical_state=state, 
                                     visuals = visuals, 
                                     distances=distances,))
    return spot



@SofaScene
def createScene(root):
    Object("RequiredPlugin", name="Sofa.PointCloud")
    Object("BackgroundSetting", name="settings", color=[0.1, 0.1, 0.1, 1.0])    

    camera = Object("InteractiveCamera", name="camera", position=[0.911807, -5.2017, -5.83083, 0.918399, 0.173357, -0.0340889, 0.354019], zFar=1000)
    renderer = Object("PointCloudRenderer", name="renderer", camera=camera.getLinkPath())
    Object(animation.AnimationManager(root))

    with Node("Modelling"):
        files = ["splats/spot/spot_r4.ply", "splats/spot/spot_r10.ply", "splats/spot/spot_r18.ply"]
        dist_thresholds = [10, 20, 30] 

        Container("container", 
                  position=[0,0,0,0,0,0,1], 
                  splats=files, 
                  distances=dist_thresholds, 
                  camera=camera)

        
        Container("container2", 
                  position=[0,2,0,0,0,0,1], 
                  splats=files, 
                  distances=dist_thresholds, 
                  camera=camera)