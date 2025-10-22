import Sofa
from Sofa.Types import RGBAColor
from splib3 import animation
from splib3 import numerics
import math
from psl import SofaScene, SofaPrefab, Object, Node, this_node, set
import os
import SofaRuntime
p = os.path.abspath( os.path.join(os.getcwd(), "assets") )
SofaRuntime.DataRepository.addFirstPath(p)  

class DofManipulator(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.target = kwargs.get("target", None)
        #self.addData("frame", value=[0,0,0,0,0,0,1], default=[0,0,0,0,0,0,1], help="", group="Gizmos", type="Rigid3::Coord")
        
    def draw(self, visual_params):
        dt=visual_params.getDrawTool()
        #dt.drawFrames(self.frame, [0.1,0.1,0.1])

    def onKeypressedEvent(self, event):
            print(event["key"])
            self.text += 1

            def myAnimate1(target, factor, terminationDelay=0):
                angle=factor*3.14/2.0
                ref = target.mechanical.state.rest_position.value
                with target.mechanical.state.position.writeableArray() as w:
                    print(w)
                    for i in range(w.shape[0]):
                        q = numerics.Quat.createFromAxisAngle([0,1,0],angle+float(i)*angle*0.2)
                        x = numerics.Quat(ref[i,3:])
                        print("=", x)
                        r = numerics.Quat.product(q,x)
                        print(r)
                        w[i,3:]=r

            if self.text == 1:
                animation.animate(myAnimate1, {"target" : self.target}, 1, mode="pingpong")
    

    def onMouseEvent(self, mouse_event):
        if mouse_event["State"] == 1:
            print("Left click event ", mouse_event)
        elif mouse_event["State"] == 2:
            print("Left released event ", mouse_event)


class SpotController(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.spot = kwargs.get("spot", None)

    def onIdleEvent(self, params):
        pass

    def onKeypressedEvent(self, params):
        print("KEY MOVE ", params, self.spot.linkpath)
        self.spot.mechanical.state.position.value = [[1,0,0,0,0,0,1]]

@SofaScene
def createScene(root):
    Object("RequiredPlugin", name="Sofa.PointCloud")

    Object("BackgroundSetting", name="settings", color=[0.0,0.0,0.0,1.0])    
    Object("InteractiveCamera", name="camera", computeZClip=False, zFar=1000)    

    Object(animation.AnimationManager(root))

    with Node("Modelling"):
        def Spot(name):
            with Node(name) as Spot:
                with Node("geometry") as geometry:
                    Object("PointCloudContainer", name="loader", filename="assets/spot-cleaned.ply")
                    
                    Object("PointCloudContainer", name="pc1", filename="assets/spot-cleaned.ply")

                    Object("PointCloudTransform", name="transform", input=geometry.loader.linkpath, output=geometry.pc1.linkpath)


                with Node("mechanical") as mechanical:
                    Object("MechanicalObject", name="state", template="Rigid3")
                    
                with Node("renderer"):
                    geometry.pc1.init()
                    #indices = [1]*len(geometry.pc1.indices.value)
                    #for g,v in groups.items():
                    #    for index in v:
                    #        indices[index] = g

                    Object("PointCloudRenderer", 
                                        name="renderer", 
                                        indices=geometry.pc1.indices.value, 
                                        geometry=geometry.pc1.linkpath, 
                                        #frames=mechanical.state.position.linkpath,
                                        #frameIndices=indices, 
                                        camera=root.camera.linkpath)

            return Spot

        with Spot("spot1") as spot1:
            spot1.addObject(DofManipulator(name="manipulator", target=spot1))

        if False:
            with Node("Environment"):
                with Node("geometry") as geometry:
                    Object("PointCloudContainer", name="pc", filename="assets/room-ground.ply")

                with Node("renderer"):
                    Object("PointCloudRenderer", 
                                        name="renderer", 
                                        indices=geometry.pc.indices.value, 
                                        geometry=geometry.pc.linkpath, 
                                        frames=[[0,0,0, 0,1,0,1]],
                                        framesIndices=indices, 
                                        camera=root.camera.linkpath)

    if False:
        with Node("Simulation") as this_node:
            Object("EulerImplicitSolver")    
            Object("SparseLDLSolver")
            this_node.addChild(Spot)
