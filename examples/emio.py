import sys
import os
sys.path.append(os.path.dirname(os.path.realpath(__file__))+"/emio")
import Sofa
from Sofa.Types import RGBAColor
from splib3 import animation
from splib3 import numerics
import math
from psl import SofaScene, SofaPrefab, Object, Node, this_node, set
import parts.emio
import SofaRuntime 

@SofaScene
def createScene(root):
    SofaRuntime.DataRepository.addFirstPath("assets/")

    Object("RequiredPlugin", name="Sofa.PointCloud")

    Object("BackgroundSetting", name="settings", color=[0.0,0.0,0.0,1.0])    
    Object("InteractiveCamera", name="camera", computeZClip=False, zFar=1000)    

    Object(animation.AnimationManager(root))

    with Node("Modelling"):
        #with Node("Emio-compilance") as this_node:
        #    this_node.addChild( parts.emio.Emio() ) 

        def Emio(name):
            with Node(name) as emio:
                with Node("geometry") as geometry:
                    Object("PointCloudContainer", name="loader", filename="assets/emio-test1.ply")

                with Node("mechanical") as mechanical:
                    Object("MechanicalObject", name="state", template="Rigid3", 
                                               position=[[.0,0.0,0.0, 0.0,0.0,0.0,1.0]]) 

                with Node("renderer"):
                    Object("PointCloudRenderer", 
                                        name="renderer", 
                                        indices=geometry.loader.indices.value, 
                                        geometry=geometry.loader.linkpath,  
                                        camera=root.camera.linkpath)
            return emio

        with Emio("emio1") as emio1:
            emio1.mechanical.state.showObject = True
   
