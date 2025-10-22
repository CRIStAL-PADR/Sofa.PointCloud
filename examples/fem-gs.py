import os
import SofaRuntime
p = os.path.abspath( os.path.join(os.getcwd(), "assets") )
SofaRuntime.DataRepository.addFirstPath(p)  

import Sofa
from Sofa.Types import RGBAColor
from splib3 import animation
from psl import SofaScene, SofaPrefab, Object, Node, this_node, set
from skinning import Skinning

import random


@SofaScene
def createScene(root):
    Object("VisualStyle", displayFlags="showBehaviorModels showForceFields")  
    with Node("Settings"):
        Object("BackgroundSetting", name="settings", color=[0.0,0.0,0.0,1.0])    
        with Node("Plugins"):
            Object("RequiredPlugin", name="Sofa.PointCloud")
        Object(animation.AnimationManager(root))

    Object("InteractiveCamera", name="camera", computeZClip=True, zFar=10000)    

    with Node("Modelling") as modelling:
        with Node("Beam") as beam:
            Object("RegularGridTopology", name="grid", min="-5 -5 0", max="5 5 40", n="5 5 20")
            Object("MechanicalObject", template="Vec3", translation="11 0 0")
            Object("TetrahedronSetTopologyContainer", name="topology")
            Object("TetrahedronSetTopologyModifier", name="modifier")
            Object("TetrahedronSetGeometryAlgorithms", template="Vec3", name="GeomAlgo")
            Object("Hexa2TetraTopologicalMapping", input="@grid", output="@topology")

            Object("DiagonalMass", massDensity="0.2")
            Object("TetrahedronFEMForceField", name="FEM", youngModulus="1000", poissonRatio="0.4", computeGlobalMatrix="false",
                                               method="large", computeVonMisesStress="1", showVonMisesStressPerNodeColorMap="true")
            Object("BoxROI", template="Vec3", name="box_roi", box="-6 -6 -1 50 6 0.1", drawBoxes="1")
            Object("FixedProjectiveConstraint", template="Vec3", indices="@box_roi.indices")
            
            with Node("Frames"):
                Object("MechanicalObject", name="state", template="Rigid", 
                                           position=[[random.random()*4,random.random()*4,float(i)*40/float(60000),0, 0,0,1] for i in range(0, 60000, 1)])
                Object("BarycentricMapping", input_topology=beam.topology.linkpath)

    with Node("GaussianSplat") as gs:
        Object("PointCloudContainer", name="loader", filename="assets/spot-cleaned.ply")
        Object("PointCloudContainer", name="container", filename="assets/spot-cleaned.ply")
        Object("PointCloudTransform", name="transform", input= gs.loader.linkpath,
                                                        output = gs.container.linkpath,
                                                        scale=[30,1,1],
                                                        frame=[20, 10,20,  0.707,0,0,-0.707])
        gs.init()   
        gs.addObject(Skinning(name="skinning", target=modelling.Beam.Frames.state, container=gs.container))

        Object("PointCloudRenderer",  name="renderer", 
                                      indices=gs.container.indices.value, 
                                      geometry=gs.container.linkpath,
                                      frames=gs.skinning.frames.linkpath,
                                      frameIndices=gs.skinning.indices.linkpath,
                                      camera=root.camera.linkpath)
    
    #root.Modelling.Beam.Frames.state.showObject = True
    #root.Modelling.Beam.Frames.state.showObjectScale = 3.0


    with Node("Simulation") as this_node:
        Object("EulerImplicitSolver", name="solver")
        Object("CGLinearSolver", name="solver")
        this_node.addChild(modelling)

