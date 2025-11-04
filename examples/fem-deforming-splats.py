from psl import SofaScene, SofaPrefab, Object, Node, this_node, set

import Sofa
import json
        
@SofaPrefab
def Camera(name):
    c = Object("InteractiveCamera", name="state", computeZClip=True, zFar=10000)    
    Object(CameraController(name="controller", camera=c))

class IdentitySkinning(Sofa.Core.Controller):
    """This controller initialze the references frames""" 

    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.addData("indices", value=[0], default=[0], help="size", group="", type="vector<int>")
        self.container = kwargs.get("container")
        self.target = kwargs.get("target")
        self.initializeFrames()
        
    def initializeFrames(self):
        frame_count = len(self.container.positions.value)
        indices = list(range(0, len(self.container.positions.value)))

        self.target.position = [[0,0,0,0,0,0,1]]*len(self.container.positions.value)
        with self.target.position.writeableArray() as w:
            w[:,0:3]  = self.container.positions.value[:,0:3]        
        self.indices = indices

@SofaScene
def createScene(root):
    Object("VisualStyle", displayFlags="showBehaviorModels showForceFields")  
    Object("InteractiveCamera", name="camera", computeZClip=True, zFar=10000)    
    
    with Node("Settings"):
        Object("BackgroundSetting", name="settings", color=[0.0,0.0,0.0,1.0])    
        with Node("Plugins"):
            Object("RequiredPlugin", name="Sofa.PointCloud")
            
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
            
            with Node("Frames") as frames:
                Object("MechanicalObject", name="state", template="Rigid", 
                                           position=[[0,0,0,0,0,0,1]])
                Object("BarycentricMapping", input_topology=beam.topology.linkpath)

    with Node("GaussianSplat") as gs:
        Object("PointCloudContainer", name="loader", filename="splats/spot.ply")
        Object("PointCloudContainer", name="container", filename="splats/spot.ply")
        Object("PointCloudTransform", name="transform", input= gs.loader.linkpath,
                                                        output = gs.container.linkpath,
                                                        scale=[30,1,1],
                                                        frame=[10, 10, 10, -0.375,0.598,0.598,0.375])
        gs.init()   
        Object(IdentitySkinning, name="skinning", 
                                 target=modelling.Beam.Frames.state, 
                                 container=gs.container)

        Object("PointCloudRenderer",  name="renderer", 
                                      indices=gs.container.indices.value, 
                                      geometry=gs.container.linkpath,
                                      frames=modelling.Beam.Frames.state.position.linkpath,
                                      frameIndices=gs.skinning.indices.linkpath,
                                      camera=root.camera.linkpath)
    
    with Node("Simulation") as this_node:
        Object("EulerImplicitSolver", name="solver")
        Object("CGLinearSolver", name="solver")
        this_node.addChild(modelling)

