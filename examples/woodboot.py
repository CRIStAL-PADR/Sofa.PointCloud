from psl import Node, Object, SofaPrefab, SofaScene

def setupScene(scene):
    with Node("Settings"):
        with Node("Plugins") as plugins: 
            plugins.addObject('RequiredPlugin', name='Sofa.Component.Constraint.Projective') # Needed to use components [FixedProjectiveConstraint]  
            plugins.addObject('RequiredPlugin', name='Sofa.Component.Engine.Select') # Needed to use components [BoxROI]  
            plugins.addObject('RequiredPlugin', name='Sofa.Component.LinearSolver.Iterative') # Needed to use components [CGLinearSolver]  
            plugins.addObject('RequiredPlugin', name='Sofa.Component.Mapping.Linear') # Needed to use components [BarycentricMapping]  
            plugins.addObject('RequiredPlugin', name='Sofa.Component.Mass') # Needed to use components [DiagonalMass]  
            plugins.addObject('RequiredPlugin', name='Sofa.Component.ODESolver.Backward') # Needed to use components [EulerImplicitSolver]  
            plugins.addObject('RequiredPlugin', name='Sofa.Component.SolidMechanics.FEM.Elastic') # Needed to use components [TetrahedronFEMForceField]  
            plugins.addObject('RequiredPlugin', name='Sofa.Component.StateContainer') # Needed to use components [MechanicalObject]  
            plugins.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Dynamic') # Needed to use components [TetrahedronSetGeometryAlgorithms,TetrahedronSetTopologyContainer,TetrahedronSetTopologyModifier]  
            plugins.addObject('RequiredPlugin', name='Sofa.Component.Topology.Container.Grid') # Needed to use components [RegularGridTopology]  
            plugins.addObject('RequiredPlugin', name='Sofa.Component.Topology.Mapping') # Needed to use components [Hexa2TetraTopologicalMapping]  
            plugins.addObject('RequiredPlugin', name='Sofa.Component.Visual') # Needed to use components [InteractiveCamera]

@SofaPrefab
def Beam(self):
        Object("RegularGridTopology", name="grid", min="-5 -5 0", max="5 5 20", n="5 5 20")
        Object("MechanicalObject", name="state", template="Vec3", translation="11 0 0")
        Object("TetrahedronSetTopologyContainer", name="topology")
        Object("TetrahedronSetTopologyModifier", name="modifier")
        Object("TetrahedronSetGeometryAlgorithms", template="Vec3", name="GeomAlgo")
        Object("Hexa2TetraTopologicalMapping", input="@grid", output="@topology")

        Object("DiagonalMass", massDensity="0.2")
        Object("TetrahedronFEMForceField", name="FEM", youngModulus="10000", poissonRatio="0.4", computeGlobalMatrix="false",
                                               method="large", computeVonMisesStress="1", showVonMisesStressPerNodeColorMap="true")
        Object("BoxROI", template="Vec3", name="box_roi", box="-6 -6 -1 50 6 2.0", drawBoxes="1")
        Object("FixedProjectiveConstraint", template="Vec3", indices="@box_roi.indices")
            
        with Node("Frames") as frames:
            Object("MechanicalObject", name="state", template="Rigid", 
                                           position=[[0,0,0,0,0,0,1]])
            Object("BarycentricMapping", input_topology=self.topology.linkpath)


import Sofa
from splib3.numerics import Quat #< probably deprecate if one day we have better implementation of quaternion utils 

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


class EulerToQuaternion(Sofa.Core.DataEngine):
    def __init__(self, *args, **kwargs):
        Sofa.Core.DataEngine.__init__(self,*args, **kwargs)
        self.addData(name="euler_angle", value=[0.0,0.0,0.0], default=[0.0,0.0,0.0], help="", group="Input", type="Vec3d")
        self.addData(name="quaternion", value=[0.0,0.0,0.0,1.0], default=[0.0,0.0,0.0,1.0], help="", group="Output", type="Quat")        

        self.addInput(self.euler_angle)
        self.addOutput(self.quaternion)

    def update(self):
        self.quaternion = list(Quat.createFromEuler(self.euler_angle.value))

class RigidDofBuilder(Sofa.Core.DataEngine):
    def __init__(self, *args, **kwargs):
        Sofa.Core.DataEngine.__init__(self,*args, **kwargs)
        self.addData(name="translation", value=[0.0,0.0,0.0], default=[0.0,0.0,0.0], help="", group="Input", type="Vec3d")        
        self.addData(name="orientation", value=[0.0,0.0,0.0,1.0], default=[0.0,0.0,0.0,1.0], help="", group="Input", type="Quat")        
        self.addData(name="rigid", value=[0.0,0.0,0.0,0.0,0.0,0.0,1.0], default=[0.0,0.0,0.0,0.0,0.0,0.0,1.0], help="", group="Output", type="Rigid3::Coord")        

        self.addInput(self.translation)
        self.addInput(self.orientation)
        self.addOutput(self.rigid)

    def update(self):
        self.rigid = self.translation.value.tolist() + self.orientation.value.tolist()


@SofaScene
def createScene(root):
    root.apply(setupScene)

    Object("RequiredPlugin",name="Sofa.PointCloud")
    Object("InteractiveCamera", name="camera")

    root.addObject(EulerToQuaternion(name="euler_to_quaternion"))
    root.addObject(RigidDofBuilder(name="rigid_dof_builder"))
    root.rigid_dof_builder.orientation.setParent(root.euler_to_quaternion.quaternion) 
    
    with Node("Modelling") as modelling:
        with Beam("beam") as beam:
            beam.state.showObject = True
            beam.state.showObjectScale = 1.0

    with Node("Visual") as this_node:
        Object("PointCloudContainer",name="loader", filename="splats/woodstructure.ply")
        Object("PointCloudContainer",name="container1", filename="splats/woodstructure.ply")

        Object("PointCloudTransform", name="transform", 
                                  input=this_node.loader.linkpath, 
                                  output=this_node.container1.linkpath,
                                  frame=[12.53,0.12,1.38, 0.0199684, -0.0549613, 0.00109937, 0.998288],
                                  scale = [70,100,100])

        this_node.init()   
        Object(IdentitySkinning, name="skinning", 
                                 target=modelling.beam.Frames.state, 
                                 container=this_node.container1)

        Object("PointCloudVisualModel", name="visualmodel", 
                                        geometry=this_node.loader.linkpath,
                                        frames=beam.Frames.state.position.linkpath,
                                        frameIndices=this_node.skinning.indices.linkpath,
                                        uniformScale = 70)

    Object("PointCloudRenderer",  name="renderer", 
                                  camera=root.camera.linkpath)
    root.renderer.printLog = True

    with Node("Simulation") as this_node:
        Object("EulerImplicitSolver", name="solver")
        Object("CGLinearSolver", name="solver")
        this_node.addChild(modelling)

