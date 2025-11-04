from psl import SofaScene, SofaPrefab, Object, Node, this_node, set
from spot import Spot 

@SofaScene
def createScene(root):
    Object("RequiredPlugin", name="Sofa.PointCloud")

    Object("BackgroundSetting", name="settings", color=[0.0,0.0,0.0,1.0])    
    Object("InteractiveCamera", name="camera", computeZClip=False, zFar=1000)    

    with Node("Modelling") as modelling:
        with Spot("spot1") as this_object:
            with Node("visual"): 
                Object("PointCloudRenderer", 
                                        name="renderer", 
                                        indices=this_object.geometry.pc1.indices.value, 
                                        geometry=this_object.geometry.pc1.linkpath, 
                                        camera=root.camera.linkpath)