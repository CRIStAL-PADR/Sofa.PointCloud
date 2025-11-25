from psl import SofaScene, SofaPrefab, Object, Node, this_node, set
from spot import Spot 

@SofaScene
def createScene(root):
    Object("RequiredPlugin", name="Sofa.PointCloud")

    Object("BackgroundSetting", name="settings", color=[0.0,0.0,0.0,1.0])    
    Object("InteractiveCamera", name="camera", computeZClip=False, zFar=1000)    

    Object("PointCloudRenderer", name="renderer", 
                                camera=root.camera.linkpath, printLog=True)

    with Node("Modelling") as modelling:
        with Spot("spot1") as this_object:
             pass
