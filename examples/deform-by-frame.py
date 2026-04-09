import math
from splib3 import animation  # Il faut installer la STLIB3
import Sofa
import miniskinning

def createScene(rootNode):
    rootNode.addObject(animation.AnimationManager(rootNode))
    rootNode.addObject("RequiredPlugin", pluginName="Sofa.PointCloud")

    rootNode.addObject("InteractiveCamera", name="camera")
    rootNode.addObject('PointCloudRenderer', name='renderer', camera=rootNode.camera.linkpath)

    # Add a mechanical object to the deformable node
    mechanicalObject = rootNode.addObject('MechanicalObject', name='mechObj', 
                                          template='Rigid3d', position=[[i/10-0.5, 0, 0, 0,0,0,1] for i in range(0,10)])
    mechanicalObject.showObject = True

    rootNode.addObject("PointCloudContainer", filename="splats/spot.ply", name="container")
    rootNode.addObject("PointCloudVisualModel", name="pc", geometry=rootNode.container.linkpath)

    skin = miniskinning.Skinning(target=rootNode.mechObj, container=rootNode.container)

    rootNode.pc.frames.setParent(rootNode.mechObj.position)
    rootNode.pc.initFrames.setParent(rootNode.mechObj.position)
    rootNode.pc.frameIndices.setParent(skin.indices)
    
    def my_animation(target, factor):
        with target.position.writeableArray() as w:
            for i in range(len(w)):
                w[i][0] = 0.1 * i
                w[i][1] = math.cos(factor*i) * 0.1 
                                   
        print("HELLO")

    animation.animate(my_animation, {"target" : rootNode.mechObj}, mode="pingpong", duration=1)

