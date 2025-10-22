import Sofa 
import Sofa.Core
import numpy
import os
import SofaRuntime
p = os.path.abspath( os.path.join(os.getcwd(), "assets") )
SofaRuntime.DataRepository.addFirstPath(p)  


class ROI(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.addData("size", value=0, default=0, help="size", group="", type="int")
        self.container = kwargs.get("container")
        self.renderer = kwargs.get("renderer")

    def is_closer_to(self, array, center, radius):
        tmp = numpy.sqrt(numpy.sum((array-center)**2, axis=1)) < radius
        for i in range(0, len(tmp)-4, 4):
            if tmp[i+0] or tmp[i+1] or tmp[i+2] or tmp[i+3]: 
                tmp[i+0] = True
                tmp[i+1] = True
                tmp[i+2] = True
                tmp[i+3] = True
        print("PY", tmp[0:100])
        return tmp    

    def onKeypressedEvent(self, event):
        if event["key"] == 'A':
            print("VALUES ", type(self.container.positions.value))
            center, radius = numpy.array([0.0,0.0,0.0]), 3.0
            res = numpy.nonzero( self.is_closer_to( self.container.positions.value, center, radius ))[0]
            print("PY", res[0:100])
            self.renderer.indices = res 
            
def createScene(root):
    root.addObject("RequiredPlugin", name="Sofa.PointCloud")

    root.addObject("BackgroundSetting", name="settings", color=[0.0,0.0,0.0,1.0])    
    root.addObject("InteractiveCamera", name="camera", computeZClip=False, zFar=1000)    
    
    root.addChild("Meshes")
    root.Meshes.addObject("OglModel", filename="mesh/dragon.obj", scale=0.1, translation=[0,3,0])
    root.addChild("PointCloud")
    root.PointCloud.addChild("Geometry")
    root.PointCloud.Geometry.addObject("PointCloudContainer", name="pc1", filename="assets/flowers.ply")

    root.PointCloud.addChild("Renderer")
    root.PointCloud.Renderer.addObject("PointCloudRenderer", 
                                        name="renderer", 
                                        indices=root.PointCloud.Geometry.pc1.indices.value, 
                                        geometry=root.PointCloud.Geometry.pc1.linkpath, 
                                        camera=root.camera.linkpath)

    root.addObject(ROI("roi", renderer=root.PointCloud.Renderer.renderer, 
                              container=root.PointCloud.Geometry.pc1))
