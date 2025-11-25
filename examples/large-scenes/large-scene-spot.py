def createScene(root):
    root.addObject("RequiredPlugin", name="Sofa.PointCloud")

    root.addObject("BackgroundSetting", name="settings", color=[0.0,0.0,0.0,1.0])    
    root.addObject("InteractiveCamera", name="camera", computeZClip=False, zFar=1000, pivot=3, 
                   zoomSpeed=10, panSpeed=0.01)    
    
    root.addChild("PointCloud")
    root.PointCloud.addChild("Geometry")
    root.PointCloud.Geometry.addObject("PointCloudContainer", name="pc1", filename="splats/Spot/Full_spot.ply")

    root.PointCloud.addObject("PointCloudVisualModel", geometry=root.PointCloud.Geometry.pc1.linkpath, isStatic=True)

    root.PointCloud.addChild("Renderer")
    root.PointCloud.addObject("PointCloudRenderer", 
                                        name="renderer", 
                                        camera=root.camera.linkpath)
    

