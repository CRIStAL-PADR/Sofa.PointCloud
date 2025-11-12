def createScene(root):
    root.addObject("RequiredPlugin", name="Sofa.PointCloud")

    root.addObject("BackgroundSetting", name="settings", color=[0.0,0.0,0.0,1.0])    
    root.addObject("InteractiveCamera", name="camera", computeZClip=False, zFar=1000)    
    
    root.addChild("PointCloud")
    root.PointCloud.addChild("Geometry")
    root.PointCloud.addObject("PointCloudContainer", name="container", filename="splats/flowers.ply")
    root.PointCloud.addObject("PointCloudVisualModel", name="visual", geometry=root.PointCloud.container.linkpath)
    root.PointCloud.addObject("PointCloudRenderer", name="renderer", camera=root.camera.linkpath)
