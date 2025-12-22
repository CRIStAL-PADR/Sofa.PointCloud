def createScene(root):
    root.addObject("RequiredPlugin", name="Sofa.PointCloud")

    root.addObject("BackgroundSetting", name="settings", color=[0.0,0.0,0.0,1.0])    
    root.addObject("InteractiveCamera", name="camera", computeZClip=False, zFar=1000, pivot=3)    
    
    root.addChild("Meshes")
    root.Meshes.addObject("OglModel", filename="mesh/dragon.obj", scale=0.1, translation=[0,3,0])

    root.addChild("PointCloud")
    root.PointCloud.addObject("PointCloudContainer", name="container", filename="splats/garden.ply")
    root.PointCloud.addObject("PointCloudVisualModel", name="visual", geometry=root.PointCloud.container.linkpath, isStatic=True)
    root.PointCloud.addObject("PointCloudRenderer", name="renderer", camera=root.camera.linkpath, withCuda=True)
