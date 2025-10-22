def createScene(root):
    root.addObject("RequiredPlugin", name="Sofa.PointCloud")

    root.addObject("BackgroundSetting", name="settings", color=[0.0,0.0,0.0,1.0])    
    root.addObject("InteractiveCamera", name="camera", computeZClip=False, zFar=1000)    
    
    root.addChild("Meshes")
    root.Meshes.addObject("MeshOffLoader", name="geometry", filename="assets/garden.off")

    root.addChild("TriangleCloud")
    root.TriangleCloud.addObject("OglModel", src=root.Meshes.geometry.linkpath)