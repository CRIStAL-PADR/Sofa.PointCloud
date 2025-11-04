def createScene(root):
    root.addObject("RequiredPlugin", name="Sofa.PointCloud")

    root.addObject("BackgroundSetting", name="settings", color=[0.0,0.0,0.0,1.0])    
    root.addObject("InteractiveCamera", name="camera", computeZClip=False, zFar=1000)    
    
    root.addChild("PointCloud")
    root.PointCloud.addChild("Geometry")
    root.PointCloud.Geometry.addObject("PointCloudContainer", name="pc1", filename="splats/drjhonson.ply")

    root.PointCloud.addChild("Renderer")
    root.PointCloud.Renderer.addObject("PointCloudRenderer", 
                                        name="renderer", 
                                        indices=root.PointCloud.Geometry.pc1.indices.value, 
                                        geometry=root.PointCloud.Geometry.pc1.linkpath, 
                                        camera=root.camera.linkpath)