

def createScene(root):
    root.addObject("RequiredPlugin", name="Sofa.PointCloud")

    root.addObject("BackgroundSetting", name="settings", color=[0.0,0.0,0.0,1.0])    
    root.addObject("InteractiveCamera", name="camera", computeZClip=False, zFar=1000)    
    
    root.addChild("Meshes")
    #root.addObject("OglShader")
    root.Meshes.addObject("OglModel", filename="mesh/dragon.obj", scale=0.1, translation=[0,3,0])
    root.addChild("PointCloud")

    root.PointCloud.addObject("PointCloudContainer", name="pc1", filename="assets/garden.ply")
    #root.PointCloud.addObject("PointCloudContainer", name="pc1", filename="assets/garden.ply")
    #root.PointCloud.addObject("PointCloudTransform", name="transform", input="@pc1", output="@pc2")

    #root.PointCloud.addObject("PointCloudInspector", name="inspect", input="@pc1")

    #root.PointCloud.addObject("PointCloudContainer", name="pc1", filename="assets/treehill.ply")
    root.PointCloud.addObject("PointCloudRenderer", geometry=root.PointCloud.pc1.linkpath, camera=root.camera.linkpath)
