

def createScene(root):
    root.addObject("RequiredPlugin", name="Sofa.PointCloud")

    root.addObject("PointCloudRenderer")
    root.addObject("PointCloudData", name="pc1", filename="example.ply")
    root.addObject("PointCloudRenderer", input=root.pc1.linkpath)