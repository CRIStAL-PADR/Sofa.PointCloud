import spot.assets as assets
from psl import SofaPrefab, Object, Node

@SofaPrefab
def Spot(self):
    with Node("geometry") as this_node:
        Object("PointCloudContainer", name="loader", filename=assets.get("spot.ply"))
        Object("PointCloudContainer", name="pc1", filename=assets.get("spot.ply"))
        Object("PointCloudTransform", name="transform", input=this_node.loader.linkpath, output=this_node.pc1.linkpath)
    
    with Node("mechanical") as mechanical:
        Object("MechanicalObject", name="state", template="Rigid3")
