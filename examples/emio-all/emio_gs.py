import sys
import os
sys.path.append(os.path.dirname(os.path.realpath(__file__))+"/emio")

import Sofa
from Sofa.Types import RGBAColor
from splib3 import animation
from splib3 import numerics
import math
import psl
from psl import SofaScene, SofaPrefab, Object, Node, this_node, set
import parts.emio
import SofaRuntime 
import numpy
import json

def distance(a, b):
    q = a[0] - b[0]
    r = a[1] - b[1]
    s = a[2] - b[2]
    return math.sqrt(q*q+r*r+s*s)

class Skinning(Sofa.Core.Controller):
    def __init__(self, *args, **kwargs):
        frames = [[0,0,0,0,0,0,1]]*2

        Sofa.Core.Controller.__init__(self, *args, **kwargs)
        self.addData("size", value=0, default=0, help="size", group="", type="int")
        self.addData("frames", value=frames, default=frames, help="size", group="", type="Rigid3::VecCoord")
        self.addData("indices", value=[0], default=[0], help="size", group="", type="vector<int>")
        self.container = kwargs.get("container")
        self.renderer = kwargs.get("renderer")
        self.target = kwargs.get("target")
        self.legs = [self.target.Leg0.Leg0DeformablePart.MechanicalObject,
                     self.target.Leg1.Leg1DeformablePart.MechanicalObject,
                     self.target.Leg2.Leg2DeformablePart.MechanicalObject,
                     self.target.Leg3.Leg3DeformablePart.MechanicalObject]
        self.initializeFrames()
         

    def is_closer_to(self, array, center, radius):
        tmp = numpy.sqrt(numpy.sum((array-center)**2, axis=1)) < radius
        for i in range(0, len(tmp)-4, 4):
            if tmp[i+0] or tmp[i+1] or tmp[i+2] or tmp[i+3]: 
                tmp[i+0] = True
                tmp[i+1] = True
                tmp[i+2] = True
                tmp[i+3] = True
        return tmp    

    def initializeFrames(self):
        ref_frames = json.load(open("dump-emio.json"))
        frame_count = 2
        for name, frames in ref_frames.items():
            frame_count += len(frames)

        print(f"Total number of frame: {frame_count}")
        self.frames = [[0,   0,0,0,0,0,1]]*frame_count
        with self.frames.writeableArray() as w:
            w[0, 0:]  = [0,   0,0,0,0,0,1]
            w[1, 0:]  = [0,-180,0,1,0,0,0]
        
            i = 2 
            for name, frames in ref_frames.items():
                print("processing ", name)
                for frame in frames:
                    print(f"SETTING AT {i}, {frame}")
                    w[i, 0:] = frame
                    i+=1

        # Load the cube
        p = json.load(open("../../../assets/spot-cleaned.json"))
        indices = [0] * len(self.container.positions.value)
        for index in p["Group1"]:
            indices[index] = 1

        i_indice = 0
        for p in self.container.positions.value:
            if indices[i_indice] == 0: 
                i_frame = 2
                for f in self.frames.value[2:]:
                    pf = f[0:3]
                    d = distance(p, pf) 

                    if d < 25: 
                        indices[i_indice] = i_frame 
                        #print(f"  I'm below a limit {d} for {i_frame} and {i_indice} => , {indices[i_indice]}")
                        
                    i_frame += 1
            i_indice += 1

        self.indices = indices

    def onAnimateBeginEvent(self, event):
        
        with self.frames.writeableArray() as w:
            w[1, 0:]  = self.target.CenterPart.Load.MechanicalObject.position.value[0]
            
            i = 2    
            for frames in self.legs:
                j=0
                for frame in frames.position.value:
                    w[i, 0:] = frame 
                    i+=1
                    j+=1

            for frame in self.frames.value[2:]:
                i+=1

    def onKeypressedEvent(self, event):
        indices = [0]*len(self.container.positions.value)
        distances = [100000]*len(self.container.positions.value)
        print("Number of indices" , len(indices)) 

        tmp = {}
        if event["key"] == 'A':            
            i = 0
            for frames in self.legs:
                tmp[f"Leg{i}"] = frames.position.value.tolist()
                i+=1
        print(tmp)
        json.dump(tmp, open("dump-emio.json", "w+t"))

def EmioGS(name, root, emio_meca):
    with Node("geometry") as geometry:
        Object("PointCloudContainer", name="loader", filename="../../../assets/emio-test1.ply")

        Object("PointCloudContainer", name="container", filename="../../../assets/emio-test1.ply")

        Object("PointCloudTransform", name="transform", input= geometry.loader.linkpath,
                                                        output = geometry.container.linkpath,
                                                        scale=[780,100,100],
                                                        frame=[0, 0,0, -0.690,-0.153,-0.153, 0.690]
                                                        )

    with Node("renderer") as this_node:      
        geometry.init() 
        
        this_node.addObject(Skinning(name="skinning", legs=[], 
                                     container=geometry.container,
                                     target=emio_meca))    

        Object("PointCloudRenderer",    name="renderer", 
                                        indices=geometry.container.indices.value, 
                                        geometry=geometry.container.linkpath,
                                        frames=this_node.skinning.frames.linkpath, 
                                        frameIndices=this_node.skinning.indices.linkpath,
                                        camera=root.camera.linkpath)

        this_node.skinning.renderer = this_node.renderer

    #return emio
