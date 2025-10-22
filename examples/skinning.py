import Sofa
import math
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
        self.initializeFrames()
        
    def initializeFrames(self):
        frame_count = len(self.target.position.value)
        print(f"Total number of frame: {frame_count}")
        self.frames = [[0,0,0,0,0,0,1]]*frame_count
        indices = [0]*len(self.container.positions.value)

        with self.frames.writeableArray() as w:
            w[:]  = self.target.position.value        
        
        r = json.load(open("dump-fem-ges.json")) 
        self.indices = list(r)

    def saveFrames(self):        
        
        frame_count = len(self.target.position.value)
        print(f"Total number of frame: {frame_count}")

        # Initialize by default the indices so every object point to frame 0
        indices = [0]*len(self.container.positions.value)

        self.frames = [[0,0,0,0,0,0,1]]*frame_count
        with self.frames.writeableArray() as w:
            w[:]  = self.target.position.value

        
        i_indice = 0
        for p in self.container.positions.value:
            if indices[i_indice] == 0: 
                i_frame = 0
                d_min = 100000
                for f in self.target.position.value:
                    pf = [0,0, f[2]]
                    p = [0,0, p[2]]
                    d = distance(p, pf) 
                    #print(f"{p} et {pf} = {d} vs {d_min} => {i_frame} {f} et {p}")
                    if d < d_min:
                        d_min = d  
                        indices[i_indice] = i_frame 
                        #print(f"  I'm below a limit {d} for {i_frame} and {i_indice} => , {indices[i_indice]}")
                        #continue
                    i_frame += 1
            i_indice += 1
        self.indices = indices
        json.dump(self.indices.value.tolist(), open("dump-fem-ges.json", "wt"))

    def onKeypressedEvent(self, params):
        print("KEY MOVE ", params)
        self.saveFrames()


    def onAnimateBeginEvent(self, event):
        with self.frames.writeableArray() as w:
            w[:]  = self.target.position.value