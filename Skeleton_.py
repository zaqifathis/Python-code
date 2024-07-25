import rhinoscriptsyntax as rs
import scriptcontext as sc
import math

import System
import System.Collections.Generic
import Rhino 
import Rhino.Geometry as rg


class Skeleton(): 
    def __init__(self, pcloud, neighbourSize, iterations):
        self.pcloud = pcloud
        self.points = pcloud.GetPoints()
        self.neighbourSize = neighbourSize
        self.isAligned = []
        self.iterations = iterations
        self.result = rg.PointCloud()
    
    def getFlags(self):
        self.isAligned = [False for i in range(len(self.points))]
        self.isAligned[0] = True
        self.isAligned[len(self.points)-1] = True

    
    def getRtree(self):
        needlePts = []
        for i in range(len(self.isAligned)):
            if self.isAligned[i]:
                continue
            else:
                needlePts.append(self.points[i])
        
        neighbours_ids = rg.RTree.Point3dKNeighbors(self.points, needlePts, self.neighbourSize +1)
        neighbours_tolist = []
        for nn in neighbours_ids:
            ids = []
            for n in nn:
                ids.append(n)
            neighbours_tolist.append(ids)
        
        return neighbours_tolist
    
    def run(self):
        for itr in range(self.iterations):
            if False in self.isAligned:
                newpoints = []
                step = 0  # to keep track of the index of neighbours list as we only do rtree on pt which isAligned is False

                rtree = self.getRtree()

                for i in range(len(self.isAligned)):
                    if self.isAligned[i]:
                        newpoints.append(self.points[i])
                    else:
                        local_neighboursid = rtree[step]
                        
                        if not isinstance(local_neighboursid, list):
                            print(f"Error: local_neighboursid is not a list, it is {type(local_neighboursid)}")
                            print(f"Value of local_neighboursid: {local_neighboursid}")
                            return
                        
                        local_neighbours = [self.points[id] for id in local_neighboursid]
                        
                        # fitline
                        bolFl, fitLine = rg.Line.TryFitLineToPoints(local_neighbours)
                        if not bolFl:
                            print("Error: Could not fit line to points")
                            return

                        mlsPt = fitLine.ClosestPoint(local_neighbours[0], True)
                        newpoints.append(mlsPt)

                        midPt = fitLine.PointAt(0.5)
                        endPt = fitLine.From

                        # fitplane
                        fitPln = rs.PlaneFitFromPoints(local_neighbours)
                        # bolFp, fitPln = rg.Plane.FitPlaneToPoints(local_neighbours)
                        if not fitPln:
                            print("Error: Could not fit plane to points")
                            return
                        
                        fitPln.Origin = midPt

                        # rotate plane
                        vec1 = endPt - midPt
                        vec2 = fitPln.XAxis
                        angle = rg.Vector3d.VectorAngle(vec2, vec1, fitPln)
                        fitPln.Rotate(angle, fitPln.ZAxis, midPt)

                        # axis aligned boundingbox
                        # to check directionality degree
                        local_cloud = rg.PointCloud(local_neighbours)
                        bbox = local_cloud.GetBoundingBox(fitPln)

                        minPt = bbox.Corner(True, True, True)
                        maxPt = bbox.Corner(False, False, False)
                        xlen = maxPt.X - minPt.X
                        ylen = maxPt.Y - minPt.Y
                        zlen = maxPt.Z - minPt.Z

                        dd = xlen / (xlen + ylen + zlen)
                        if dd > 0.7:
                            self.isAligned[i] = True
                        
                        step += 1

                self.points = newpoints

            else:
                break
        
        # get skeleton pointcloud
        self.result.AddRange(self.points)

