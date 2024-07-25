"""
MOVING LEAST SQUARE+ / SKELETON algorithm
developed by: Zaqi Fathis
"""

#! python3

import rhinoscriptsyntax as rs
import scriptcontext as sc
import math

import System
import System.Collections.Generic
import Rhino 
import Rhino.Geometry as rg


#--------------------------- METOHDS / CLASS ------------------------------------------


#--------------------------------------------------------------------------------------

#predefined
max_iterations = 10

#USER INPUT
pcloud_id = rs.GetObjects(
    message="Select pointclouds",
    filter=2,
    group=False,
    minimum_count=1,
    maximum_count=100,
    preselect=False
)
neighboursSize = rs.GetInteger("neighbours size", 10, 5, 100)
controlpoints = rs.GetInteger("num of control points", 30, 3, 100)


#results
curves_output= []

#loop through each pointcloud
for id in pcloud_id:
    ref= Rhino.DocObjects.ObjRef(sc.doc.ActiveDoc, id)
    pcloud= Rhino.DocObjects.ObjRef.PointCloud(ref)
    points = pcloud.GetPoints()

    sortedPoints = rg.Point3d.SortAndCullPointList(points, 0.005)

    #create flag for each point
    #1st and last pt alwasy fixed = True
    isAligned = [False for i in range(len(sortedPoints))]
    isAligned[0] = True
    isAligned[len(sortedPoints)-1] = True

    #run mls/skeleton
    for iter in range(max_iterations):

        if False in isAligned:
        
            #rtree
            #only do rtree for pt which is not aligned
            needlePts = []
            for i in range(len(isAligned)):
                if isAligned[i] == True:
                    continue
                else:
                    needlePts.append(sortedPoints[i])
            
            neighbours_ids = rg.RTree.Point3dKNeighbors(sortedPoints, needlePts, neighboursSize +1)
            neighbours_tolist = []
            for nn in neighbours_ids:
                neighbours_tolist.append(nn)
            
            #NEWPOINTS after mls process <<<--------------
            newpoints = []

            #to keep track of the index of neighbours list as we only do rtree on pt which isAligned is False
            step = 0 

            for i in range(len(isAligned)):
                if isAligned[i] == True:
                    newpoints.append(sortedPoints[i])
                else:
                    local_neighboursid = neighbours_tolist[step]
                    local_neighbours = [sortedPoints[id] for id in local_neighboursid]
                    
                    #fitline
                    bolFl, fitLine = rg.Line.TryFitLineToPoints(local_neighbours)
                    mlsPt = fitLine.ClosestPoint(local_neighbours[0], True)
                    newpoints.append(mlsPt)

                    midPt = fitLine.PointAt(0.5)
                    endPt = fitLine.From

                    #fitplane
                    bolFp, fitPln = rg.Plane.FitPlaneToPoints(local_neighbours)
                    fitPln.Origin = midPt

                    #rotate plane
                    vec1 = endPt - midPt
                    vec2 = fitPln.XAxis
                    angle = rg.Vector3d.VectorAngle(vec2, vec1, fitPln)
                    fitPln.Rotate(angle, fitPln.ZAxis, midPt)

                    #axis aligned boundingbox
                    #to check directionality degree
                    local_cloud = rg.PointCloud(local_neighbours)
                    bbox = local_cloud.GetBoundingBox(fitPln)

                    minPt = bbox.Corner(True, True, True)
                    maxPt = bbox.Corner(False, False, False)
                    xlen = maxPt.X - minPt.X
                    ylen = maxPt.Y - minPt.Y
                    zlen = maxPt.Z - minPt.Z

                    dd = xlen / (xlen + ylen + zlen)
                    if dd > 0.7:
                        isAligned[i] = True
                    
                    step += 1

            sortedPoints = newpoints
        else:
            break

    #sort points
    outputPts = rg.Point3d.SortAndCullPointList(sortedPoints, 0.01)

    #interpolate curve
    rs.EnableRedraw(False)
    crv = rs.AddInterpCurve(outputPts)
    #rebuid curves
    rs.RebuildCurve(crv, 3, controlpoints)

    




       
       
        

        



