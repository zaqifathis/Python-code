
# r: scikit-learn==1.3.2
#! python3


import sklearn
import numpy as np
from sklearn.cluster import HDBSCAN

import rhinoscriptsyntax as rs
import scriptcontext as sc
import math

import System
import System.Collections.Generic
import Rhino as rh
import Rhino.Geometry as rg

import Rhino.UI
import Eto.Drawing as drawing
import Eto.Forms as forms

################################################################################

def rebuildCurve(curve, target_controlPoint):
    params = curve.DivideByCount(target_controlPoint, True)
    pts = [curve.PointAt(p) for p in params]
    final_curve = rg.Curve.CreateInterpolatedCurve(pts, 3)
    return final_curve

def joinEndCorner(clusteredPoints, cornerPoints):
    results = []

    for cluster in clusteredPoints:
        sortedCLuster = rg.Point3d.SortAndCullPointList(cluster, 0.005)
        sortedCLuster = list(sortedCLuster)

        ptStart = sortedCLuster[0]
        ptEnd = sortedCLuster[len(sortedCLuster)-1]

        dis_st = []
        dis_end = []

        for pt in cornerPoints:
            d1 = pt.DistanceTo(ptStart)
            d2 = pt.DistanceTo(ptEnd)
            dis_st.append(d1)
            dis_end.append(d2)

        sorted_st_indices = sorted(range(len(dis_st)), key=lambda x: dis_st[x])
        sorted_end_indices = sorted(range(len(dis_end)), key=lambda x: dis_end[x])

        cornerSt = cornerPoints[sorted_st_indices[0]]
        cornerEnd = cornerPoints[sorted_end_indices[0]]

        sortedCLuster.append(cornerEnd)
        sortedCLuster.insert(0, cornerSt)
        results.append(sortedCLuster)
        
    return results

################################################################################

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

################################################################################

class EscapeKeyHelper:
    # Constructor
    def __init__(self):
        self.escape_key_pressed = False
        rh.RhinoApp.EscapeKeyPressed += self.OnEscapeKeyPressed
    # Destructor
    def __del__(self):
        rh.RhinoApp.EscapeKeyPressed -= self.OnEscapeKeyPressed
    # Event handler
    def OnEscapeKeyPressed(self, sender, e):
        self.escape_key_pressed = True
    # Gets the 'escape key pressed' status
    @property
    def EscapeKeyPressed(self):
        print("exit function")
        return self.escape_key_pressed
    
################################################################################

class EdgeCreationOptionsDialog(forms.Dialog[bool]):
 
    # Dialog box Class initializer
    def __init__(self):
        super().__init__()
        # Initialize dialog box
        self.Title = 'Edge Creation Options'
        self.Padding = drawing.Padding(10)
        self.Resizable = False

        #neighbour
        self.neighbour_label = forms.Label()
        self.neighbour_label.Text = "Neighbours to be removed:"
        self.neighbour_updown = forms.NumericUpDown()
        self.neighbour_updown.DecimalPlaces = 0
        self.neighbour_updown.Increment = 1
        self.neighbour_updown.MaxValue = 100
        self.neighbour_updown.MinValue = 5
        self.neighbour_updown.Value = 7

        #control points
        self.cpt_label = forms.Label()
        self.cpt_label.Text = "Control points of the output curve:"
        self.cpt_updown = forms.NumericUpDown()
        self.cpt_updown.DecimalPlaces = 0
        self.cpt_updown.Increment = 1
        self.cpt_updown.MaxValue = 100
        self.cpt_updown.MinValue = 3
        self.cpt_updown.Value = 10
 
        # Create the default button
        self.DefaultButton = forms.Button()
        self.DefaultButton.Text ='OK'
        self.DefaultButton.Click += self.OnOKButtonClick
 
        # Create the abort button
        self.AbortButton = forms.Button()
        self.AbortButton.Text ='Cancel'
        self.AbortButton.Click += self.OnCloseButtonClick
 
        # Create a table layout and add all the controls
        layout = forms.DynamicLayout()
        layout.Spacing = drawing.Size(5, 5)
        layout.AddRow(self.neighbour_label, self.neighbour_updown)
        layout.AddRow(self.cpt_label, self.cpt_updown)
        layout.AddRow(None) # spacer
        layout.AddRow(self.DefaultButton, self.AbortButton)
 
        # Set the dialog content
        self.Content = layout
 
    # Get the value of the textbox
    def GetNeighbourSize(self):
        return self.neighbour_updown.Value 
 
    def GetControlPoints(self):
        return self.cpt_updown.Value
 
    # Close button click handler
    def OnCloseButtonClick(self, sender, e):
        self.Close(False)
 
    # OK button click handler
    def OnOKButtonClick(self, sender, e):
        self.Close(True)

def RequestOption():
    dialog = EdgeCreationOptionsDialog()
    rc = dialog.ShowModal(Rhino.UI.RhinoEtoApp.MainWindow)
    if (rc):
        return True, (int(dialog.GetNeighbourSize()), int(dialog.GetControlPoints()))
    else:
        return False, (int(dialog.GetNeighbourSize()), int(dialog.GetControlPoints()))
    
################################################################################

def runEdgeCreation():
    helper = EscapeKeyHelper()

    #USER INPUT 
    pcloud_id = rs.GetObject(
        message="Select boundary pointcloud ",
        filter=2,
    )
    if pcloud_id == None: return

    cornerPoints = rs.GetPoints(False, False, "Select corner points", "Select the next point or press spacebar if finished")
    if (helper.EscapeKeyPressed):return 

    bool_op, option = RequestOption()
    cullNeighbourSize, controlpoints = option
    if bool_op == False: return
    
    #----------------------------------------------------------------

    ref = rh.DocObjects.ObjRef(sc.doc.ActiveDoc, pcloud_id)
    pcloud = rh.DocObjects.ObjRef.PointCloud(ref)

    #remove points around corner to make clustering easy
    culledId = []
    neighbour_ids = rg.RTree.PointCloudKNeighbors(pcloud, cornerPoints, cullNeighbourSize) 
    for neighbour in neighbour_ids:
        for id in neighbour:
            culledId.append(id)

    culledId.sort()
    cleanedId = list(dict.fromkeys(culledId)) #in case have duplicate data, remove it
    pcloud.RemoveRange(cleanedId)
    points = rg.PointCloud.GetPoints(pcloud) #convert pcloud to points

    #CLUSTERING ALGORITHM USING HDBSCAN FROM SCIKIT-LEARN
    #convert point to numpy array
    ptsArr = []
    for pt in points:
        p = [pt.X, pt.Y, pt.Z]
        ptsArr.append(p)

    hdb = HDBSCAN().fit(ptsArr)
    labels = hdb.labels_

    n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
    n_noise_ = list(labels).count(-1)
    print("Estimated number of clusters: %d" % n_clusters_)
    print("Estimated number of noise points: %d" % n_noise_)

    clusteredPoints = [[] for i in range(n_clusters_)]
    for idx, l in enumerate(labels) :
        if l == -1:
            continue
        else:
            clusteredPoints[l].append(points[idx])

    # add cornerPoints
    combined = joinEndCorner(clusteredPoints, cornerPoints)

    #object attributes
    prop = rh.DocObjects.ObjectAttributes()
    prop.ObjectColor = System.Drawing.Color.FromArgb(0, 0, 255)
    prop.ColorSource = rh.DocObjects.ObjectColorSource.ColorFromObject

    for c in combined:

        #skeleton
        pc = rg.PointCloud(c)
        skeleton = Skeleton(pc, 10, 5)
        skeleton.getFlags()
        skeleton.run()

        result= skeleton.result
        skel_pts = result.GetPoints()

        crv = rg.Curve.CreateInterpolatedCurve(skel_pts, 3)
        final_curve = rebuildCurve(crv, controlpoints)
        
        #bake object
        sc.doc.Objects.AddCurve(final_curve, prop)

    print("edge creation is done!")

################################################################################

runEdgeCreation()