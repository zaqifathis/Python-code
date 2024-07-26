"""
Boundary Point Detection (BPD) algorithm
developed by: Zaqi Fathis

"""
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

import FF_Attributes

################################################################################

def pointTransformation(pt, initialPlane, targetPlane):
    transformation = rg.Transform.ChangeBasis(targetPlane, initialPlane)
    pt.Transform(transformation)
    return pt

def getGap(angles):
    angularGap = []

    for i in range(0,len(angles) - 1):
        gap = angles[i + 1] - angles[i]
        angularGap.append(gap)
    
    #add gap between first and last 
    first = angles[0] + 360
    last = angles[len(angles) - 1]
    gap2 = first - last
    angularGap.append(gap2)

    angularGap.sort(reverse=True)
    return angularGap

def getAngularGap(localPoints):
    results = []

    #fit line
    bolFl, fitLine = rg.Line.TryFitLineToPoints(localPoints)
    midPt = fitLine.PointAt(0.5)
    endPt = fitLine.From

    #fit plane
    bolPl, localPlane = rg.Plane.FitPlaneToPoints(localPoints)
    localPlane.Origin = midPt

    #aligned plane
    vec1 = endPt - midPt
    vec2 = localPlane.XAxis
    rotationAngle = rg.Vector3d.VectorAngle(vec2, vec1, localPlane)
    localPlane.Rotate(rotationAngle, localPlane.ZAxis, midPt)

    localPlane.Origin = localPoints[0] #update local plane
    planeOriginXY = rg.Plane(rg.Point3d(0,0,0), rg.Point3d(1,0,0), rg.Point3d(0,1,0)) #xyplane

    #Compute angular angles
    angularAngles = []

    for i in range(1, len(localPoints)):
        projectedPt = localPlane.ClosestPoint(localPoints[i])
        transformedPt = pointTransformation(projectedPt, localPlane, planeOriginXY)

        vec = transformedPt - planeOriginXY.Origin
        angle = math.atan2(vec.X, vec.Y)
        angle_degree = math.degrees(angle)

        if angle_degree < 0:
            angle_degree += 2 * math.pi
        
        angularAngles.append(angle_degree)
    
    angularAngles.sort()
    results = getGap(angularAngles)

    return results

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


class BpdOptionsDialog(forms.Dialog[bool]):
 
    # Dialog box Class initializer
    def __init__(self):
        super().__init__()
        # Initialize dialog box
        self.Title = 'BPD Options'
        self.Padding = drawing.Padding(10)
        self.Resizable = False

        #neighbour
        self.neighbour_label = forms.Label()
        self.neighbour_label.Text = "Neighbour size:"
        self.neighbour_updown = forms.NumericUpDown()
        self.neighbour_updown.DecimalPlaces = 0
        self.neighbour_updown.Increment = 1
        self.neighbour_updown.MaxValue = 100
        self.neighbour_updown.MinValue = 10
        self.neighbour_updown.Value = 15

        #Angle to test
        self.angle_label = forms.Label()
        self.angle_label.Text = "Angle to tests:"
        self.angle_updown = forms.NumericUpDown()
        self.angle_updown.DecimalPlaces = 0
        self.angle_updown.Increment = 1
        self.angle_updown.MaxValue = 360
        self.angle_updown.MinValue = 50
        self.angle_updown.Value = 90

        #target group
        self.target_label = forms.Label()
        self.target_label.Text = "Target group:"
        self.target_updown = forms.NumericUpDown()
        self.target_updown.DecimalPlaces = 0
        self.target_updown.Increment = 1
        self.target_updown.MaxValue = 50
        self.target_updown.MinValue = 1
        self.target_updown.Value = 1

        #alpha size
        self.alpha_label = forms.Label()
        self.alpha_label.Text = "Alpha size:"
        self.alpha_updown = forms.NumericUpDown()
        self.alpha_updown.DecimalPlaces = 1
        self.alpha_updown.Increment = 0.1
        self.alpha_updown.MaxValue = 10
        self.alpha_updown.MinValue = 0.1
        self.alpha_updown.Value = 0.5
 
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
        layout.AddRow(self.angle_label, self.angle_updown)
        layout.AddRow(self.target_label, self.target_updown)
        layout.AddRow(self.alpha_label, self.alpha_updown)
        layout.AddRow(None) # spacer
        layout.AddRow(self.DefaultButton, self.AbortButton)
 
        # Set the dialog content
        self.Content = layout
 
    # Get the value of the textbox
    def GetNeighbourSize(self):
        return self.neighbour_updown.Value
    
    def GetAngle(self):
        return self.angle_updown.Value
 
    def GetTarget(self):
        return self.target_updown.Value
    
    def GetAlpha(self):
        return self.alpha_updown.Value
 
    # Close button click handler
    def OnCloseButtonClick(self, sender, e):
        self.Close(False)
 
    # OK button click handler
    def OnOKButtonClick(self, sender, e):
        self.Close(True)

def RequestOption():
    dialog = BpdOptionsDialog()
    rc = dialog.ShowModal(Rhino.UI.RhinoEtoApp.MainWindow)
    if (rc):
        return True, (int(dialog.GetNeighbourSize()), int(dialog.GetAngle()), int(dialog.GetTarget()), dialog.GetAlpha())
    else:
        return False, (int(dialog.GetNeighbourSize()), int(dialog.GetAngle()), int(dialog.GetTarget()), dialog.GetAlpha())
    
################################################################################

def runBoundaryPointDetection():
    helper = EscapeKeyHelper()

    #USER INPUT
    pcloud_id = rs.GetObject("Select pointcloud", 2, False, False)
    if pcloud_id == None: return

    bool_op, option = RequestOption()
    input_neighbourSize, input_angleTest, input_targetGroup, input_aplha = option
    if bool_op == False: return
    
    #----------------------------------------------------------------

    ref = rh.DocObjects.ObjRef(sc.doc.ActiveDoc, pcloud_id)
    pcloud = rh.DocObjects.ObjRef.PointCloud(ref)
    points = rg.PointCloud.GetPoints(pcloud) #convert pcloud to points

    #get neighbours
    neighbours_indices = rg.RTree.Point3dKNeighbors(points, points, input_neighbourSize + 1)


    #Compute Boundary Point Detection
    ptsArr = []
    pts_np = []

    for idx, i in enumerate(neighbours_indices):
        neighbourPoints = []

        for j in i:
            pt = points[j]
            neighbourPoints.append(pt)
        
        angularGap = getAngularGap(neighbourPoints)
        highestGap = angularGap[0]

        if highestGap > input_angleTest:
            p = points[idx]
            ptsArr.append(p)

            pExplode = [p.X, p.Y, p.Z]
            pts_np.append(pExplode)
            
    #CLUSTERING ALGORITHM USING HDBSCAN FROM SCIKIT-LEARN 
    #convert point to numpy array
    hdb = HDBSCAN(alpha= input_aplha).fit(pts_np)
    labels = hdb.labels_

    n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
    n_noise_ = list(labels).count(-1)
    print("Estimated number of clusters from clustering: %d" % n_clusters_)
    print("Estimated number of noise points: %d" % n_noise_)

    #object attributes
    prop = FF_Attributes.getObjectProperties()

    if n_clusters_ > input_targetGroup:
        final_pcl = rg.PointCloud(ptsArr)
        #bake object
        sc.doc.Objects.AddPointCloud(final_pcl, prop)
        print("BPD is done!")

    elif n_clusters_ <= input_targetGroup:
        #point clustering
        clusteredPoints = [[] for i in range(n_clusters_)]

        for idx, l in enumerate(labels) :
            if l == -1:
                continue
            else:
                clusteredPoints[l].append(ptsArr[idx])

        for cl in clusteredPoints:
            final_pcl = rg.PointCloud(cl)
            #bake object
            sc.doc.Objects.AddPointCloud(final_pcl, prop)
            print("BPD is done!")

################################################################################

runBoundaryPointDetection()