"""
Walkers algorithm
developed by: Zaqi Fathis
initial development: Daria Dordina
"""

#! python3

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

class Walkers():
    def __init__(self, iStart, skeleton_pc, iNeighbours, max_walker_step):
        self.iStart = iStart
        self.skeleton_pc = skeleton_pc
        self.rtree = rg.RTree.CreatePointCloudTree(skeleton_pc)
        self.scope_of_points = skeleton_pc.GetPoints()
        self.scope_of_colors = []
        self.iNeighbours = iNeighbours
        self.max_walker_step = max_walker_step

        self.startPt = rg.Point3d(0,0,0)
        self.endPt = rg.Point3d(0,0,0)
        self.prev_vector = rg.Vector3d(0,0,0)
        self.finished = False

        self.results = []

    def getColor(self):
        if self.skeleton_pc.GetColors():
            self.scope_of_colors = self.skeleton_pc.GetColors()
        else:
            self.scope_of_colors = [rh.Display.ColorRGBA(0,0,0,0)] * len(self.scope_of_points)
    
    def compute_weight_color(self, n_colors):
        mycolors = []
        
        for i in range(len(n_colors)):
            mycolors.append(n_colors[i].G/255)

        mycolors = [i * color_coefficient for i in mycolors]
        return mycolors

    def compute_weight_veclength(self, n_points):
        vec = [i - self.startPt for i in n_points]
        vecLengths = [i.Length for i in vec]
        myvecLengths = [(1 - v * vector_length_coefficient / max(vecLengths)) for v in vecLengths]
        return myvecLengths, vec

    def compute_weight_vecdir(self, vec):
        default_vec = self.endPt - self.startPt
        angles = [rg.Vector3d.VectorAngle(v, default_vec) * angle_coefficient for v in vec]
        myangles = [(1 - angle * angle_coefficient / max(angles)) for angle in angles]
        return myangles, angles

    def compute_weight_smoothness(self, vec, angles):
        if self.prev_vector.Length != 0:
            smooth_angles = [rg.Vector3d.VectorAngle(v, self.prev_vector) * angle_coefficient for v in vec]
            mysmoothangles = [ (1-sm * smoothness_coefficient/max(smooth_angles)) for sm in smooth_angles]
            return mysmoothangles
        else:
            mysmoothangles = [0 for sm in angles]
            return mysmoothangles

    def calculate_coefficients(self, n_points, n_colors):
        mycolors = self.compute_weight_color(n_colors) #compute weight by color
        myvecLengths, vec = self.compute_weight_veclength(n_points) #compute weight by vec length
        myangles, angles = self.compute_weight_vecdir(vec) #compute weight by vec direction
        mysmoothangles = self.compute_weight_smoothness(vec, angles) #compute weight by smoothness

        sum_coeff = []
        for i in range(len(mycolors)):
            sum = mycolors[i] + myvecLengths[i] + myangles[i] + mysmoothangles[i]
            sum_coeff.append(sum)
        return sum_coeff
    
    def update_data(self, foundPt):

        if foundPt != self.endPt:
            self.prev_vector = self.startPt - foundPt
            self.startPt = foundPt
            self.results.append(foundPt)
        else:
            self.finished = True
            self.results.append(self.endPt)

    def walk(self):
        ptCheck = [self.startPt]
        neighbourPt_idx = self.rtree.Point3dKNeighbors(self.scope_of_points, ptCheck, self.iNeighbours + 1)
        n_points = []
        n_colors = []

        for j in neighbourPt_idx:
            for k in range(1, len(j)):
                index = j[k]
                n_points.append(self.scope_of_points[index])
                n_colors.append(self.scope_of_colors[index])

        coeff = self.calculate_coefficients(n_points, n_colors)
        choice = coeff.index(max(coeff))

        #check if found point is in results
        if(n_points[choice] in self.results):
            culled_list = [-1000 if x==max(coeff) else x for x in coeff]
            choice = culled_list.index(max(culled_list))

        foundPt = n_points[choice]
        self.update_data(foundPt)
        
    def run(self):
        for i in range(len(self.iStart) - 1):
            self.finished = False
            self.startPt = self.iStart[i]
            self.endPt = self.iStart[i+1]
            self.results.append(self.startPt)
            print("starPt", i)

            for j in range(self.max_walker_step):
                if self.finished == False:
                    self.walk()
                else:
                    print("total steps:",j)
                    break
            
            if i == (len(self.iStart) - 2):
                self.results.append(self.endPt)
                self.results = rg.Point3d.SortAndCullPointList(self.results, 0.01)

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

class WalkerOptionsDialog(forms.Dialog[bool]):
 
    # Dialog box Class initializer
    def __init__(self):
        super().__init__()
        # Initialize dialog box
        self.Title = 'Walker Options'
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
        self.neighbour_updown.Value = 20

        #walker steps
        self.steps_label = forms.Label()
        self.steps_label.Text = "Max walker steps:"
        self.steps_updown = forms.NumericUpDown()
        self.steps_updown.DecimalPlaces = 0
        self.steps_updown.Increment = 1
        self.steps_updown.MaxValue = 10000
        self.steps_updown.MinValue = 100
        self.steps_updown.Value = 500

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
        layout.AddRow(self.steps_label, self.steps_updown)
        layout.AddRow(self.cpt_label, self.cpt_updown)
        layout.AddRow(None) # spacer
        layout.AddRow(self.DefaultButton, self.AbortButton)
 
        # Set the dialog content
        self.Content = layout
 
    # Get the value of the textbox
    def GetNeighbourSize(self):
        return self.neighbour_updown.Value
    
    def GetSteps(self):
        return self.steps_updown.Value
 
    def GetControlPoints(self):
        return self.cpt_updown.Value
 
    # Close button click handler
    def OnCloseButtonClick(self, sender, e):
        self.Close(False)
 
    # OK button click handler
    def OnOKButtonClick(self, sender, e):
        self.Close(True)

def RequestOption():
    dialog = WalkerOptionsDialog()
    rc = dialog.ShowModal(Rhino.UI.RhinoEtoApp.MainWindow)
    if (rc):
        return True, (int(dialog.GetNeighbourSize()), int(dialog.GetSteps()), int(dialog.GetControlPoints()))
    else:
        return False, (int(dialog.GetNeighbourSize()), int(dialog.GetSteps()), int(dialog.GetControlPoints()))
    
################################################################################

def rebuildCurve(curve, target_controlPoint):
    params = curve.DivideByCount(target_controlPoint, True)
    pts = [curve.PointAt(p) for p in params]
    final_curve = rg.Curve.CreateInterpolatedCurve(pts, 3)
    return final_curve

def runWalkers():
    helper = EscapeKeyHelper()

    #USER INPUT
    pcloud_id = rs.GetObject("Select pointcloud", 2, False, False)
    if pcloud_id == None: return

    input_pts = rs.GetPoints(False, False, "Select the start pt", "Select the next point or press spacebar if finished")
    if (helper.EscapeKeyPressed):return 
    
    bool_op, option = RequestOption()
    iNeighbours, max_walker_step, controlpoints = option
    if bool_op == False: return

    #----------------------------------------------------------------

    #sorted input points
    iStart = rg.Point3d.SortAndCullPointList(input_pts, 0.01)

    #convert pcloud to points
    ref = rh.DocObjects.ObjRef(sc.doc.ActiveDoc, pcloud_id)
    skeleton_pc = rh.DocObjects.ObjRef.PointCloud(ref)

    #walkers
    walkers = Walkers(iStart, skeleton_pc, iNeighbours, max_walker_step)
    walkers.getColor()
    walkers.run()
    results_points = list(walkers.results)
    pc = rg.PointCloud(results_points)

    #skeleton
    skeleton = Skeleton(pc, 10, 5)
    skeleton.getFlags()
    skeleton.run()
    result= skeleton.result
    skel_pts = result.GetPoints()
    crv = rg.Curve.CreateInterpolatedCurve(skel_pts, 3)
    final_curve = rebuildCurve(crv, controlpoints)

    #object attributes
    prop = FF_Attributes.getObjectProperties()
    
    #bake object
    sc.doc.Objects.AddCurve(final_curve, prop)

    print("walkers is done!")


################################################################################

#predefined
color_coefficient = 15
vector_length_coefficient = 1
angle_coefficient = 6
smoothness_coefficient = 3

runWalkers()