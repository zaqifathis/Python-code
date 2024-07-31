#! python3

import rhinoscriptsyntax as rs
import scriptcontext as sc
import math

import System
import System.Collections.Generic
import Rhino as rh
import Rhino.Geometry as rg
from ghpythonlib.components import ConvexHull

import Rhino.UI
import Eto.Drawing as drawing
import Eto.Forms as forms

################################################################################

def get_convexhull(boundCurves):
    cornerPts = []
    for cr in boundCurves:
        cornerPts.append(cr.PointAtStart)
        cornerPts.append(cr.PointAtEnd)
    hull = ConvexHull(cornerPts, rg.Plane.WorldXY)[0] #convexhull
    plyline = hull.ToPolyline()
    explodedHull = plyline.GetSegments()
    return explodedHull

def compute_subd_curves(averagePt, boundaryCurves):
    midPtonBoundary = [bc.PointAtNormalizedLength(0.5) for bc in boundaryCurves]
    subdCurves = [rg.LineCurve(mp, averagePt) for mp in midPtonBoundary]
    return subdCurves

def getAveragePts(curves):
    temp_pts = []

    for crv in curves:
        #divide curve
        divideParam = crv.DivideByCount(100, True)
        for t in divideParam:
            p = crv.PointAt(t)
            temp_pts.append(p)
    sumX = 0
    sumY = 0
    sumZ = 0
    length = len(temp_pts)

    for pt in temp_pts:
        sumX += pt.X
        sumY += pt.Y
        sumZ += pt.Z
    averagePt = rg.Point3d(sumX/length, sumY/length, sumZ/length)

    return averagePt

def compute_projectedSubd_toBrep(surfaceType, curves, brep):
    if surfaceType == 0:
        return rg.Curve.ProjectToBrep(curves, brep, rg.Vector3d(0,0,1), 0.1)
    if surfaceType == 1:
        crvs = []
        for crv in curves:
            pts = []
             #divide curve
            divideParam = crv.DivideByCount(100, True)
            for t in divideParam:
                p = crv.PointAt(t)
                pts.append(p)
        
            pulledPts = brep.PullPointsToFace(pts, 0.1)
            crv = rg.Curve.CreateInterpolatedCurve(pulledPts, 3)
            crvs.append(crv)
        return crvs

def reorder_sub_curves(guideCrv, crvToOrder):
    orderedSubCrv = []

    for exc in guideCrv:
        dis_temp = []
        for prc in crvToOrder:
            ptS = prc.PointAtStart # point that close to boundary
            ptTarget = exc.PointAtNormalizedLength(0.5)
            d = ptTarget.DistanceTo(ptS)
            dis_temp.append(d)

        sorted_idx = sorted(range(len(dis_temp)), key=lambda k: dis_temp[k])
        orderedSubCrv.append(crvToOrder[sorted_idx[0]])

    return orderedSubCrv

def get_split_point_parameters(targetCurve,subdCurve):
    splitParams = []

    for i in range(len(subdCurve)):
        bdc = targetCurve[i]
        subc = subdCurve[i]
        stPt = subc.PointAtStart
        t = bdc.ClosestPoint(stPt)[1]
        splitParams.append(t)

    return splitParams

def pull_firstPt_subd_toBoundary(guideCrv, subdCurve, splitParams):
    final_subdCrvs = []

    for i in range(len(subdCurve)):
        bdc = guideCrv[i]
        subc = subdCurve[i]
        t = splitParams[i]

        #update subd curve
        pts = []
        pt = bdc.PointAt(t)
        pts.append(pt)
        divparams = subc.DivideByCount(50, True)

        for idx, pr in enumerate(divparams):
            if idx != 0:
                p = subc.PointAt(pr)
                pts.append(p)

        intcrv = rg.Curve.CreateInterpolatedCurve(pts, 3)
        final_subdCrvs.append(intcrv)

    return final_subdCrvs

def split_boundary(curves, splitParams):
    boundarySeg = []

    for idx, crv in enumerate(curves):
        segments = []
        param = splitParams[idx]
        boundarySeg.append(crv.Split(param))

    return boundarySeg

def compute_clusters(boundarySeg, subCurves):
    clusters = []

    for idx, bsegs in enumerate(boundarySeg):
        cluster = []
        #for case group 1 & 2
        if idx < 2: 
            cluster.append(bsegs[1])
            cluster.append(boundarySeg[idx +1][0])
            cluster.append(subCurves[idx+1])
            cluster.append(subCurves[idx])
        else:  #case group 3
            cluster.append(bsegs[1])
            cluster.append(boundarySeg[0][0])
            cluster.append(subCurves[0])
            cluster.append(subCurves[idx])

        #join curves to get an ordered direction
        temp_join = rg.Curve.JoinCurves(cluster, 0.1, False)[0]
        explodedCrv = temp_join.Explode()
        clusters.append(explodedCrv)

    return clusters

def getDomain(gridSize):
    domain = []

    for i in range(1, gridSize):
            step = 1/gridSize
            domain.append(step * i)

    return domain

def compute_network_surface(pcloud, baseSrf, clusters, gridSize):
    domain = getDomain(gridSize)
    surfaces = []

    for cluster in clusters:
        #create edge surface
        edgeSurf = rg.Brep.CreateEdgeSurface(cluster)

        if edgeSurf.IsSurface:
            srfFace = edgeSurf.Faces[0]
        else:
            print("ERROR: edge surface did not work!")
            break
        
        #reparameterize surface domain
        srfFace.SetDomain(0, rg.Interval(0,1)) #U
        srfFace.SetDomain(1, rg.Interval(0,1)) #V

        #uv points
        Upoints, Vpoints = get_UV_points(srfFace, domain)

        #pull points to base surface
        pulledUpts = [baseSrf.PullPointsToFace(u_pts, 0.1) for u_pts in Upoints]
        pulledVpts = [baseSrf.PullPointsToFace(v_pts, 0.1) for v_pts in Vpoints]

        #pull point to closest pointcloud with min angle
        final_pulled_Upts = [pulled_pts_to_pcloud(pcloud, baseSrf, u_pts) for u_pts in pulledUpts]
        final_pulled_Vpts = [pulled_pts_to_pcloud(pcloud, baseSrf, v_pts) for v_pts in pulledVpts]

        #add first and end point to the pulled pts
        final_U_Pts = [addStartandEndPtToList(srfFace, domain[idx], 0, upts) for idx, upts in enumerate(final_pulled_Upts)]
        final_V_Pts = [addStartandEndPtToList(srfFace, domain[idx], 1, vpts) for idx, vpts in enumerate(final_pulled_Vpts)]

        #interpolate curves
        grid_U_curves = [rg.Curve.CreateInterpolatedCurve(upts, 3) for upts in final_U_Pts]
        grid_V_curves = [rg.Curve.CreateInterpolatedCurve(vpts, 3) for vpts in final_V_Pts]

        #add first and last curves to the list
        surfaceEdges = edgeSurf.Edges
        final_U_curves = addFirstAndLastCurve(surfaceEdges[1], surfaceEdges[3], grid_U_curves)
        final_V_curves = addFirstAndLastCurve(surfaceEdges[0], surfaceEdges[2],  grid_V_curves)

        #create network surface
        curves = final_U_curves + final_V_curves
        surface = rg.NurbsSurface.CreateNetworkSurface(curves, 1, 0.01, 0.01, 0.01)[0]
        surfaces.append(surface)

    return surfaces

def pulled_pts_to_pcloud(pcloud, surface, ptList):
    final_pts = []

    #get normals
    closestPt_UV = [surface.ClosestPoint(pt) for pt in ptList]
    normalsOnSurf = [surface.NormalAt(cp[1], cp[2]) for cp in closestPt_UV]
    closestPt = [surface.PointAt(cp[1], cp[2]) for cp in closestPt_UV]
    vectransform = [rg.Vector3d(pt) - rg.Vector3d(0,0,0) for pt in closestPt] #transformation
    [nm.Unitize() for nm in normalsOnSurf]

    #new ptList to search
    new_ptList = []
    for index, nm in enumerate(normalsOnSurf):
        nm /= 100
        vect = vectransform[index]
        newvec = nm + vect
        pt = rg.Point3d(newvec.X, newvec.Y, newvec.Z)
        new_ptList.append(pt)
    
    #get Rtree
    rtree = rg.RTree.PointCloudKNeighbors(pcloud, new_ptList, 30) #neighbour size is predefined
    neighbours_tolist = []
    for nn in rtree:
        ids = []
        for n in nn:
            ids.append(n)
        neighbours_tolist.append(ids)

    #get angles from neighbours
    for idx, pt in enumerate(new_ptList):
        neighbours = neighbours_tolist[idx]
        angles = []
        for nId in neighbours:
            ptTarget = pcloud.PointAt(nId)
            targetVec = ptTarget - pt
            angle = rg.Vector3d.VectorAngle(normalsOnSurf[idx], targetVec)
            angles.append(angle)
        
        #sort angles
        sortedAngles_id = sorted(range(len(angles)), key=lambda x: angles[x])
        selectedNeighbour_id = neighbours[sortedAngles_id[0]]
        selectedNeighbour = pcloud.PointAt(selectedNeighbour_id)

        #project selected neighbour to normal on surface
        line = rg.Line(pt, normalsOnSurf[idx], 10)
        finalPt = line.ClosestPoint(selectedNeighbour, False)
        final_pts.append(finalPt)

    return final_pts

def addFirstAndLastCurve(firstCrv, LastCrv, grid):
    gridList = list(grid)
    gridList.insert(0,firstCrv)
    gridList.append(LastCrv)
    return gridList

def addStartandEndPtToList(surface, param, gridDir, ptsList):
    ptsList = list(ptsList)
    if gridDir == 0: #U
        ptStart = surface.PointAt(0, param)
        ptEnd = surface.PointAt(1, param)      
    
    if gridDir == 1: #V
        ptStart = surface.PointAt(param, 0)
        ptEnd = surface.PointAt(param, 1)

    ptsList.insert(0, ptStart)
    ptsList.append(ptEnd)
    return ptsList

def get_UV_points(srfFace, domain):
    Upoints = []
    Vpoints = []

    #get U points
    for u in domain:
        ustrip = []
        vstrip = []
        for v in domain:
            pt = srfFace.PointAt(v,u)
            ustrip.append(pt)
            pt1 = srfFace.PointAt(u,v)
            vstrip.append(pt1)
        
        Upoints.append(ustrip)
        Vpoints.append(vstrip)
    return Upoints, Vpoints

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

class TriangularOptionsDialog(forms.Dialog[bool]):
 
    # Dialog box Class initializer
    def __init__(self):
        super().__init__()
        # Initialize dialog box
        self.Title = 'Triangular surface Options'
        self.Padding = drawing.Padding(10)
        self.Resizable = False
 
        # Surface type
        self.surfacetype_label = forms.Label()
        self.surfacetype_label.Text = "Surface type:"
        self.surfacetype_dropdownlist = forms.DropDown()
        self.surfacetype_dropdownlist.DataStore = ['strict', 'loose']
        self.surfacetype_dropdownlist.SelectedIndex = 0

        #Grid size
        self.gridSize_label = forms.Label()
        self.gridSize_label.Text = "Grid size:"
        self.gridSize_updown = forms.NumericUpDown()
        self.gridSize_updown.DecimalPlaces = 0
        self.gridSize_updown.Increment = 1
        self.gridSize_updown.MaxValue = 100
        self.gridSize_updown.MinValue = 5
        self.gridSize_updown.Value = 10
 
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
        layout.AddRow(self.surfacetype_label, self.surfacetype_dropdownlist)
        layout.AddRow(None) # spacer
        layout.AddRow(self.gridSize_label, self.gridSize_updown)
        layout.AddRow(None) # spacer
        layout.AddRow(self.DefaultButton, self.AbortButton)
 
        # Set the dialog content
        self.Content = layout
 
    # Close button click handler
    def OnCloseButtonClick(self, sender, e):
        self.Close(False)
 
    # OK button click handler
    def OnOKButtonClick(self, sender, e):
        self.Close(True)

def RequestOption():
    dialog = TriangularOptionsDialog()
    rc = dialog.ShowModal(Rhino.UI.RhinoEtoApp.MainWindow)
    if (rc):
        return True, (dialog.surfacetype_dropdownlist.SelectedIndex, int(dialog.gridSize_updown.Value))
    else:
        return False, (dialog.surfacetype_dropdownlist.SelectedIndex, int(dialog.gridSize_updown.Value))
    
################################################################################
    
def runTriangularSurface():
    helper = EscapeKeyHelper()
    
    #USER INPUT
    pcloud_id = rs.GetObject(
        message="Select pointcloud",
        filter=2,
    )
    if pcloud_id == None: return

    curves_id = rs.GetObjects(
        message="Select boundary curves",
        filter=4,
        group=False,
        minimum_count=3,
        maximum_count=3,
        preselect=False
    )
    if curves_id == None: return

    bool_op, option = RequestOption()
    surfaceType, gridSize = option
    if bool_op == False: return

    #----------------------------------------------------------------
    
    geom = []
    ref = rh.DocObjects.ObjRef(sc.doc.ActiveDoc, pcloud_id)
    pcloud = rh.DocObjects.ObjRef.PointCloud(ref)
    geom.append(pcloud)

    curves = []
    for crv in curves_id:
        ref = rh.DocObjects.ObjRef(sc.doc.ActiveDoc, crv)
        c = rh.DocObjects.ObjRef.Curve(ref)
        curves.append(c)
        geom.append(c)

    #join curves to get an ordered direction
    temp_join = rg.Curve.JoinCurves(curves, 0.1, False)[0]
    explodedCrv = temp_join.Explode()
 
    trimmedPatch = True
    patch = rg.Brep.CreatePatch(geometry=geom, startingSurface=None, uSpans=20, vSpans=20,
        trim=trimmedPatch, tangency= False, pointSpacing= 1, flexibility=1,surfacePull=1,
        fixEdges=[False,False,False,False], tolerance=0.01)
    baseSrf = patch.Faces[0]

    if surfaceType == 0:
        #generate subd curves
        explodedHull = get_convexhull(explodedCrv)
        nurbshull = [hc.ToNurbsCurve() for hc in explodedHull]
        averagePt = getAveragePts(nurbshull)
        temp_subD = compute_subd_curves(averagePt, nurbshull)
        projectedSubD = compute_projectedSubd_toBrep(surfaceType, temp_subD, [baseSrf.Brep])

    if surfaceType == 1:
        #generate subd curves
        averagePt = getAveragePts(explodedCrv)
        temp_subD = compute_subd_curves(averagePt, explodedCrv)
        projectedSubD = compute_projectedSubd_toBrep(surfaceType, temp_subD, baseSrf)

    #update subd curves
    orderedSubCrv = reorder_sub_curves(explodedCrv, projectedSubD)
    splitParams = get_split_point_parameters(explodedCrv, orderedSubCrv)
    final_subdCrvs = pull_firstPt_subd_toBoundary(explodedCrv, orderedSubCrv, splitParams)

    #split boundary curve
    boundarySeg = split_boundary(explodedCrv, splitParams)

    #clusters
    clusters = compute_clusters(boundarySeg, final_subdCrvs)

    #network surface
    final_surfaces = compute_network_surface(pcloud, baseSrf, clusters, gridSize)

    #object attributes
    prop = rh.DocObjects.ObjectAttributes()
    prop.ObjectColor = System.Drawing.Color.FromArgb(0, 0, 255)
    prop.ColorSource = rh.DocObjects.ObjectColorSource.ColorFromObject

    #bake object
    [sc.doc.Objects.AddSurface(surface, prop) for surface in final_surfaces]
    print("triangular surface is done!")

################################################################################

runTriangularSurface()