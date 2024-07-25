#! python3

import rhinoscriptsyntax as rs
import scriptcontext as sc
import math

import System
import System.Collections.Generic
import Rhino as rh
import Rhino.Geometry as rg


################################################################################

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

def compute_network_surface(initSurf, curves, gridSize):
    #get domain
    domain = []
    for i in range(1, gridSize):
        step = 1/gridSize
        domain.append(step * i)

    #create edge surface
    edgeSurf = rg.Brep.CreateEdgeSurface(curves)
    if edgeSurf.IsSurface:
        srfFace = edgeSurf.Faces[0]
    else:
        print("ERROR: edge surface did not work!")
        return
    
    #reparameterize surface domain
    srfFace.SetDomain(0, rg.Interval(0,1)) #U
    srfFace.SetDomain(1, rg.Interval(0,1)) #V

    #uv points
    Upoints, Vpoints = get_UV_points(srfFace, domain)

    #pull points to init surface
    pulledUpts = [initSurf.PullPointsToFace(u_pts, 0.1) for u_pts in Upoints]
    pulledVpts = [initSurf.PullPointsToFace(v_pts, 0.1) for v_pts in Vpoints]

    #add first and end points from boundary curves
    final_U_Pts = [addStartandEndPtToList(srfFace, domain[idx], 0, upts) for idx, upts in enumerate(pulledUpts)]
    final_V_Pts = [addStartandEndPtToList(srfFace, domain[idx], 1, vpts) for idx, vpts in enumerate(pulledVpts)]

    #interpolate curves
    grid_U_curves = [rg.Curve.CreateInterpolatedCurve(upts, 3) for upts in final_U_Pts]
    grid_V_curves = [rg.Curve.CreateInterpolatedCurve(vpts, 3) for vpts in final_V_Pts]

    #add first and last curves to the list
    #from bondary curves
    edges = edgeSurf.Edges
    final_U_curves = addFirstAndLastCurve(edges[1], edges[3], grid_U_curves)
    final_V_curves = addFirstAndLastCurve(edges[0], edges[2],  grid_V_curves)

    #create network surface
    curves = final_U_curves + final_V_curves
    surface = rg.NurbsSurface.CreateNetworkSurface(curves, 1, 0.01, 0.01, 0.01)[0]

    return surface

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

def runNetworkSurface():
    helper = EscapeKeyHelper()

    #USER INPUT
    pcloud_id = rs.GetObject(
        message="Select pointcloud",
        filter=2,
    )
    if pcloud_id == None: return

    curves_id = rs.GetObjects(
        message="Select 4 boundary curves",
        filter=4,
        group=False,
        minimum_count=4,
        maximum_count=4,
        preselect=False
    )
    if curves_id == None: return

    gridSize = rs.GetInteger("grid size", 10, 8, 1000)
    if (helper.EscapeKeyPressed):return 

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
        trim=trimmedPatch, tangency= False, pointSpacing= 0.01, flexibility=1,surfacePull=1,
        fixEdges=[False,False,False,False], tolerance=0.01)
    initSurf = patch.Faces[0]

    #network surface
    final_surface = compute_network_surface(initSurf, explodedCrv, gridSize)
    if final_surface == None:
        print("No network surface is created")
        return

    #object attributes
    prop = rh.DocObjects.ObjectAttributes()
    prop.ObjectColor = System.Drawing.Color.FromArgb(0, 0, 255)
    prop.ColorSource = rh.DocObjects.ObjectColorSource.ColorFromObject

    #bake object
    sc.doc.Objects.AddSurface(final_surface, prop)
    print("network surface is done!")

################################################################################

runNetworkSurface()