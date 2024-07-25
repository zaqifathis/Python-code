
#! python3

import rhinoscriptsyntax as rs
import Rhino as rh
import Rhino.Geometry as rg
import scriptcontext as sc

import Rhino.UI
import Eto.Drawing as drawing
import Eto.Forms as forms

import System
import System.Collections.Generic

import FF_Attributes

################################################################################

class PatchOptionsDialog(forms.Dialog[bool]):
 
    # Dialog box Class initializer
    def __init__(self):
        super().__init__()
        # Initialize dialog box
        self.Title = 'Patch Options'
        self.Padding = drawing.Padding(10)
        self.Resizable = False
 
        # Grid U
        self.gridU_label = forms.Label()
        self.gridU_label.Text = 'U-Grid size:'
        self.gridU_textbox = forms.TextBox()
        self.gridU_textbox.Text = ""

        # Grid V
        self.gridV_label = forms.Label()
        self.gridV_label.Text = 'V-Grid size:'
        self.gridV_textbox = forms.TextBox()
        self.gridV_textbox.Text = ""

        #flexibility
        self.flex_label = forms.Label()
        self.flex_label.Text = "Flexibility:"
        self.flex_updown = forms.NumericUpDown()
        self.flex_updown.DecimalPlaces = 1
        self.flex_updown.Increment = 0.1
        self.flex_updown.MaxValue = 100
        self.flex_updown.MinValue = 0.1
        self.flex_updown.Value = 100
 
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
        layout.AddRow(self.gridU_label, self.gridU_textbox)
        layout.AddRow(self.gridV_label, self.gridV_textbox)
        layout.AddRow(None) # spacer
        layout.AddRow(self.flex_label, self.flex_updown)
        layout.AddRow(None) # spacer
        layout.AddRow(self.DefaultButton, self.AbortButton)
 
        # Set the dialog content
        self.Content = layout
 
    # Get the value of the textbox
    def GetGrid_U(self):
        return self.gridU_textbox.Text
    
    def GetGrid_V(self):
        return self.gridV_textbox.Text
 
    # Close button click handler
    def OnCloseButtonClick(self, sender, e):
        self.Close(False)
 
    # OK button click handler
    def OnOKButtonClick(self, sender, e):
        if self.gridU_textbox.Text == "" or self.gridV_textbox.Text == "":
            self.Close(False)
        else:
            self.Close(True)

def RequestOption():
    dialog = PatchOptionsDialog()
    rc = dialog.ShowModal(Rhino.UI.RhinoEtoApp.MainWindow)
    if (rc):
        return True, (int(dialog.GetGrid_U()), int(dialog.GetGrid_V()), dialog.flex_updown.Value )
    else:
        return False, (dialog.GetGrid_U(), dialog.GetGrid_V(), dialog.flex_updown.Value)
    
################################################################################

def GetRebuildCurve(curves, gridU, gridV):
    temp_join = rg.Curve.JoinCurves(curves, 0.1, False)[0]
    explodedCrv = temp_join.Explode()

    rebuildCrv = []
    for idx, crv in enumerate(explodedCrv):
        if idx == 0 or idx == 2:
            params = crv.DivideByCount(gridU, True)
        if idx == 1 or idx == 3:
            params = crv.DivideByCount(gridV, True)

        pts = [crv.PointAt(p) for p in params]
        rcrv = rg.Curve.CreateInterpolatedCurve(pts, 3)
        rebuildCrv.append(rcrv)

    return rebuildCrv

################################################################################

def RunPatchSurface():

    #USER INPUT
    pcloud_id = rs.GetObject(
        message="Select pointcloud",
        filter=2,
    )
    if pcloud_id == None: return

    curves_id = rs.GetObjects(
        message="Select 4 curves",
        filter=4,
        group=False,
        minimum_count=4,
        maximum_count=4,
        preselect=False
    )
    if curves_id == None: return

    bool_op, option = RequestOption()
    gridU, gridV, flexibility = option
    if bool_op == False: return

    #----------------------------------------------------------------

    geom = []
    ref = rh.DocObjects.ObjRef(sc.doc.ActiveDoc, pcloud_id)
    pcloud = rh.DocObjects.ObjRef.PointCloud(ref)
    geom.append(pcloud)

    tempCrvs = []
    for crv in curves_id:
        ref = rh.DocObjects.ObjRef(sc.doc.ActiveDoc, crv)
        c = rh.DocObjects.ObjRef.Curve(ref)
        tempCrvs.append(c)
   
    finalCurves = GetRebuildCurve(tempCrvs, gridU, gridV)
    init_srf = rg.Brep.CreateEdgeSurface(finalCurves).Faces[0]

    #patch surface
    fixedEdges = [True, True, True, True]
    patch = rg.Brep.CreatePatch(geom, init_srf, gridU, gridV, True, False, 0.1, flexibility, 0.001,fixedEdges, 100 )

    #object attributes
    prop = FF_Attributes.getObjectProperties()

    sc.doc.Objects.AddBrep(patch, prop)
    print("patch surface is done!")

################################################################################

RunPatchSurface()