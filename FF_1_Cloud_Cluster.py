
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

import FF_Attributes
    
################################################################################

def runCloudCluster():
    #USER INPUT
    pcloud_id = rs.GetObject(
        message="Select pointcloud",
        filter=2,
    )
    if pcloud_id == None: return

    ################################################################################

    ref = rh.DocObjects.ObjRef(sc.doc.ActiveDoc, pcloud_id)
    pcloud = rh.DocObjects.ObjRef.PointCloud(ref)
    points = rg.PointCloud.GetPoints(pcloud) #convert pcloud to points

    #convert point to numpy array
    ptsArr = []
    for pt in points:
        p = [pt.X, pt.Y, pt.Z]
        ptsArr.append(p)

    hdb = HDBSCAN(alpha= 0.5).fit(ptsArr)
    labels = hdb.labels_

    n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
    n_noise_ = list(labels).count(-1)

    print("Estimated number of clusters: %d" % n_clusters_)
    print("Estimated number of noise points: %d" % n_noise_)

    #clustering
    clusteredPoints = [[] for i in range(n_clusters_)]
    for idx, l in enumerate(labels) :
        if l == -1:
            continue
        else:
            clusteredPoints[l].append(points[idx])

    #object attributes
    prop = FF_Attributes.getObjectProperties()

    for cl in clusteredPoints:
        final_pcl = rg.PointCloud(cl)
        #bake object
        sc.doc.Objects.AddPointCloud(final_pcl, prop)
        
    print("clustering is done!")  

################################################################################

runCloudCluster()