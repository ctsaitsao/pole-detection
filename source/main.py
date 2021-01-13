import numpy as np
import matplotlib.pyplot as plt
import sklearn
import collections
# %matplotlib inline
import open3d as o3d
import csv
import sys
import utils

latitude = []
longitude = []
altitude = []
intensity = []

read_file("../data/final_project_point_cloud.fuse")
cartesian = np.ones((len(latitude),4))

for i in range(0,len(latitude)):
    cartesian[i] = lat_to_cartesian(latitude[i],longitude[i],altitude[i],intensity[i])

point_cloud_obj(cartesian,"point_cloud_all.obj")

point_cloud = o3d.geometry.PointCloud()
point_cloud.points = o3d.utility.Vector3dVector(cartesian[ : , 0:3])
intensities = np.zeros((np.size(cartesian[ : , 3]),3))
point_cloud.colors = o3d.utility.Vector3dVector(intensities)
o3d.io.write_point_cloud("../results/point_cloud_all.ply", point_cloud)
o3d.visualization.draw_geometries([point_cloud])

down_voxel = point_cloud.voxel_down_sample(voxel_size=0.8)
o3d.io.write_point_cloud("../results/ds_by_vox.ply", down_voxel)
down_uniform = point_cloud.uniform_down_sample(every_k_points=5)
o3d.io.write_point_cloud("../results/ds_uni.ply", down_uniform)
down_stats, ind = down_voxel.statistical_outlier_removal(nb_neighbors=50,std_ratio=5.0)
o3d.io.write_point_cloud("../results/ds_no_outlier.ply", down_stats)

plana_obj = planar_filter(cartesian, grid_size=0.5, lower_bound=6, filter_axis = 'z')
cartesian_planar = cartesian[plana_obj, :]

lim = 10
cartesian_scaled = sklearn.cluster.preprocessing.scale(cartesian_planar[:,0:2])
cartesian_norm = sklearn.cluster.preprocessing.normalize(cartesian_planar[:, 0:2], norm='l2')

fittings = []
for i in range(lim):
    fittings.append((sklearn.cluster.KMeans(n_clusters=i+1).fit(cartesian_scaled)).inertia_)

fittings_diff = np.zeros((lim-1,1))
for i in range(lim-1):
    fittings_diff[i] = fittings[i]/fittings[i+1]

x = range(2,lim+1)
plt.plot(x,fittings_diff)
n_clusters = np.argmax(fittings_diff)+2

cartesian_model = sklearn.cluster.KMeans(n_clusters=n_clusters).fit_predict(cartesian_planar[:,0:2])
verif = o3d.geometry.PointCloud()

verif.points = o3d.utility.Vector3dVector(cartesian_planar[ : , 0:3])
color_list = np.array([[0,0,0],[255,0,255],[0,255,255],[255,0,0],[0,255,0],[0,0,255],[255,128,0],[102,0,51],[0,128,255],[0,0,0],[255,255,0],[255,0,255],[0,255,255],[255,0,0],[0,255,0],[0,0,255],[128,255,0],[255,128,0],[0,128,255],[0,0,0]])
color_list = color_list/255.0
intensities = np.zeros((np.size(cartesian_planar[ : , 3]),3))

for i in range(len(cartesian_model)):
    intensities[i] = color_list[cartesian_model[i]]

point_cloud.colors = o3d.utility.Vector3dVector(intensities)
o3d.io.write_point_cloud("../results/final_results.ply", point_cloud)
o3d.visualization.draw_geometries([point_cloud])

res_0 = np.zeros((1,4))
res_1 = np.zeros((1,4))
res_2 = np.zeros((1,4))
res_3 = np.zeros((1,4))
res_4 = np.zeros((1,4))
res_5 = np.zeros((1,4))
res_6 = np.zeros((1,4))
res_7 = np.zeros((1,4))

for i in range(len(cartesian_model)):
    if(cartesian_model[i]==0):
        if(cartesian_planar[i,2]>227.0):
            res_0 = np.append(res_0,cartesian_planar[i,:].reshape((1,4)),axis=0)
    elif(cartesian_model[i]==1):
        if(cartesian_planar[i,2]>227.0):
            res_1 = np.append(res_1,cartesian_planar[i,:].reshape((1,4)),axis=0)
    elif(cartesian_model[i]==2):
        if(cartesian_planar[i,2]>227.0):
            res_2 = np.append(res_2,cartesian_planar[i,:].reshape((1,4)),axis=0)
    elif(cartesian_model[i]==3):
        if(cartesian_planar[i,2]>227.0 and cartesian_planar[i,0]>4363850.1):
            res_3 = np.append(res_3,cartesian_planar[i,:].reshape((1,4)),axis=0)
    elif(cartesian_model[i]==4):
        if(cartesian_planar[i,2]>227.0):
            res_4 = np.append(res_4,cartesian_planar[i,:].reshape((1,4)),axis=0)
    elif(cartesian_model[i]==5):
        if(cartesian_planar[i,2]>227.0):
            res_5 = np.append(res_5,cartesian_planar[i,:].reshape((1,4)),axis=0)
    elif(cartesian_model[i]==6):
        if(cartesian_planar[i,2]>227.0):
            res_6 = np.append(res_6,cartesian_planar[i,:].reshape((1,4)),axis=0)
    elif(cartesian_model[i]==7):
        if(cartesian_planar[i,2]>227.0 and cartesian_planar[i,0]>4363906.0):
            res_7 = np.append(res_7,cartesian_planar[i,:].reshape((1,4)),axis=0)

res_0 = res_0[1:,:]
res_1 = res_1[1:,:]
res_2 = res_2[1:,:]
res_3 = res_3[1:,:]
res_4 = res_4[1:,:]
res_5 = res_5[1:,:]
res_6 = res_6[1:,:]
res_7 = res_7[1:,:]
            
np.set_printoptions(threshold=sys.maxsize)

res_poly_0 = np.poly1d(np.polyfit(res_0[:,2],res_0[:,0],5))
res_poly_1 = np.poly1d(np.polyfit(res_1[:,2],res_1[:,0],5))
res_poly_2 = np.poly1d(np.polyfit(res_2[:,2],res_2[:,0],5))
res_poly_3 = np.poly1d(np.polyfit(res_3[:,2],res_3[:,0],5))
res_poly_4 = np.poly1d(np.polyfit(res_4[:,2],res_4[:,0],5))
res_poly_5 = np.poly1d(np.polyfit(res_5[:,2],res_5[:,0],5))
res_poly_6 = np.poly1d(np.polyfit(res_6[:,2],res_6[:,0],5))
res_poly_7 = np.poly1d(np.polyfit(res_7[:,2],res_7[:,0],5))
res_poly_span = np.linspace(227,234.5,100)

cartesian_ld = np.asarray(down_voxel.points)

o3d.geometry.estimate_normals(verif,search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.2,max_nn=10)) 
o3d.visualization.draw_geometries([verif]) 