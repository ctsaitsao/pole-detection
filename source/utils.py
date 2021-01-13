import numpy as np
import csv
import collections

def read_file(file_name):
    global latitude, longitude, altitude, intensity
    with open(file_name) as csvfile:
        r = csv.r(csvfile, delimiter =' ')
        for row in r:
            latitude.append(float(row[0]))
            longitude.append(float(row[1]))
            altitude.append(float(row[2]))
            intensity.append(float(row[3]))

def point_cloud_obj(array, name):
    point_cloud_obj = open(name,"w")
    for j in array:
        point_cloud_obj.write("v "+str(j[0])+" "+str(j[1])+" "+str(j[2])+"\n")
    point_cloud_obj.close

def lat_to_cartesian(latitude, longitude, altitude, intensity):
    earth_rad = (6378.137*1000)
    latitude_cos = np.cos(latitude*np.pi/180)
    latitude_sin = np.sin(latitude*np.pi/180)
    longitude_cos = np.cos(longitude*np.pi/180)
    longitude_sin = np.sin(longitude*np.pi/180)
    f = 1.0 / 298.257224
    C = 1.0/ np.sqrt(latitude_cos**2 + ((1-f)**2)*((latitude_sin)**2))
    S = ((1.0 - f)**2)*(C)
    x = (earth_rad*C)*latitude_cos*longitude_cos
    y = (earth_rad*C)*latitude_cos*longitude_sin
    z = altitude
    return x, y, z, intensity

def planar_filter(cartesian, grid_size, lower_bound, ax):
    ratio = 1 / grid_size
    if ratio > 0:
        ratio = int(ratio)
    xs = np.round(cartesian[:, 0] * ratio)
    ys = np.round(cartesian[:, 1] * ratio)
    zs = np.round(cartesian[:, 2] * ratio)
    delta_x = collections.defaultdict(list)
    delta_y = collections.defaultdict(list)
    delta_z = collections.defaultdict(list)
    for i in range(len(cartesian)):
        xc = xs[i]
        yc = ys[i]
        zc = zs[i]
        x = cartesian[i, 0]
        y = cartesian[i, 1]
        z = cartesian[i, 2]
        delta_z[(xc, yc)].append((i, z))
        delta_y[(xc, zc)].append((i, y))
        delta_x[(yc, zc)].append((i, x))
    planar_res = []
    if ax == 'z':
        for xy, i_and_z in delta_z.items():
            iterator, z_list = zip(*i_and_z)
            if max(z_list) - min(z_list) >= lower_bound:
                planar_res.extend(iterator)
    if ax == 'y':
        for xz, i_and_y in delta_y.items():
            iterator, list_y = zip(*i_and_y)
            if max(list_y) - min(list_y) <= lower_bound:
                planar_res.extend(iterator)
    if ax == 'x':
        for yz, i_and_x in delta_x.items():
            iterator, list_x = zip(*i_and_x)
            if max(list_x) - min(list_x) <= lower_bound:
                planar_res.extend(iterator)
    return planar_res
