# -*- coding: utf-8 -*-

import numpy as np
import scipy.optimize as optimization
from os import listdir
from os.path import isfile, join

template = """# .PCD v.7 - Point Cloud Data file format
VERSION .7
FIELDS x y z
SIZE 4 4 4
TYPE F F F
COUNT 1 1 1
WIDTH {0}
HEIGHT 1
VIEWPOINT 0 0 0 1 0 0 0
POINTS {0}
DATA ascii
"""

def offset(points, offset_x=0, offset_y=0, offset_z=0):
    return [[_[0]+offset_x, _[1]+offset_x, _[2]+offset_x] for _ in points]


def load(filename):
    points = list()
    if filename.endswith(".pcd"):
        with open(filename, "r") as fp:
            count = 0
            for line in fp:
                count += 1
                if count == 11:
                    break

            for line in fp:
                items = line.split(" ")
                points.append([float(_) for _ in items])
    else:
        with open(filename, "r") as fp:
            for line in fp:
                items = line.split(",")
                points.append([float(_) for _ in items])

    return points


def save(points, filename):
    with open(filename, "w") as fp:
        fp.write(template.format(len(points)))
        for point in points:
            fp.write("{0[0]} {0[1]} {0[2]}\n".format(point))


# axis: x->0, y->1, z->2
def scope_filter(points, h_axis, v_axis=2, safe_dist=1):
    statistic = dict()
    for point in points:
        h_v = point[h_axis]
        v_v = point[v_axis]
        if not statistic.has_key(h_v):
            statistic[h_v] = v_v
        elif v_v > statistic[h_v]:
            statistic[h_v] = v_v
    
    keys = sorted(statistic.keys())
    values = [statistic[key] for key in keys]
  
    if False:
        mean = sum(values) / len(values)
        l_i, r_i = 0, len(values)-1
    
        while values[l_i] < mean:
            l_i += 1

        while values[r_i] < mean:
            r_i -= 1

        sp_points = list()
        #for i in range(len(keys)):
        for i in range(l_i, r_i+1):
            point = [0.0, 0.0, 0.0]
            point[h_axis] = keys[i]
            point[v_axis] = values[i]
            sp_points.append(point)

        a, b = est_line(sp_points, h_axis, v_axis)
        line = lambda x: a*x + b

        print a, b
        t = [line(key) for key in keys]

        import matplotlib.pyplot as plt
        plt.plot(keys, values, keys, t, 'r.')
        plt.show()

    else:
        diffs = [values[i]-values[i+1] for i in range(0, len(values)-1)]
        diffs = [abs(diff) for diff in diffs]
        mean_diff = sum(diffs) / len(diffs)
        
        threshold = 0.9 * mean_diff + 0.1 * max(diffs)
        l_i, r_i = 0, len(diffs)-1
        while diffs[l_i] < threshold:
            l_i += 1
        
        while diffs[r_i] < threshold:
            r_i -= 1

        r_i += 1
       
        temp = values[l_i: r_i+1]
        mean = sum(temp) / len(temp)

        #sp_points = list()
        ##for i in range(len(keys)):
        #for i in range(l_i, r_i+1):
        #    point = [0.0, 0.0, 0.0]
        #    point[h_axis] = keys[i]
        #    point[v_axis] = values[i]
        #    sp_points.append(point)

        #a, b = est_line(sp_points, h_axis, v_axis)
        #line = lambda x: a*x + b

        #print a, b
        #t = [line(key) for key in keys]

        import matplotlib.pyplot as plt
        #plt.plot(keys, values, keys, t, 'r.')
        plt.plot(keys, values, keys[l_i: r_i+1], temp, 'r.')
        plt.show()
    
    l_v = keys[l_i]
    r_v = keys[r_i]

    while keys[l_i] > l_v-safe_dist and l_i > 0:
        l_i -= 1

    while keys[r_i] < r_v+safe_dist and r_i < len(keys)-1:
        r_i += 1

    l_v = keys[l_i]
    r_v = keys[r_i]

    return [point for point in points if point[h_axis] >= l_v and point[h_axis] <= r_v]


def xy_filter(points):
    points = scope_filter(points, 0)
    return scope_filter(points, 1)
    

def z_filter(points):
    all_z = [_[2] for _ in points]
    max_z = max(all_z)
    min_z = min(all_z)
    mean_z = sum(all_z) / len(all_z)
    print "Min:{0}, Max:{1}, Mean:{2}".format(min_z, max_z, mean_z)

    #threshold = mean_z + 0.5*(max_z-mean_z)
    threshold = mean_z - 0.0*(mean_z-min_z)
    #threshold = max_z
    points = map(lambda x: x if x[2] >= threshold else [x[0], x[1], min_z], points)
    return points


# [[x1,y1], [x2,y2], ...]
def est_line(points, h_axis, v_axis):
    ydata = np.array([point[v_axis] for point in points])
    # Provide data as design matrix: straight line with a=0 and b=1 plus some noise.
    xdata = np.transpose(np.array([
        np.array([point[h_axis] for point in points]),
        [1 for i in range(len(points))]
    ]))

    x0 = np.array([0.0, 0.0])
    # params mean x => [a, b]
    x = optimization.leastsq(
        lambda params, xdata, ydata: (ydata - np.dot(xdata, params)),
        x0, args=(xdata, ydata)
    )

    # a, b
    return x[0]


#points = [(1, 0, 2+10), (2, 0, 4+10), (3, 0, 6+10)]
#print est_line(points, 0, 2)
#exit()

path = "."
files = [f for f in listdir(path) if isfile(join(path, f))]
files = [f for f in files if f.endswith(".csv")]

out_dir = "crop_filter/"

files = ["台幣1元 反面.csv", "台幣1元 正面.csv", 
         "台幣10元 反面.csv", "台幣10元 正面.csv",
         "台幣50元 反面.csv", "台幣50元 正面.csv",
         "外國幣-1 反面.csv", "外國幣-1 正面.csv",
         "外國幣- 3反面.csv", "外國幣- 3正面.csv",
         "外國幣- 4反面.csv", "外國幣- 4正面.csv"
        ]

for f in files:
    print f
    
    points = load(f)

    origin = str(len(points))
    points = xy_filter(points)
    mod = str(len(points))
    print origin + "V.S." + mod

    print "------------------------------>"
    continue
    points = z_filter(points)
    save(points, out_dir + (f[:-4]+".pcd"))
    print "------------------------------>"
