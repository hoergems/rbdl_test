import plot as Plot
import numpy as np
import os

class PlotRes:
    def __init__(self):
        points = []
        with open("somefile.txt", 'r') as f:
            for line in f.readlines():
                line = line.split(" ")[0:2]
                points.append(np.array([float(line[i]) for i in xrange(len(line))]))
        x_min = 1000
        x_max = -1000
        y_min = 1000
        y_max = -1000
        for point in points:
            if point[0] < x_min:
                x_min = point[0]
            if point[0] > x_max:
                x_max = point[0]
            if point[1] < y_min:
                y_min = point[1]
            if point[1] > y_max:
                y_max = point[1]
        #x_min = 1.1 * x_min
        #x_max = 1.1 * x_max 
        #y_min = 1.1 * y_min
        #y_max = y_max * 1.1  
        print "x_min " + str(x_min)
        print "x_max " + str(x_max)
        print "y_min " + str(y_min) 
        print "y_max " + str(y_max)      
        Plot.plot_2d_points(np.array(points), xrange=[x_min, x_max], yrange=[y_min, y_max])

if __name__ == "__main__":
    PlotRes()
    