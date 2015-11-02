import plot as Plot
import numpy as np
import os

class PlotRes:
    def __init__(self):
        state_points = []
        velocity_points = []
        with open("somefile.txt", 'r') as f:
            for line in f.readlines():
                state_line = line.split(" ")[0:2]
                print line
                vel_line = line.split(" ")[2:4]
                state_points.append(np.array([float(state_line[i]) for i in xrange(len(state_line))]))
                velocity_points.append(np.array([float(vel_line[i]) for i in xrange(len(vel_line))]))
        x_min = 1000
        x_max = -1000
        y_min = 1000
        y_max = -1000
        for point in state_points:
            if point[0] < x_min:
                x_min = point[0]
            if point[0] > x_max:
                x_max = point[0]
            if point[1] < y_min:
                y_min = point[1]
            if point[1] > y_max:
                y_max = point[1]
        Plot.plot_2d_points(np.array(state_points), xrange=[x_min, x_max], yrange=[y_min, y_max])
        #x_min = 1.1 * x_min
        #x_max = 1.1 * x_max 
        #y_min = 1.1 * y_min
        #y_max = y_max * 1.1 
        x_min = 1000
        x_max = -1000
        y_min = 1000
        y_max = -1000
        for point in velocity_points:
            if point[0] < x_min:
                x_min = point[0]
            if point[0] > x_max:
                x_max = point[0]
            if point[1] < y_min:
                y_min = point[1]
            if point[1] > y_max:
                y_max = point[1]
        Plot.plot_2d_points(np.array(velocity_points), xrange=[x_min, x_max], yrange=[y_min, y_max])

if __name__ == "__main__":
    PlotRes()
    