#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d, CubicSpline
import sys

path_distance_away = 0.05

def straight_line(utm_coords):
    distance = np.sqrt((utm_coords[3][0]-utm_coords[0][0])**2 + (utm_coords[3][1]-utm_coords[0][1])**2)
    points_count = int(distance / path_distance_away)
    points = []
    
    dx = (utm_coords[3][0] - utm_coords[0][0]) / points_count
    dy = (utm_coords[3][1] - utm_coords[0][1]) / points_count
    points_x = utm_coords[0][0]
    points_y = utm_coords[0][1]
    
    with open('straight_line.txt','w') as file:
        file.write(str(points_x)+","+str(points_y)+"\n")
        for i in range(points_count):
            points_x = points_x + dx
            points_y = points_y + dy
            file.write(str(points_x)+","+str(points_y)+"\n")
            points.append([points_x,points_y])
            # print("points:",points[i])
    
def circle_line(utm_coords):
    radius = np.sqrt((utm_coords[11][0]-utm_coords[12][0])**2 + (utm_coords[11][1]-utm_coords[12][1])**2)
    points_count = int((np.pi * radius * 2) / path_distance_away)
    transported_coords = [utm_coords[12][0] - utm_coords[11][0],utm_coords[12][1] - utm_coords[11][1]]
    delta_theta = 2 * np.pi / points_count

    points = []
    theta = 0
    
    with open('circle_line.txt','w') as file:
        file.write(str(transported_coords[0] + utm_coords[11][0]) + "," + str(transported_coords[1] + utm_coords[11][1]) + "\n")
        for i in range(points_count):
            theta = theta + delta_theta
            points.append(np.array([[np.cos(theta),-np.sin(theta)],[np.sin(theta),np.cos(theta)]]) @ np.array(transported_coords))
            point_x = str(points[i][0] + utm_coords[11][0]); point_y = str(points[i][1] + utm_coords[11][1])
            file.write(point_x + "," + point_y + "\n")
    
def U_turn(utm_coords):
    utm_coords_cut = utm_coords[3:7]
    print(utm_coords)
    
    # Linear length along the line:
    distance = np.cumsum( np.sqrt(np.sum( np.diff(utm_coords_cut, axis=0)**2, axis=1 )) )
    distance = np.insert(distance, 0, 0)/distance[-1]

    # Interpolation for different methods:
    interpolations_methods = ['slinear', 'quadratic', 'cubic']
    alpha = np.linspace(0, 1, 75)

    interpolated_points = {}
    for method in interpolations_methods:
        interpolator =  interp1d(distance, utm_coords_cut, kind=method, axis=0)
        interpolated_points[method] = interpolator(alpha)

    # Graph:
    print(interpolated_points)
    # plt.figure(figsize=(7,7))
    # for method_name, curve in interpolated_points.items():
    #     plt.plot(*np.transpose(curve), '-', label=method_name)

    # plt.plot(*np.transpose(utm_coords_cut), 'ok', label='original points')
    # plt.axis('equal'); plt.legend(); plt.xlabel('x'); plt.ylabel('y')
    # plt.show()
    
    U_points = interpolated_points['cubic']
    
    distance = np.sqrt((utm_coords[3][0]-utm_coords[2][0])**2 + (utm_coords[3][1]-utm_coords[2][1])**2)
    points_count = int(distance / path_distance_away)
    points = []
    
    dx = (utm_coords[3][0] - utm_coords[2][0]) / points_count
    dy = (utm_coords[3][1] - utm_coords[2][1]) / points_count
    points_x = utm_coords[2][0]
    points_y = utm_coords[2][1]
    
    with open('U_turn.txt','w') as file:
        file.write(str(points_x)+","+str(points_y)+"\n")
        for i in range(points_count):
            points_x = points_x + dx
            points_y = points_y + dy
            file.write(str(points_x)+","+str(points_y)+"\n")
            points.append([points_x,points_y])
            
        distance = np.sqrt((utm_coords[7][0]-utm_coords[6][0])**2 + (utm_coords[7][1]-utm_coords[6][1])**2)
        points_count = int(distance / path_distance_away)
        points = []
        
        dx = (utm_coords[7][0] - utm_coords[6][0]) / points_count
        dy = (utm_coords[7][1] - utm_coords[6][1]) / points_count
        points_x = utm_coords[6][0]
        points_y = utm_coords[6][1]
        
        for i in range(len(U_points)):
            file.write(str(U_points[i][0]) + "," + str(U_points[i][1]) + "\n")
    
        file.write(str(points_x)+","+str(points_y)+"\n")
        for i in range(points_count):
            points_x = points_x + dx
            points_y = points_y + dy
            file.write(str(points_x)+","+str(points_y)+"\n")
            points.append([points_x,points_y])

def gym_path(utm_coords):
    bottom_radius = np.sqrt((utm_coords[0][0]-utm_coords[1][0])**2 + (utm_coords[0][1]-utm_coords[1][1])**2) / 2
    bottom_center = [(utm_coords[0][0] + utm_coords[1][0])/2,(utm_coords[0][1] + utm_coords[1][1])/2]
    bottom_circle_point_counts = int((np.pi * bottom_radius) / path_distance_away)
    bottom_transported_coords = [utm_coords[0][0]-bottom_center[0],utm_coords[0][1]-bottom_center[1]]
    bottom_delta_theta = np.pi / bottom_circle_point_counts
    bottom_arange = np.arange(np.pi, 2*np.pi, bottom_delta_theta)

    top_radius = np.sqrt((utm_coords[8][0]-utm_coords[9][0])**2 + (utm_coords[8][1]-utm_coords[9][1])**2) / 2
    top_center = [(utm_coords[8][0] + utm_coords[9][0])/2,(utm_coords[8][1] + utm_coords[9][1])/2]
    top_circle_point_counts = int((np.pi * top_radius) / path_distance_away)
    top_transported_coords = [utm_coords[9][0]-top_center[0],utm_coords[9][1]-top_center[1]]
    top_delta_theta = np.pi / top_circle_point_counts
    top_arange = np.arange(np.pi, 2*np.pi, top_delta_theta)

    straight_1_distance = np.sqrt((utm_coords[1][0]-utm_coords[9][0])**2 + (utm_coords[1][1]-utm_coords[9][1])**2)
    straight_1_points_count = int(straight_1_distance / path_distance_away)
    straight_1_dx = (utm_coords[9][0] - utm_coords[1][0]) / straight_1_points_count
    straight_1_dy = (utm_coords[9][1] - utm_coords[1][1]) / straight_1_points_count
    straight_1_points_x = utm_coords[9][0]
    straight_1_points_y = utm_coords[9][1]

    straight_2_distance = np.sqrt((utm_coords[0][0]-utm_coords[2][0])**2 + (utm_coords[0][1]-utm_coords[2][1])**2)
    straight_2_points_count = int(straight_2_distance / path_distance_away)
    straight_2_dx = (utm_coords[2][0] - utm_coords[0][0]) / straight_2_points_count
    straight_2_dy = (utm_coords[2][1] - utm_coords[0][1]) / straight_2_points_count
    straight_2_points_x = utm_coords[0][0]
    straight_2_points_y = utm_coords[0][1]

    straight_3_distance = np.sqrt((utm_coords[4][0]-utm_coords[5][0])**2 + (utm_coords[4][1]-utm_coords[5][1])**2)
    straight_3_points_count = int(straight_3_distance / path_distance_away)
    straight_3_dx = (utm_coords[5][0] - utm_coords[4][0]) / straight_3_points_count
    straight_3_dy = (utm_coords[5][1] - utm_coords[4][1]) / straight_3_points_count
    straight_3_points_x = utm_coords[4][0]
    straight_3_points_y = utm_coords[4][1]

    straight_4_distance = np.sqrt((utm_coords[7][0]-utm_coords[8][0])**2 + (utm_coords[7][1]-utm_coords[8][1])**2)
    straight_4_points_count = int(straight_4_distance / path_distance_away)
    straight_4_dx = (utm_coords[8][0] - utm_coords[7][0]) / straight_4_points_count
    straight_4_dy = (utm_coords[8][1] - utm_coords[7][1]) / straight_4_points_count
    straight_4_points_x = utm_coords[7][0]
    straight_4_points_y = utm_coords[7][1]

    line_changer_1_utm_coords_cut = utm_coords[2:5]
    line_changer_1_x = np.flip(np.transpose(line_changer_1_utm_coords_cut)[0])
    line_changer_1_y = np.transpose(line_changer_1_utm_coords_cut)[1]
    line_changer_1_count = int((np.sqrt((line_changer_1_x[0] - line_changer_1_x[2]) ** 2 + (line_changer_1_y[0] - line_changer_1_y[2]) ** 2)) / path_distance_away)

    line_changer_1_start_slope = 3; line_changer_1_end_slope = 3
    line_changer_1_cs = CubicSpline(line_changer_1_x, line_changer_1_y, bc_type=((1, line_changer_1_start_slope), (1, line_changer_1_end_slope)))
    line_changer_1_new_x = np.linspace(line_changer_1_x[0], line_changer_1_x[2], line_changer_1_count)
    line_changer_1_new_y = line_changer_1_cs(line_changer_1_new_x)
    line_changer_1_new_x = np.flip(line_changer_1_new_x)

    # print(line_changer_1_new_x, line_changer_1_new_y)
    # plt.plot(line_changer_1_x, line_changer_1_y, 'ro', label='Original Points')
    # plt.plot(line_changer_1_new_x, line_changer_1_new_y, label='Interpolated Curve')
    # plt.xlabel('X')
    # plt.ylabel('Y')
    # plt.title('Cubic Interpolation with Boundary Conditions')
    # plt.legend()
    # plt.grid(True)
    # plt.show()

    line_changer_2_utm_coords_cut = utm_coords[5:8]
    line_changer_2_x = np.transpose(line_changer_2_utm_coords_cut)[0]
    line_changer_2_y = np.transpose(line_changer_2_utm_coords_cut)[1]
    line_changer_2_count = int((np.sqrt((line_changer_2_x[0] - line_changer_2_x[2]) ** 2 + (line_changer_2_y[0] - line_changer_2_y[2]) ** 2)) / path_distance_away)

    line_changer_2_start_slope = 3; line_changer_2_end_slope = 3
    line_changer_2_cs = CubicSpline(line_changer_2_x, line_changer_2_y, bc_type=((1, line_changer_2_start_slope), (1, line_changer_2_end_slope)))
    line_changer_2_new_x = np.linspace(line_changer_2_x[0], line_changer_2_x[2], line_changer_2_count)
    line_changer_2_new_y = line_changer_2_cs(line_changer_2_new_x)

    # print(line_changer_2_new_x, line_changer_2_new_y)
    # plt.plot(line_changer_2_x, line_changer_2_y, 'ro', label='Original Points')
    # plt.plot(line_changer_2_new_x, line_changer_2_new_y, label='Interpolated Curve')
    # plt.xlabel('X')
    # plt.ylabel('Y')
    # plt.title('Cubic Interpolation with Boundary Conditions')
    # plt.legend()
    # plt.grid(True)
    # plt.show()

    with open('gym_path.txt','w') as file:
        #right staright line 1 point 1-3
        file.write(str(straight_2_points_x)+","+str(straight_2_points_y)+"\n")
        for i in range(straight_2_points_count):
            straight_2_points_x = straight_2_points_x + straight_2_dx
            straight_2_points_y = straight_2_points_y + straight_2_dy
            file.write(str(straight_2_points_x)+","+str(straight_2_points_y)+"\n")

        # line changer 1 point 3-5
        for i in range(len(line_changer_1_new_x)):
            file.write(str(line_changer_1_new_x[i])+","+str(line_changer_1_new_y[i])+"\n")

        # right straight line 2 point 5-6
        file.write(str(straight_3_points_x)+","+str(straight_3_points_y)+"\n")
        for i in range(straight_3_points_count):
            straight_3_points_x = straight_3_points_x + straight_3_dx
            straight_3_points_y = straight_3_points_y + straight_3_dy
            file.write(str(straight_3_points_x)+","+str(straight_3_points_y)+"\n")

        #line changer 2 point 6-8
        for i in range(len(line_changer_2_new_x)):
            file.write(str(line_changer_2_new_x[i])+","+str(line_changer_2_new_y[i])+"\n")

        #right straight line 3 point 8-9
        file.write(str(straight_4_points_x)+","+str(straight_4_points_y)+"\n")
        for i in range(straight_4_points_count):
            straight_4_points_x = straight_4_points_x + straight_4_dx
            straight_4_points_y = straight_4_points_y + straight_4_dy
            file.write(str(straight_4_points_x)+","+str(straight_4_points_y)+"\n")

        # top half circle point 9-10
        points = []
        for i in top_arange:
            points.append(np.array([[np.cos(i),-np.sin(i)],[np.sin(i),np.cos(i)]]) @ np.array(top_transported_coords))
        for i in range(len(points)):
            point_x = str(points[i][0] + top_center[0]); point_y = str(points[i][1] + top_center[1])
            file.write(point_x + "," + point_y + "\n")
        
        #left straight line point 10-2
        file.write(str(straight_1_points_x)+","+str(straight_1_points_y)+"\n")
        for i in range(straight_1_points_count):
            straight_1_points_x = straight_1_points_x - straight_1_dx
            straight_1_points_y = straight_1_points_y - straight_1_dy
            file.write(str(straight_1_points_x)+","+str(straight_1_points_y)+"\n")

        # bottom half circle point 2-1
        points = []
        for i in bottom_arange:
            points.append(np.array([[np.cos(i),-np.sin(i)],[np.sin(i),np.cos(i)]]) @ np.array(bottom_transported_coords))
        for i in range(len(points)):
            point_x = str(points[i][0] + bottom_center[0]); point_y = str(points[i][1] + bottom_center[1])
            file.write(point_x + "," + point_y + "\n")




def show_line(file_name,num):
    # Open the file for reading
    with open(file_name, 'r') as file:
        # Read all lines from the file
        lines = file.readlines()

    # Process the data
    x_values = []
    y_values = []

    for line in lines:
        # Split each line into two values
        values = line.strip().split(',')

        # Convert values to float
        x_values.append(float(values[0]))
        y_values.append(float(values[1]))

    # Plot the data
    plt.figure(num)
    plt.plot(x_values, y_values, marker='o', linestyle='-', color='b')
    plt.title('Data Plot')
    plt.xlabel('X-axis')
    plt.ylabel('Y-axis')
    plt.grid(True)
    plt.show()
    
utm_x_zero = 346638.99532500084
utm_y_zero = 4070388.235393337
utm_coords_path = []
utm_coords_continuous_path = []
# Open the file in 'read' mode
with open('path.txt', 'r') as file:
    # Read all lines from the file
    lines = file.readlines()

    # Process each line
    for line in lines:
        # Split the line into UTM coordinates
        utm_coordinates = line.strip().split(',')
        
        # Convert the coordinates to float
        utm_coord = [float(utm_coordinates[1]),float(utm_coordinates[0])]
        utm_coords_path.append(utm_coord)

with open('path_continuous.txt', 'r') as file:
    # Read all lines from the file
    lines = file.readlines()

    # Process each line
    for line in lines:
        # Split the line into UTM coordinates
        utm_coordinates = line.strip().split(',')
        
        # Convert the coordinates to float
        utm_coord = [float(utm_coordinates[0]),float(utm_coordinates[1])]
        utm_coords_continuous_path.append(utm_coord)
# print(utm_coords)

# straight_line(utm_coords_path)
# show_line('straight_line.txt',1)

# circle_line(utm_coords_path)
# show_line('circle_line.txt',2)

# U_turn(utm_coords_path)
# show_line('U_turn.txt',3)

print(utm_coords_continuous_path)
gym_path(utm_coords_continuous_path)
show_line('gym_path.txt', 4)