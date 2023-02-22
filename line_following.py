import numpy as np
import pyproj
from heading_command import followHeading


def gps_to_xy(lat, lon):
    # Define the projection system you want to use
    project = pyproj.Proj(proj='utm', zone='30T', ellps='WGS84')

    # Convert the GPS coordinates to x, y coordinates
    x, y = project(lon, lat)

    return x, y


def followLine(file, gps, imu, arduino, encoder, A, b, gps_points, point_cnt):
    line_a, line_b = gps_points[point_cnt], gps_points[point_cnt + 1]
    line_a[0, 0], line_a[1, 0] = gps_to_xy(line_a[0, 0],
                                           line_a[1, 0])  # convert gps to xy
    line_b[0, 0], line_b[1, 0] = gps_to_xy(line_b[0, 0],
                                           line_b[1, 0])  # convert gps to xy

    followHeading(file, imu, arduino, encoder, gps, A, b,
                  line_a, line_b)

    point_cnt += 1
