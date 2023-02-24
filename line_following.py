from heading_command import followHeading, gpsConversion


def followLine(data_file, position_file, gps, imu, arduino, encoder, A, b,
               gps_points, point_cnt):
    line_a, line_b = gps_points[point_cnt], gps_points[point_cnt + 1]
    line_a[0, 0], line_a[1, 0] = gpsConversion(line_a[0, 0],
                                               line_a[1,
                                                      0])  # convert gps to xy
    line_b[0, 0], line_b[1, 0] = gpsConversion(line_b[0, 0],
                                               line_b[1,
                                                      0])  # convert gps to xy
    print("############# Following line ", point_cnt + 1)
    followHeading(data_file, position_file, imu, arduino, encoder, gps, A, b,
                  line_a, line_b)

    point_cnt += 1
    return point_cnt
