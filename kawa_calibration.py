def translate_coordinates_45cm(coordinates, centre_point):
    # pixels
    px1 = [380, 462]  # x,y
    px2 = [686, 510]  # x,y
    px3 = [1082, 295]  # x,y
    # robot coordinates
    cd1 = [-190.7, 407.6]  # x,y
    cd2 = [-25.5, 387.9]  # x,y
    cd3 = [176.4, 506.3]  # x,y
    cdX = (((coordinates[0] - px1[0]) * (cd3[0] - cd1[0])) / (px3[0] - px1[0])) + cd1[0]
    cdY = (((coordinates[1] - px1[1]) * (cd2[1] - cd1[1])) / (-px1[1] + px2[1])) + cd1[1]
    cdZ = translate_height(coordinates[2])

    centre_x = (((centre_point[0] - px1[0]) * (cd3[0] - cd1[0])) / (px3[0] - px1[0])) + cd1[0]
    centre_y = (((centre_point[1] - px1[1]) * (cd2[1] - cd1[1])) / (-px1[1] + px2[1])) + cd1[1]

    return [cdX, cdY, cdZ, coordinates[3], centre_x, centre_y]


def translate_coordinates_35cm(coordinates, centre_point):
    # pixels
    px1 = [802, 250]  # x,y
    px2 = [506, 437]  # x,y
    px3 = [917, 482]  # x,y
    # robot coordinates
    cd1 = [64.0, 548.0]  # x,y
    cd2 = [-125.5, 429.3]  # x,y
    cd3 = [133.9, 399.7]  # x,y
    cdX = (((coordinates[0] - px1[0]) * (cd3[0] - cd1[0])) / (px3[0] - px1[0])) + cd1[0]
    cdY = (((coordinates[1] - px1[1]) * (cd2[1] - cd1[1])) / (-px1[1] + px2[1])) + cd1[1]
    cdZ = translate_height(coordinates[2])

    centre_x = (((centre_point[0] - px1[0]) * (cd3[0] - cd1[0])) / (px3[0] - px1[0])) + cd1[0]
    centre_y = (((centre_point[1] - px1[1]) * (cd2[1] - cd1[1])) / (-px1[1] + px2[1])) + cd1[1]

    return [cdX, cdY, cdZ, coordinates[3], centre_x, centre_y]


def translate_height(z_coordinate):
    # Measured distance from depth camera in meters
    height1 = 0.755
    height2 = 0.25
    # Z-coordinates as read from teachbox
    robot_z1 = -69
    robot_z2 = 433.75

    return ((robot_z1 - robot_z2)/(height1 - height2)) * (z_coordinate - height1) + robot_z1
