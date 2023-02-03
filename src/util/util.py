import numpy as np
from netifaces import interfaces, ifaddresses, AF_INET


def get_closest_objects(detections, img, K):
    # function to calculate x and y angle of object from camera perspective
    def calculate_object_angle(x, y):
        x_c, y_c = width / 2, height / 2
        x_angle = np.rad2deg(np.arctan2(x - x_c, K[0][0]))
        y_angle = np.rad2deg(np.arctan2(y_c - y, K[1][1]))
        return x_angle, y_angle

    height, width = img.shape[:2]
    possible_collision_objects = []
    for i in range(len(detections)):
        x = int((detections[i]['box'][0] + (detections[i]['box'][2]))/2)
        y = int((detections[i]['box'][1] + (detections[i]['box'][3]))/2)
        x_pos, y_pos = calculate_object_angle(x, y)
        if -15 <= x_pos <= 15:
            possible_collision_objects.append(detections[i]['label'])
    return possible_collision_objects


def measure_collision_distance(lidar_data):
    # Initialize closest obstacle distance
    collision_distance = []

    # Loop over each LiDAR data point
    for angle, (x, y, z) in lidar_data.items():
        if 260 <= angle <= 280:
            collision_distance.append(z)
    return min(collision_distance) if len(collision_distance) else 0


# Method is used to project lidar points to detected objects but most of the point it misses because of 2d points
def project_lidar_to_camera(img, detections, lidar_data, K, fov=78):
    # Calculate the height and width of the image
    height, width = img.shape[:2]

    # Define the LiDAR boundary based on the field of view
    boundary = np.tan(np.deg2rad(fov / 2))

    # Initialize a list to store the projected LiDAR points
    projected_points = []

    # Initialize closest obstacle distance
    collision_distance = []

    # Loop over each LiDAR data point
    for angle, (x, y, z) in lidar_data.items():
        # Check if the point is within the LiDAR boundary
        if abs(x) > boundary:
            continue
        # Check if z is not equal to zero
        if z == 0:
            continue
        # Calculate the x and y coordinates of the point in the image
        x_img = int(width / 2 + x * K[0][0] / z)
        y_img = int(height / 2 - y * K[1][1] / z)
        # Check if the point is within the image bounds
        if 0 <= x_img < width and 0 <= y_img < height:
            projected_points.append((x_img, y_img))
            temp_distance = []
            for i in range(len(detections)):
                if detections[i]['box'][0] < x_img < detections[i]['box'][2] \
                        and detections[i]['box'][1] < y_img < detections[i]['box'][3]:
                    temp_distance.append(z)
                if len(temp_distance) > 0:
                    detections[i]['distance'] = min(temp_distance)
                    detections[i]['angle'] = angle
        if 260 <= angle <= 280:
            collision_distance.append(z)

    print(f'Projected points: {projected_points}')
    print(f'Obstacle ahead at distance:{min(collision_distance) if len(collision_distance) else 0}')
    return detections, min(collision_distance) if len(collision_distance) else 0


def create_app_url():
    ip = None
    for ifaceName in interfaces():
        if "wl" in ifaceName:
            ip = [i['addr'] for i in ifaddresses(ifaceName).setdefault(AF_INET, [{'addr': 'No IP addr'}])][0]
            break
    return f"http://{ip}:8000"


