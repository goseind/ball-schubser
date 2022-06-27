from geometry_msgs.msg import Quaternion

# CONSTANTS FOR DISTANCE CALCULATION
SENSOR_HEIGHT = 2.70
FOCAL_LENGTH = 1.67  # 24
HEIGHT_CAMERA = 5.8
WIDTH_CAMERA = 13.3

BALL_HEIGHT = 67
GOAL_OBJECT_HEIGHT = 20
GOAL_OBJECT_WIDTH = 5


def get_distance_of_ball(width_ball, height_ball, x_pos_ball, width_image, height_image):
    pos = Quaternion()

    # X POSITION
    x_center = width_ball/2 + x_pos_ball
    rel = (BALL_HEIGHT/width_ball)
    pos_x_pixel = x_center - (width_image/2)
    pos.x = pos_x_pixel * rel

    # Y POSITION
    height_ball_sensor = SENSOR_HEIGHT * ((height_ball)/height_image)
    relation_height = height_ball_sensor/FOCAL_LENGTH
    pos.y = BALL_HEIGHT/relation_height


def get_distance_of_goal(width_goal, height_goal, x_pos_goal, width_image, height_image):
    pos = Quaternion()

    # X POSITION
    x_center = width_goal/2 + x_pos_goal
    rel = (GOAL_OBJECT_WIDTH/width_goal)
    pos_x_pixel = x_center - (width_image/2)
    pos.x = pos_x_pixel * rel

    # Y POSITION
    height_goal_sensor = SENSOR_HEIGHT * ((height_goal)/height_image)
    relation_height = height_goal_sensor/FOCAL_LENGTH
    pos.y = GOAL_OBJECT_HEIGHT/relation_height
