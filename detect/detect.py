import torch
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge
from geometry_msgs.msg import Quaternion
import cv2

# or yolov5n - yolov5x6, custom
model = torch.hub.load('ultralytics/yolov5', 'yolov5s')
bridge = CvBridge()
ball_class = 32
bottle_class = 1 # TODO
image_detection_running = False

pos = Quaternion()
pos.x = -1.0
pos.y = -1.0
pos.z = -1.0
pos.w = -1.0

# CONSTANTS FOR DISTANCE CALCULATION
SENSOR_HEIGHT = 2.70
FOCAL_LENGTH = 1.67  # 24
BALL_HEIGHT = 67
HEIGHT_CAMERA = 5.8
WIDTH_CAMERA = 13.3

def callback(img: Image):
    global image_detection_running, pos, debug_pub

    if image_detection_running:
        return

    image_detection_running = True

    # rospy.loginfo("Uhh I got a new image")

    cv_image1 = bridge.imgmsg_to_cv2(img, desired_encoding='passthrough')
    cv_image = cv2.flip(cv_image1, -1)
    # cv2.imwrite('/app/cap.jpg', cv_image)
    image_message = bridge.cv2_to_imgmsg(cv_image, encoding="passthrough")
    debug_pub.publish(image_message)

    # Inference
    results = model(cv_image)

    # results.print()
    p = results.pandas().xyxy[0]
    detections = p.to_dict(orient="records")
    pos.x = -1.0
    pos.y = -1.0
    pos.z = -1.0
    pos.w = -1.0

    for d in detections:
        con = d['confidence']
        cs = d['class']
        if (cs == ball_class or cs == bottle_class) and con > 0.25:
            x1 = int(d['xmin'])
            x2 = int(d['xmax'])
            y1 = int(d['ymin'])
            y2 = int(d['ymax'])
            x_center = int((x1 + x2) / 2)
            y_center = int((y1 + y2) / 2)
            if cs == ball_class:
                pos.x = x_center
                pos.y = y_center
            if cs == bottle_class:
                pos.z = x_center
                pos.w = y_center
    image_detection_running = False

def loop():
    global pos_pub
    if rospy.is_shutdown():
        return
    pos_pub.publish(pos)

def init():
    global pos_pub, debug_pub

    pos_pub = rospy.Publisher('ball_pos', Quaternion, queue_size=10)
    debug_pub = rospy.Publisher('ball_image', Image, queue_size=10)
    rospy.init_node('ball_schubser_detect', anonymous=True)
    rospy.Subscriber("cv_camera/image_raw", Image, callback)
    rospy.loginfo("Starting detection node ...")
    print("done")

    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        loop()
        rate.sleep()

if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass

# if cs == ball_class:

#     # X POSITION
#     rel = (BALL_HEIGHT/(x2-x1))
#     pos_X_Pixel = (img.shape[1]/2) - (img.shape[1] - x_center)
#     pos_X = pos_X_Pixel * rel

#     # Y POSITION
#     y1 = int(d['ymin'])
#     y2 = int(d['ymax'])
#     heightBallSensor = SENSOR_HEIGHT * ((y2-y1)/heightImg)
#     relation_Height = heightBallSensor/FOCAL_LENGTH
#     pos_Y = BALL_HEIGHT/relation_Height

#     ball_pos = [pos_X, pos_Y]
#     #float(x_center / cv_image.shape[1])
#     # rospy.loginfo("I found a ball at %f with confidence %f", ball_pos, con)
#     break
