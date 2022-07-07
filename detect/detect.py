import torch
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Vector3
import cv2

# or yolov5n - yolov5x6, custom
model = torch.hub.load('ultralytics/yolov5', 'yolov5s')
bridge = CvBridge()
ball_class = 32
bottle_class = 39
cup_class = 41
wine_glass = 40

image_detection_running = False

# CONSTANTS FOR DISTANCE CALCULATION
SENSOR_HEIGHT = 2.70
FOCAL_LENGTH = 1.67  # 24
BALL_HEIGHT = 67
HEIGHT_CAMERA = 5.8
WIDTH_CAMERA = 13.3

def callback(img: Image):
    global image_detection_running, pos, debug_pub, pos_pub

    if image_detection_running:
        return

    image_detection_running = True

    # rospy.loginfo("Uhh I got a new image")

    cv_image1 = bridge.imgmsg_to_cv2(img, desired_encoding='passthrough')
    cv_image = cv2.flip(cv_image1, -1)
    debug_image = cv_image.copy()

    # Inference
    results = model(cv_image)

    # results.print()
    p = results.pandas().xyxy[0]
    detections = p.to_dict(orient="records")

    found = False
    for d in detections:
        con = d['confidence']
        cs = d['class']
        # if (cs == ball_class or cs == bottle_class or cs == cup_class) and con > 0.1:
        if (cs == ball_class or cs == bottle_class or cs == cup_class or cs == wine_glass) and con > 0.1:
            found = True
            # print("detected: " + toClass(cs))
            x1 = int(d['xmin'])
            x2 = int(d['xmax'])
            y1 = int(d['ymin'])
            y2 = int(d['ymax'])
            x_center = int((x1 + x2) / 2)
            x_center = float(x_center / cv_image.shape[1])
            y_center = int((y1 + y2) / 2)
            y_center = float(y_center / cv_image.shape[0])
            cv2.rectangle(debug_image, (x1, y1), (x2, y2), classToColor(cs), 2)
            label = "{} {:.2f}%".format(toClass(cs), con)
            label_y = y1 if y1 > 24 else y2 + 24
            cv2.putText(debug_image, label, (x1, label_y), cv2.FONT_HERSHEY_SIMPLEX, 1, classToColor(cs), 2)
            pos = Vector3()
            pos.x = x_center
            pos.y = y2 / cv_image.shape[0]
            pos.z = cs
            pos_pub.publish(pos)
    image_detection_running = False
    debug_pub.publish(bridge.cv2_to_imgmsg(debug_image, encoding="passthrough"))

    if not found:
        pos = Vector3()
        pos.x = -1
        pos.y = -1
        pos.z = -1
        pos_pub.publish(pos)

    # print("Finished {}".format(time.localtime().tm_sec))

def toClass(x):
    if x == ball_class:
        return "ball"
    elif x == bottle_class:
        return "bottle"
    elif x == cup_class:
        return "cup"
    elif x == wine_glass:
        return "wine glass"
    else:
        return "unknown"

def classToColor(x):
    if x == ball_class:
        return (0, 255, 0)
    elif x == bottle_class:
        return (0, 0, 255)
    elif x == cup_class:
        return (255, 0, 0)
    elif x == wine_glass:
        return (0, 255, 255)
    else:
        return (0, 0, 0)

def loop():
    global pos_pub
    if rospy.is_shutdown():
        return

def init():
    global pos_pub, debug_pub

    pos_pub = rospy.Publisher('ball_pos', Vector3, queue_size=10)
    debug_pub = rospy.Publisher('debug_image', Image, queue_size=1)
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
