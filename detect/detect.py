import torch
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge

model = torch.hub.load('ultralytics/yolov5', 'yolov5s')  # or yolov5n - yolov5x6, custom
bridge = CvBridge()
ball_class=32

def callback(img: Image):
    global ball_pos_pub
    # rospy.loginfo("Uhh I got a new image")

    cv_image = bridge.imgmsg_to_cv2(img, desired_encoding='passthrough')

    # Inference
    results = model(cv_image)

    # results.print()
    p = results.pandas().xyxy[0]
    detections = p.to_dict(orient="records")
    ball_pos = float(-1.0)

    for d in detections:
        con = d['confidence']
        cs = d['class']
        x1 = int(d['xmin'])
        x2 = int(d['xmax'])
        # y1 = int(d['ymin'])
        # y2 = int(d['ymax'])
        x_center = int((x1 + x2) / 2)
        if cs == ball_class:
            ball_pos = float(x_center / cv_image.shape[1])
            rospy.loginfo("Ball detected at: " + str(ball_pos) + " with confidence: " + str(con))
            ball_pos_pub.publish(ball_pos)
            break

# def loop():
#     global ball_pos_pub
#     if ball_pos > 0:
#         rospy.loginfo("Ball detected at: ", float(ball_pos))
#     ball_pos_pub.publish(str(ball_pos))

def init():
    global ball_pos_pub

    ball_pos_pub = rospy.Publisher('ball_pos', Float64, queue_size=10)
    rospy.init_node('ball_schubser_detect', anonymous=True)
    rospy.Subscriber("usb_cam/image_raw", Image, callback)
    rospy.loginfo("Starting detection node ...")
    print("done")
    rospy.spin()

    # rate = rospy.Rate(10) # 10hz
    # while not rospy.is_shutdown():
    #     hello_str = "hello world %s" % rospy.get_time()
    #     rospy.loginfo(hello_str)
    #     ball_pos_pub.publish(hello_str)
    #     loop()
    #     rate.sleep()

if __name__ == '__main__':
    try:
        init()
    except rospy.ROSInterruptException:
        pass
