import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge
import cv2
from models import Yolov4

class YoloNode(object):
    def __init__(self):
        self.image = None
        self.imageBridge = CvBridge()
        self.loop_rate = rospy.Rate(10)
        self.RELEVANT_OBJECT = 'sports ball'
        self.model = Yolov4(weight_path='weights/yolov4.weights', class_name_path='class_names/coco_classes.txt')

        # Publishers
        self.pub = rospy.Publisher('ball_pos', Float64, queue_size=5)

        # Subscribers
        rospy.Subscriber('cv_camera/image_raw', Image, self.callback)

        rospy.loginfo("Initialized target calculation ...")

    def callback(self, msg):
        # rospy.loginfo('Image received...')
        self.image = self.imageBridge.imgmsg_to_cv2(msg)

    def start(self):
        rospy.loginfo("Starting target calculation...")
        self.loop()

    def loop(self):
        while not rospy.is_shutdown():
            if self.image is not None:

                self.image = cv2.flip(self.image, -1)
                prediction = self.model.predict_img(self.image, plot_img=False, return_output=True)
                datframe = prediction[1]

                if not datframe.empty:
                    filtered = datframe.loc[datframe['class_name'] == self.RELEVANT_OBJECT]

                    if not filtered.empty:
                        sorted_lowest = filtered.sort_values('y1', ascending=True)

                        nearest_target = sorted_lowest.iloc[0]
                        image_x_width = self.image.shape[1]
                        x1 = nearest_target['x1']
                        w = nearest_target['w']
                        x_center = x1 + 0.5 * w

                        target_offset = 2 * x_center / image_x_width - 1

                        # print("Target offset: {}".format(target_offset))
                        newtarget_offset = (target_offset +1) /2
                        self.pub.publish(newtarget_offset)
                    else:
                        self.pub.publish(-1.0)
                else:
                    self.pub.publish(-1.0)

            self.loop_rate.sleep()

if __name__ == '__main__':
    rospy.init_node('target_calculation')
    node = YoloNode()
    node.start()
