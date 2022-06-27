from detect.yolo_node.distanceCalculation import get_distance_of_ball
import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Quaternion
from cv_bridge import CvBridge
import cv2
from yolo_minimal import MinimalYolov4


class YoloNode(object):
    def __init__(self):
        self.image = None
        self.imageBridge = CvBridge()
        self.loop_rate = rospy.Rate(10)
        self.TARGET_OBJECT = 'sports ball'
        self.DESTINATION_OBJECT = 'bottle'
        self.model = MinimalYolov4(
            weight_path='weights/yolov4.weights', class_name_path='class_names/coco_classes.txt')

        # Publishers
        self.pub = rospy.Publisher('ball_pos', Quaternion, queue_size=5)

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
                # cv2.imwrite('/app/cap.jpg', self.image)

                prediction = self.model.predict(self.image)
                # msg = [-1.0, -1.0, -1.0, -1.0]
                msg = Quaternion()
                msg.x = -1.0
                msg.y = -1.0
                msg.z = -1.0
                msg.w = -1.0

                if not prediction.empty:
                    targets = prediction.loc[prediction['class_name']
                                             == self.TARGET_OBJECT]
                    destinations = prediction.loc[prediction['class_name']
                                                  == self.DESTINATION_OBJECT]

                    if not targets.empty:
                        sorted_lowest = targets.sort_values(
                            'y1', ascending=True)

                        nearest_target = sorted_lowest.iloc[0]
                        x1 = nearest_target['x1']
                        w = nearest_target['w']
                        h = nearest_target['h']
                        x_center = x1 + 0.5 * w

                        y2 = nearest_target['y2']

                        msg.x = x_center / self.image.shape[1]
                        msg.y = y2 / self.image.shape[0]

                        pos = get_distance_of_ball(
                            w, h, x1, self.image.shape[1], self.image.shape[0])
                        msg.x = pos.x
                        msg.y = pos.y

                    if not destinations.empty:

                        # assuming only one destination is set so take the one with highest confidence
                        # print(destinations)
                        destination = destinations.iloc[0]
                        x1 = destination['x1']
                        w = destination['w']
                        h = nearest_target['h']
                        x_center = x1 + 0.5 * w

                        y2 = destination['y2']

                        msg.z = x_center / self.image.shape[1]
                        msg.w = y2 / self.image.shape[0]

                        pos = get_distance_of_ball(
                            w, h, x1, self.image.shape[1], self.image.shape[0])

                        msg.z = pos.x
                        msg.w = pos.y

                self.pub.publish(msg)
            self.loop_rate.sleep()


if __name__ == '__main__':
    rospy.init_node('target_calculation')
    node = YoloNode()
    node.start()
