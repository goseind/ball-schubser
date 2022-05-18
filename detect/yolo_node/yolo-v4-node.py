import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64
from cv_bridge import CvBridge
from models import Yolov4

class YoloNode(object):
    def __init__(self):
        self.image = None
        self.imageBridge = CvBridge()
        self.loop_rate = rospy.Rate(5)
        self.RELEVANT_OBJECT = 'sports ball'
        self.model = Yolov4(weight_path='weights\yolov4.weights', class_name_path='class_names\coco_classes.txt')

        # Publishers
        self.pub = rospy.Publisher('target_offset', Float64, queue_size=5)

        # Subscribers
        rospy.Subscriber('/camera/image', Image, self.callback)
        
        rospy.loginfo("Initialized target calculation...")

    def callback(self, msg):
        rospy.loginfo('Image received...')
        self.image = self.imageBridge.imgmsg_to_cv2(msg)

    def start(self):
        rospy.loginfo("Starting target calculation...")
        self.loop()

    def loop(self):
        while not rospy.is_shutdown():
            if self.image is not None:
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

                        self.pub.publish(target_offset)
            
            self.loop_rate.sleep()

if __name__ == '__main__':
    rospy.init_node('target_calculation')
    node = YoloNode()
    node.start()
