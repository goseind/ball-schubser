from flask import Response
from flask import Flask
from flask import render_template, send_from_directory
import cv2
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
import os
import numpy as np
from cv_bridge import CvBridge
import threading

app = Flask(__name__)
img = None
bridge = CvBridge()
mission=""

def callback(_img: Image):
  global img
  img = bridge.imgmsg_to_cv2(_img, desired_encoding='passthrough')
  # np_arr = np.fromstring(_img.data, np.uint8)
  # img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

def callback_mission(_mission: String):
    global mission
    mission = _mission.data

from flask import send_from_directory

@app.route('/favicon.svg')
def favicon():
    return send_from_directory(os.path.join(app.root_path, 'static'), 'favicon.svg',mimetype='image/svg+xml')

@app.route("/")
def index():
    return render_template("index.html")

def GetImage():
    global img, mission
    while True:
        if img is not None:
            y_offset=20
            blank_image = np.zeros((img.shape[0]+20, img.shape[1], 3), np.uint8)
            blank_image[y_offset:y_offset+img.shape[0], 0:img.shape[1]] = img
            cv2.putText(blank_image, "Mission: " + mission, (0, 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            ret, buffer = cv2.imencode('.jpg', blank_image)
            frame = buffer.tobytes()
            yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        else:
            break

@app.route("/stream")
def stream():
    return Response(GetImage(), mimetype = "multipart/x-mixed-replace; boundary=frame")

def initWebserver():
    app.run(debug=True, threaded = True, use_reloader = False)

def init():
    global pos_pub, debug_pub
    x = threading.Thread(target=initWebserver)
    x.start()
    rospy.init_node('ball_schubser_cockpit', anonymous=True)
    rospy.Subscriber("debug_image", Image, callback)
    rospy.Subscriber("mission", String, callback_mission)
    rospy.loginfo("Starting cockpit node ...")
    print("done")
    rospy.spin()

if __name__ == "__main__":
    try:
        init()
    except rospy.ROSInterruptException:
        pass
