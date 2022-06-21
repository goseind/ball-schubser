import pandas as pd
import numpy as np
import cv2
from keras import layers, models
from utils import load_weights, get_detection_data
from layers import yolov4_head, yolov4_neck, nms


class MinimalYolov4(object):
    def __init__(self, weight_path, class_name_path, verbose=False):
        self.yolo_model = None
        self.prediction_model = None
        self.image_size = (416, 416, 3)
        self.class_names = [line.strip() for line in open(class_name_path).readlines()]
        self.classes_count = len(self.class_names)
        self.anchors = np.array([12, 16, 19, 36, 40, 28, 36, 75, 76, 55, 72, 146, 142, 110, 192, 243, 459, 401]).reshape((3, 3, 2))
        self.xyscale = [1.2, 1.1, 1.05]
        self.iou_threshold = 0.413
        self.score_threshold = 0.3
        self.weight_path = weight_path

        self.verbose = verbose

        self.build_model()

    def build_model(self):
        input_layer = layers.Input(self.image_size)
        yolo_output = yolov4_neck(input_layer, self.classes_count)
        self.yolo_model = models.Model(input_layer, yolo_output)

        yolo_output = yolov4_head(yolo_output, self.classes_count, self.anchors, self.xyscale)
        # output: [boxes, scores, classes, valid_detections]
        self.prediction_model = models.Model(input_layer,
                                             nms(yolo_output, self.image_size, self.classes_count,
                                                 iou_threshold=self.iou_threshold,
                                                 score_threshold=self.score_threshold))

        load_weights(self.yolo_model, self.weight_path)

    def predict(self, raw_image):
        images = np.expand_dims(cv2.resize(raw_image, self.image_size[:2]) / 255, axis=0)
        pred_output = self.prediction_model.predict(images)
        return get_detection_data(img=raw_image, model_outputs=pred_output, class_names=self.class_names)

    def get_detections(self, image_shape, prediction):
        num_bboxes = prediction[-1][0]
        boxes, scores, classes = [output[0][:num_bboxes] for output in prediction[:-1]]

        h, w = image_shape[:2]
        df = pd.DataFrame(boxes, columns=['x1', 'y1', 'x2', 'y2'])
        df[['x1', 'x2']] = (df[['x1', 'x2']] * w).astype('int64')
        df[['y1', 'y2']] = (df[['y1', 'y2']] * h).astype('int64')
        df['class_name'] = np.array(self.class_names)[classes.astype('int64')]
        df['score'] = scores
        df['w'] = df['x2'] - df['x1']
        df['h'] = df['y2'] - df['y1']

        if self.verbose:
            print(f'# of bboxes: {num_bboxes}')

        return df
