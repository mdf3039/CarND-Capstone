from styx_msgs.msg import TrafficLight
import numpy as np
import tensorflow as tf
import rospy
import matplotlib.pyplot as plt
import cv2
import time

class TLClassifier(object):
    def __init__(self):
        
        print "\nTraffic light detection is loading, please wait...\n"
        # Load traffic light detector (Tensorflow's pretrained Mobilenet)
        PATH_TO_MODEL =  'model_detect/frozen_inference_graph.pb'  
        detection_graph = tf.Graph()

        with detection_graph.as_default():
            od_graph_def = tf.GraphDef()

            with tf.gfile.GFile(PATH_TO_MODEL, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
                
        self.sess_detect = tf.Session(graph=detection_graph)
        # Placeholders and graphs
        self.image_tensor_detect = detection_graph.get_tensor_by_name('image_tensor:0')
        self.boxes = detection_graph.get_tensor_by_name('detection_boxes:0')
        self.scores = detection_graph.get_tensor_by_name('detection_scores:0')
        self.classes = detection_graph.get_tensor_by_name('detection_classes:0')

        # Load traffic light classifier
        PATH_TO_CLASSIFIER =  'model_clf/frozen_inference_graph.pb'
        
        clf_graph = tf.Graph()
        with clf_graph.as_default():
            od_graph_def = tf.GraphDef()

            with tf.gfile.GFile(PATH_TO_CLASSIFIER, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')

        self.sess_clf = tf.Session(graph=clf_graph)
        # Placeholders and graphs
        self.image_tensor_clf = clf_graph.get_tensor_by_name('x:0')
        self.keep_prob = clf_graph.get_tensor_by_name('keep_prob:0')
        self.classify_tl = clf_graph.get_tensor_by_name('logits:0')
        self.classify_tl_softmax = clf_graph.get_tensor_by_name('softmax:0')
        
        self.DETECTION_THRESHOLD = 0.1
        self.MAX_NUM_BOXES = 3
        self.IM_HEIGHT, self.IM_WIDTH = 600, 800
        # Draws boxes and shows images if set to true
        self.DRAW_BOXES = False

        print "Traffic light detection is loaded"

    def get_classification(self, image):
        """Determines the color of the traffic light in the image
        Args:
            image (cv::Mat): image containing the traffic light
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        
        #start = time.time()
        # Traffic light detection
        boxes, scores, classes = self.sess_detect.run([self.boxes, self.scores, self.classes],
                                                                  feed_dict={self.image_tensor_detect: image[np.newaxis,:,:,:]})

        boxes = boxes.squeeze()
        classes = classes.squeeze()
        scores = scores.squeeze()
        # Ignore all classes except traffic lights
        classes = np.where(classes==10, 1, 0)
        # Set the probability of non-trafficlights to zero
        scores = (scores * classes).squeeze()
        # Ignore detections with a certainty below the threshold
        scores = np.where(scores<self.DETECTION_THRESHOLD, 0, scores)
        #rospy.logerr("np.max(scores) %s", np.max(scores))

        # Draw box around the traffic light
        green = 0
        red = 0
        yellow = 0

        for n_boxes in range(len(boxes)):
            i = np.argmax(scores)
            if scores[i] != 0 and n_boxes < self.MAX_NUM_BOXES:
                scores[i] = 0.
        
                left, right, top, bottom = (int(boxes[i, 1] * self.IM_WIDTH), 
                                            int(boxes[i, 3] * self.IM_WIDTH), 
                                            int(boxes[i, 0] * self.IM_HEIGHT), 
                                            int(boxes[i, 2] * self.IM_HEIGHT))

                clf_img = image.copy()
                clf_img = clf_img[top : bottom, left : right] / 255.
                # Resize cropped image
                clf_img = cv2.resize(clf_img, (20, 60)) 
                # Traffic light classification
                pred = self.sess_clf.run(self.classify_tl, {self.image_tensor_clf : clf_img[np.newaxis,:,:,:], self.keep_prob : 1.})
                pred = np.argmax(pred) 
                   
                if pred == 0:
                    red += 1
                    if self.DRAW_BOXES:    
                        cv2.rectangle(image, (left, top),(right, bottom), (0,0,255), 4)
                elif pred == 1:
                    green += 1
                    if self.DRAW_BOXES:    
                        cv2.rectangle(image, (left, top),(right, bottom), (0,255,0), 4)
                elif pred == 2:
                    yellow += 1
                    if self.DRAW_BOXES:    
                        cv2.rectangle(image, (left, top),(right, bottom), (0,225,255), 4)
                elif pred == 3:
                    if self.DRAW_BOXES:    
                        cv2.rectangle(image, (left, top),(right, bottom), (255,225,255), 4)
            else:
                break
        
        #elapsed_time = time.time() - start
        #rospy.logerr("time: %s", elapsed_time)
        if self.DRAW_BOXES:
            plt.imshow(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
            plt.show()

        if n_boxes:
            if red > green and red > yellow:
                return TrafficLight.RED

            elif green > red and green > yellow:
                return TrafficLight.GREEN
            
            elif yellow > green and yellow > red:
                return TrafficLight.YELLOW

        return TrafficLight.UNKNOWN
