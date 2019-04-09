from styx_msgs.msg import TrafficLight

import tensorflow as tf
import numpy as np
import cv2

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier (done)
        #SSD_GRAPH_FILE = 'ssd_mobilenet_v1_coco_2018_01_28/frozen_inference_graph.pb'
        SSD_GRAPH_FILE = 'frozen_inference_graph.pb'
        tf.reset_default_graph()
        self.detection_graph = self.load_graph(SSD_GRAPH_FILE)
        ##==== variables =====        
        # tensors in the model, for run the model and get results
        self.image_tensor = self.detection_graph.get_tensor_by_name('image_tensor:0')
        print(self.image_tensor)
        self.detection_boxes = self.detection_graph.get_tensor_by_name('detection_boxes:0')
        self.detection_scores = self.detection_graph.get_tensor_by_name('detection_scores:0')
        self.detection_classes = self.detection_graph.get_tensor_by_name('detection_classes:0')


        self.tfSession = tf.Session(graph=self.detection_graph)


        pass

    def get_classification(self, image, is_testlot=False):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        #TODO implement light color prediction (done)
        if is_testlot:
            return self.get_classification_testlot(image)
        else:
            return self.get_classification_simulator(image)

    def get_classification_simulator(self, image):
        """Determines the color of the traffic light in the simulator image
        """
        #TODO implement light color prediction (done)
        status = TrafficLight.UNKNOWN  #default status is UNKNOWN (4)
        image = cv2.resize(image, (200, 150))  #downsizing from 600*800 to 150*200
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)  # RGB image is used in mobilenet-ssd model
        image_np = np.expand_dims(image, 0)  #create tensor

        (boxes, scores, classes) = self.tfSession.run([self.detection_boxes, self.detection_scores, self.detection_classes], 
                                            feed_dict={self.image_tensor: image_np})
        # Remove unnecessary dimensions
        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes)        
        
        #==== Further process the detection results        
        idxs = (scores>0.2)&(classes==10)  # traffic light with confidence >0.2    
        filtered_boxes = boxes[idxs, ...]
        filtered_scores = scores[idxs, ...]
        filtered_classes = classes[idxs, ...]

        # pick the max 3 lights, the scores were sorted by default
        filtered_boxes = filtered_boxes[:3, ...]
        filtered_scores = filtered_scores[:3, ...]
        filtered_classes = filtered_classes[:3, ...]

        # The current box coordinates are normalized to a range between 0 and 1.
        # This converts the coordinates actual location on the image.
        height, width, _ = image.shape
        box_coords = self.to_image_coords(filtered_boxes, height, width)


        box_coords = box_coords.astype(int)
        statusList = [] #store status from each bounding box
        for i in range(len(box_coords)):
            top, left, bot, right = box_coords[i, ...] 
            roi = image[top:bot, left:right, :]
            roi_hsv = cv2.cvtColor(roi, cv2.COLOR_RGB2HSV)

            #red H value in opencv is around 0
            sensitivity = 15
            roi_red = cv2.inRange(roi_hsv, (0, 100, 100), (sensitivity, 255, 255))|cv2.inRange(roi_hsv, (180-sensitivity, 100, 100), (180, 255, 255))
            nRedPixels = np.count_nonzero(roi_red) #np.sum(roi_red)

            #yellow H value in opencv is around 30
            sensitivity = 15
            roi_yellow = cv2.inRange(roi_hsv, (30-sensitivity, 100, 100), (30+sensitivity, 255, 255))
            nYellowPixels = np.count_nonzero(roi_yellow) #np.sum(roi_yellow)

            #green H value in opencv is around 60
            sensitivity = 15
            roi_green = cv2.inRange(roi_hsv, (60-sensitivity, 100, 100), (60+sensitivity, 255, 255))
            nGreenPixels = np.count_nonzero(roi_green) #np.sum(roi_green)

            RYG_array = np.array([nRedPixels, nYellowPixels, nGreenPixels])
            ix = np.argmax(RYG_array)
            if RYG_array[ix]> (0.01*float((bot-top)*(right-left))): # a little sanity check, requires the max color at least occupies 1% of the area
                statusList.append(ix)  #the index happens to be the state of light :)

        if statusList:
            status = round(sum(statusList)/len(statusList)) # use average status value here, could use majority vote, or weighted majority vote with scores

        return status

    def get_classification_testlot(self, image):
        """Determines the color of the traffic light in the test lot image
        """
        #TODO implement light color prediction (done)
        status = TrafficLight.UNKNOWN  #default status is UNKNOWN (4)
        #image = cv2.resize(image, (200, 150))  #downsizing from 600*800 to 150*200
        image = image[200:800, 200:1000, :]
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)  # RGB image is used in mobilenet-ssd model
        image_np = np.expand_dims(image, 0)  #create tensor

        (boxes, scores, classes) = self.tfSession.run([self.detection_boxes, self.detection_scores, self.detection_classes], 
                                            feed_dict={self.image_tensor: image_np})
        # Remove unnecessary dimensions
        boxes = np.squeeze(boxes)
        scores = np.squeeze(scores)
        classes = np.squeeze(classes)        
        
        #==== Further process the detection results        
        idxs = (scores>0.2)&(classes==10)  # traffic light with confidence >0.2    
        filtered_boxes = boxes[idxs, ...]
        filtered_scores = scores[idxs, ...]
        filtered_classes = classes[idxs, ...]

        # pick the max 3 lights, the scores were sorted by default
        filtered_boxes = filtered_boxes[:3, ...]
        filtered_scores = filtered_scores[:3, ...]
        filtered_classes = filtered_classes[:3, ...]

        # The current box coordinates are normalized to a range between 0 and 1.
        # This converts the coordinates actual location on the image.
        height, width, _ = image.shape
        box_coords = self.to_image_coords(filtered_boxes, height, width)


        box_coords = box_coords.astype(int)
        statusList = [] #store status from each bounding box
        for i in range(len(box_coords)):
            top, left, bot, right = box_coords[i, ...] 
            roi = image[top:bot, left:right, :]
            roi_hsv = cv2.cvtColor(roi, cv2.COLOR_RGB2HSV)            

            #green H value in opencv is around 85 in parking lot images
            sensitivity = 5
            roi_green = cv2.inRange(roi_hsv, (85-sensitivity, 0, 225), (85+sensitivity, 150, 255))
            nGreenPixels = np.count_nonzero(roi_green) #np.sum(roi_green)
            if nGreenPixels>(0.02*float((bot-top)*(right-left))):  #enough green pixels                
                statusList.append(2)  #green light
            else:                
                # get centroid of pixels with V>225
                ret,thresh = cv2.threshold(roi_hsv[:, :, 2],225,255,0)

                nPixels = np.count_nonzero(thresh)
                if nPixels>(0.02*float((bot-top)*(right-left))):
                    # calculate moments of binary image
                    M = cv2.moments(thresh)
                    # calculate x,y coordinate of center
                    #cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])                    
                    if cY<roi.shape[0]*0.33:
                        statusList.append(0)  # red light
                    else:
                        statusList.append(1)  # green light   

        if statusList:
            status = round(sum(statusList)/len(statusList)) # use average status value here, could use majority vote, or weighted majority vote with scores

        return status  

    def __del__(self):
        self.tfSession.close()
    
##========== Utility functions below for loading tensorflow model, and postprocess SSD box output
    def load_graph(self, graph_file):
        """Loads a frozen inference graph"""
        graph = tf.Graph()
        with graph.as_default():
            od_graph_def = tf.GraphDef()
            with tf.gfile.GFile(graph_file, 'rb') as fid:
                serialized_graph = fid.read()
                od_graph_def.ParseFromString(serialized_graph)
                tf.import_graph_def(od_graph_def, name='')
        return graph	
    def to_image_coords(self, boxes, height, width):
        """
        The original box coordinate output is normalized, i.e [0, 1].

        This converts it back to the original coordinate based on the image
        size.
        """
        box_coords = np.zeros_like(boxes)
        box_coords[:, 0] = boxes[:, 0] * height
        box_coords[:, 1] = boxes[:, 1] * width
        box_coords[:, 2] = boxes[:, 2] * height
        box_coords[:, 3] = boxes[:, 3] * width

        return box_coords
