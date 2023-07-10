import numpy as np
import cv2
import argparse
import time

# Load Yolo
def Load_yolo():
net = cv2.dnn.readNet("weight/yolov3-tiny.weights", "cfg/yolov3-tiny.cfg")
net = cv2.dnn.readNet("weight/yolov3.weights", "cfg/yolov3.cfg")
classes = []
with open("coco.names", "r") as f:
    classes = [line.strip() for line in f.readlines()]
layer_names = net.getLayerNames()
output_layers = [layer_names[i[0] - 1] for i in net.getUnconnectedOutLayers()]
colors = np.random.uniform(0, 255, size=(len(classes), 3)) #Generate Random Color
return net, classes, colors, output_layers

def detect_objects(img, net, outputlayers):
    blob = cv2.dnn.blobFromImage(img, scalefactor=0.00392, size=(320,320), mean=(0, 0, 0), swapRB=True, crop=False)
    net.setInput(blob)
    outputs = net.forward(outputlayers)
    return blob, outputs

# image loading
img = cv2.imread(img_path)
img = cv2.resize(img, None, fx=0.4, fy=0.4)
height, width, channels = img.shape
return img, height, width, channels
# Load Video
cap = cv2.VideoCapture("vid2.mp4") #0 -> Webcam; "/path"
font = cv2.FONT_HERSHEY_SIMPLEX

timeframe = time.time()
frame_id = 0

while True:
    _, frame = cap.read()
    frame_id += 1
    height, width, channels = frame.shape

    # Detecting objects
    blob = cv2.dnn.blobFromImage(frame, 0.00392, (320, 320), (0, 0, 0), True, crop=False)

    net.setInput(blob)
    outs = net.forward(output_layers)

    # Showing informations on the screen
   def get_box_dimension(outputs, height, width):
    class_ids = []
    confidences = []
    boxes = []
    for out in outs:
        for detection in out:
            scores = detection[5:]
            class_id = np.argmax(scores)
            confidence = scores[class_id]
            if confidence > 0.3: #Confidence Level -> Accuracy
                # Object detected
                center_x = int(detection[0] * width)
                center_y = int(detection[1] * height)
                w = int(detection[2] * width)
                h = int(detection[3] * height)

                # Rectangle coordinates
                x = int(center_x - w / 2)
                y = int(center_y - h / 2)

                boxes.append([x, y, w, h])
                confidences.append(float(confidence))
                class_ids.append(class_id)

    indexes = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

    for i in range(len(boxes)):
        if i in indexes:
            x, y, w, h = boxes[i]
            label = str(classes[class_ids[i]])
            confidence = confidences[i]
            # color = colors[i]
            color = (255,255,255)
            cv2.rectangle(frame, (x, y), (x + w, y + h), color, 2)
            cv2.putText(frame, label, (x, y+30), font, 1, color, 2)
            cv2.putText(frame, label + " " + str(round(confidence, 2)), (x, y+30), font, 1, color, 2)

    elapsed_time = time.time() - timeframe
    fps = frame_id / elapsed_time
    cv2.putText(frame, str(round(fps,2)), (10, 50), font, 2, (255, 255, 255), 2) #FPS Value
    cv2.putText(frame, "FPS", (220, 50), font, 2, (255, 255, 255), 2) #FPS Label
    cv2.imshow("Image", frame)
    key = cv2.waitKey(1)
    if key == 27: #Escape
        break

cap.release()
cv2.destroyAllWindows()