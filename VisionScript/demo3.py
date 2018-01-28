from darkflow.net.build import TFNet
import cv2

options = {"model": "cfg/tiny-yolo-voc.cfg", "load": "bin/tiny-yolo-voc.weights", "threshold": 0.1, "gpu": 0.85}

tfnet = TFNet(options)

while True:
    imgcv = cv2.imread("./sample_img/sample_dog.jpg")
    result = tfnet.return_predict(imgcv)
    print(result)