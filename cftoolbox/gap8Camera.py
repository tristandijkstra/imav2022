import argparse
import multiprocessing
from http import client
import threading
import time
import socket,os,struct, time
from turtle import color
import numpy as np
import cv2
import cftoolbox.colourTools as ct

def getArgs():
    parser = argparse.ArgumentParser(description='Connect to AI-deck JPEG streamer example')
    parser.add_argument("-n",  default="192.168.4.1", metavar="ip", help="AI-deck IP")
    parser.add_argument("-p", type=int, default='5000', metavar="port", help="AI-deck port")
    parser.add_argument('--save', action='store_true', help="Save streamed images")
    args = parser.parse_args()
    return args

def connectSocket(args):
    print("Connecting to socket on {}:{}...".format(args.n, args.p))
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((args.n, args.p))
    print("Socket connected")
    return client_socket

def rx_bytes(client, size):
  data = bytearray()
  while len(data) < size:
    data.extend(client.recv(size-len(data)))
  return data

class ImageThread(threading.Thread):
    def __init__(self, client, cb) -> None:
        super().__init__()
        self.client = client
        self.image = None
        self.stop = False
        self.e = None
        self.callbackFunction = cb

        print("ImageGetter Started")
    
    def run(self):
        while not self.stop:
            try:
                packetInfoRaw = rx_bytes(self.client, 4)
                length, _, _ = struct.unpack("<HBB", packetInfoRaw)
                imgHeader = rx_bytes(self.client, length - 2)
                magic, _, _, _, format, size = struct.unpack("<BHHBBI", imgHeader)
                if magic == 0xBC:
                    imgStream = bytearray()
                    while len(imgStream) < size:
                        packetInfoRaw = rx_bytes(self.client, 4)
                        length, _, _ = struct.unpack("<HBB", packetInfoRaw)
                        chunk = rx_bytes(self.client, length - 2)
                        imgStream.extend(chunk)

                    if format == 0:
                        bayer_img = np.frombuffer(imgStream, dtype=np.uint8)
                        bayer_img.shape = (244, 324)
                        color_img = cv2.cvtColor(bayer_img, cv2.COLOR_BayerBG2BGRA)
                    else:
                        pass
                    
                    self.image = color_img
                    self.callbackFunction(color_img)
            except Exception as e:
                f"error: {e}"
                self.e = e

class Camera:
    def __init__(self, client, queue : multiprocessing.Queue = None):
        self.object = None
        self.image = None
        self.e = None
        self.queue = queue


        self.colorUpper = np.array([29, 2193, 255]) #RED color filter
        self.colorLower = np.array([0, 137, 0]) #RED color filter

        # self.colorUpper = np.array([180, 255, 255]) #RED color filter
        # self.colorLower = np.array([0, 0, 0]) #RED color filter
        self.grayLower = 50
        self.grayUpper = 250
        self.minSize = 100
        self.minSlenderness = 3/10
        self.minPillarSize = 300 #Something here, idk
        self.pillarSlenderness = 1/10 #Something here idk
        self.minHoopSize = 300 #Something here, idk
        self.minHoopSlenderness = 3/10 #Something here idk
        self.center = None
        self.camera = ImageThread(client, self.processImage)
        self.camera.start()
        
        # def detectObject(size, slenderness):
        #     if size = None or slenderness = None:
        #         self.object = "Nothing"
        #     else if size > hoopSize


    def processImage(self, img):
        try:
            color_filtered = ct.colorFilter(img, self.colorLower, self.colorUpper)
            # print(color_filtered)
            thresh_img_gray = ct.grayscale(color_filtered, self.grayLower, self.grayUpper)    
            contours, _  = cv2.findContours(image=thresh_img_gray, mode= cv2.RETR_TREE, method= cv2.CHAIN_APPROX_NONE)
            # orb

            orb = cv2.ORB_create()
            kp = orb.detect(img, None)
            kp, des = orb.compute(img, kp)
            img = cv2.drawKeypoints(img, kp, None, color=(0, 255, 0), flags=0)



            cv2.imshow('Color', img)
            cv2.waitKey(1)


            # if (kp is not None) and len(kp) > 600:
            #     print("TOO MANY ORBS")
            #     if self.queue is not None:
            #         self.queue.put({
            #             "center" : (None,None),
            #             # "center" : ((objx-(objx/2))/objx,(objy-(objy/2))/objy),
            #             "camStatus": "avoid",
            #             "img" : img,
            #             "targetOnScreen": False,
            #             "distance": None
            #         }, block=False)
            #         return

            # # QUANTIZED
            # N = 6
            # print("hello")
            # qimg = np.round(img*(N/255), 0)
            # qimgCounts = np.bincount(qimg)
            # totalPixels = 324*324
            # print(qimgCounts)
            # if np.amax(qimgCounts) / totalPixels > 0.8:
            #     print("SCREEN FILLED")
            #     if self.queue is not None:
            #         self.queue.put({
            #             "center" : (None,None),
            #             # "center" : ((objx-(objx/2))/objx,(objy-(objy/2))/objy),
            #             "camStatus": "avoid",
            #             "img" : img,
            #             "targetOnScreen": False,
            #             "distance": None
            #         }, block=False)
            #         return


            if len(contours) > 0:
                try:

                    (objx, objy), w, camStatus = ct.drawBoxV2(img, contours, minSlenderness=0.6) #Coordinates of the center
                    targetOnScreen = True
                    distance = ct.calculatedistance(w, 260, 0.3, 0.6) #If hoop is found, also send the distance of the drone to it
                    #The numbers for distance calculation need to be calibrated for the real life hoop
                except:
                    targetOnScreen = False
                    objx, objy, w, distance, camStatus = None, None, None, None, None
                    distance=None

            
                # # # detectObject(size, slenderness) #Save last seen object


            else:
                targetOnScreen = False
                objx, objy, w, distance, camStatus = None, None, None, None, None

            # print(f"{targetOnScreen}, dist={w}, {distance}")
            self.image = img
            # self.center = [objx, objy]
            self.center = [1, 1]

            if self.queue is not None:
                self.queue.put({
                    "center" : (objx, objy),
                    # "center" : ((objx-(objx/2))/objx,(objy-(objy/2))/objy),
                    "camStatus": camStatus,
                    "img" : img,
                    "targetOnScreen": targetOnScreen,
                    "distance": distance
                }, block=False)
        except Exception as e:
                print(f"error: {e}")
                self.e = e



        


