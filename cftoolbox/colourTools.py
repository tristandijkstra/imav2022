import cv2
import math
import numpy as np

def colorFilter(img, lower, upper):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower, upper)
    mask = cv2.erode(mask, None, iterations=2)
    mask = cv2.dilate(mask, None, iterations=2)

    result = cv2.bitwise_and(img, img, mask=mask)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (30, 30))
    result = cv2.morphologyEx(result, cv2.MORPH_CLOSE, kernel)
    
    return result

# Gets an image, converts it to grayscale and applies a threshold
def grayscale(img, lower, upper):
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # cv2.imshow('Output', img_gray) #Same image as result but in grayscale
    ret, thresh = cv2.threshold(img_gray, lower, upper, cv2.THRESH_BINARY)
    return thresh

def drawBox(img, contours, minSize=100, minSlenderness=3/10):
    hMAX, wMAX = 324, 324
    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea) #Get the largest area contour
        c = cv2.convexHull(c, False) #Draw a convex hull
        rect = cv2.minAreaRect(c) #Gives you a square rotated with the object in question
        # _, _, angle = rect
        x,y,w,h = cv2.boundingRect(c) #Dimensions of the square detected
        if (w*h < minSize) or (w/h < minSlenderness): #Detect whenever you are 'close' enough to the hoop
            return None, None
        centreDOT = [int(x + w/2), int(y + h/2)] #Compute the center
        centre = [float((x - wMAX/2)/wMAX), float((y - hMAX/2))/hMAX] #Compute the center
        # print(centre)
        box = cv2.boxPoints(rect) #Get the corner points of the rectangle
        box = np.int0(box) #Convert to numpy array
        cv2.drawContours(img,[box],0,(0,0,255),2) #Draw the box

        cv2.drawMarker(img, centreDOT, (255,255,0), cv2.MARKER_CROSS, 1, 16) #Draw a marker in the middle of the box
        return centre, w
    else:
        return None, None


def drawBoxV2(img, contours, minSize=100, minSlenderness=3/10):
    hMAX, wMAX = 324, 324

    if len(contours) > 0:
        c = max(contours, key=cv2.contourArea) #Get the largest area contour
        c = cv2.convexHull(c, False) #Draw a convex hull
        rect = cv2.minAreaRect(c) #Gives you a square rotated with the object in question
        x,y,w,h = cv2.boundingRect(c) #Dimensions of the square detected
        


        if (w*h < minSize):
            return None, None, None
        elif (w/h > minSlenderness):
            # NOTE TARGET
            centreDOT = [int(x + w/2), int(y + h/2)] #Compute the center
            centre = [float((x - wMAX/2)/wMAX), float((y - hMAX/2))/hMAX] #Compute the center
            # print(centre)
            box = cv2.boxPoints(rect) #Get the corner points of the rectangle
            box = np.int0(box) #Convert to numpy array
            cv2.drawContours(img,[box],0,(0,0,255),2) #Draw the box

            cv2.drawMarker(img, centreDOT, (255,255,0), cv2.MARKER_CROSS, 1, 16) #Draw a marker in the middle of the box
            return centre, w, "target"
        elif (w/h > minSlenderness) and (w*h > 175):
            # NOTE PILAR

            return None, None, "avoid"
        elif (w/h > 1):
            # NOTE OTHER

            return None, None, "avoid"
        else:
            return None, None, None
    else:
        return None, None, None

#Uses KNOWN DIMENSIONS OF REAL LIFE and KNOWN PIXEL WIDTHS at a KNOWN DISTANCE
def calculatedistance(input_pixel_width, known_pixel_width=115, known_width=0.6, known_distance=1.5):
    # https://pyimagesearch.com/2015/01/19/find-distance-camera-objectmarker-using-python-opencv/
    focal = (known_pixel_width*known_distance)/known_width
    distance = (known_width*focal)/input_pixel_width
    return distance

## CALIBRATION CODE

def nothing(x):
    #any operation
    pass

#Little function to calibrate your color filtering
def calibratecolor(image):
        ###########  TUNING COLOR (1/2)  ###############
        cv2.namedWindow("Trackbars")
        cv2.createTrackbar("L-H", "Trackbars", 0, 180, nothing)
        cv2.createTrackbar("L-S", "Trackbars", 0, 255, nothing)
        cv2.createTrackbar("L-V", "Trackbars", 0, 255, nothing)
        cv2.createTrackbar("U-H", "Trackbars", 180, 180, nothing)
        cv2.createTrackbar("U-S", "Trackbars", 255, 255, nothing)
        cv2.createTrackbar("U-V", "Trackbars", 255, 255, nothing)

        while True:
                img = image # This the software reading each frame and outputting it.
                img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
                
                ###########  TUNING COLOR (2/2)  ###############
                l_h = cv2.getTrackbarPos("L-H", "Trackbars")
                l_s = cv2.getTrackbarPos("L-S", "Trackbars")
                l_v = cv2.getTrackbarPos("L-V", "Trackbars")
                u_h = cv2.getTrackbarPos("U-H", "Trackbars")
                u_s = cv2.getTrackbarPos("U-S", "Trackbars")
                u_v = cv2.getTrackbarPos("U-V", "Trackbars")

                ORLower, ORUpper = np.array([l_h, l_s, l_v]), np.array([u_h, u_s, u_v])

                mask = cv2.inRange(img_hsv, ORLower, ORUpper)
                # kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (30, 30))
                # closing = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

                result = cv2.bitwise_and(img, img, mask=mask)
                kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (30, 30))
                result = cv2.morphologyEx(result, cv2.MORPH_CLOSE, kernel)
                cv2.imshow('result', result)
                key = cv2.waitKey(1)                                                            #release video capture object
        cv2.destroyAllWindows()   
