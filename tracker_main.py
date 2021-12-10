# import the necessary packages
from picamera.array import PiRGBArray
from picamera import PiCamera
from imutils.video import VideoStream
from shapedetector import ShapeDetector
from collections import deque
import time
import sys
import smbus2
import cv2
import numpy as np
import argparse
import imutils
import serial
import os

def capture_quick_image():
    '''
    Captures image from Pi camera and displays to screen
    Image is not saved
    '''
    # initialize the camera and grab a reference to the raw camera capture
    camera = PiCamera()
    rawCapture = PiRGBArray(camera)
    # allow the camera to warmup
    time.sleep(0.1)
    # grab an image from the camera
    camera.capture(rawCapture, format="bgr")
    image = resize(image=rawCapture.array, ratio=0.5)
    # display the image on screen and wait for a keypress
    cv2.imshow("Image", image)
    cv2.waitKey(0)


def image_capture():
    '''
    '''
    # initialize the camera and grab a reference to the raw camera capture
    camera = PiCamera()
    rawCapture = PiRGBArray(camera)
    # allow the camera to warmup
    time.sleep(0.1)
    # grab an image from the camera
    camera.capture(rawCapture, format="bgr")
    image = image=rawCapture.array

    return image


def start_video_stream():
    '''
    Starts Pi camera video stream and displays to screen
    q key to break
    '''
    # initialize the camera and grab a reference to the raw camera capture
    camera = PiCamera()
    camera.resolution = (640, 480)
    camera.framerate = 32
    rawCapture = PiRGBArray(camera, size=(640, 480))
    # allow the camera to warmup
    time.sleep(0.1)
    # capture frames from the camera
    for frame in camera.capture_continuous(rawCapture, format="bgr", use_video_port=True):
        # grab the raw NumPy array representing the image, then initialize the timestamp
        # and occupied/unoccupied text
        image = frame.array
        # show the frame
        cv2.imshow("Frame", image)
        key = cv2.waitKey(1) & 0xFF
        # clear the stream in preparation for the next frame
        rawCapture.truncate(0)
        # if the `q` key was pressed, break from the loop
        if key == ord("q"):
            break

def conv_to_grayscale(image):
    '''
    takes a BGR image and converts to grayscale
    :param image:
    :return:
    '''

    # Create new image converted to grayscale
    result = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    return result


def conv_to_hsv(image):
    '''
    takes a BGR image and converts to
    :param image:
    :return:
    '''

    # Convert image to hsv
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    return hsv


def bgr_mask(image, bgr=[255,255,255], thresh=[255,255,255]):
    '''
    Masks an image at specified blue/green/red values between 0-255 with threshold thresh
    :param image: input image to mask (string)
    :param bgr: bgr array of blue/gree/red
    :param thresh: threshold array of threshold values
    :return:
    '''

    im = cv2.imread(image)

    minBGR = np.array([bgr[0] - thresh[0], bgr[1] - thresh[1], bgr[2] - thresh[2]])
    maxBGR = np.array([bgr[0] + thresh[0], bgr[1] + thresh[1], bgr[2] + thresh[2]])

    maskBGR = cv2.inRange(im, minBGR, maxBGR)

    return maskBGR


def resize(image, ratio=1):
    '''
    Return a resized version of an image
    :param image:
    :param ratio:
    :return:
    '''
    # Grab dimensions of image object
    (h,w,d) = image.shape

    # Calculate resize dimensions from ratio input
    dim = (int(ratio*w), int(ratio*h))

    # Create result image with resized dimensions
    result = cv2.resize(image, dim)

    return result


def track_ball():
    '''
    '''
    # Setup arguments for terminal
    break_code = False
    ap = argparse.ArgumentParser()
    ap.add_argument("-v", "--video", help="path to the video file")
    ap.add_argument("-b", "--buffer", help = "max buffer size")
    args = vars(ap.parse_args())

    # Define ball color
    ball_color = 'yellow'

    if ball_color == 'red':
        lower_mask = np.array([150,150,50])
        upper_mask = np.array([180,255,235])
    elif ball_color == 'yellow':
        lower_mask = np.array([20,100,100])
        upper_mask = np.array([30,255,255])

    pts = deque(maxlen=args["buffer"])

    # Start video capture
    if not args.get("video", False):
        vs = VideoStream(src=0).start()

    else:
        vs = cv2.VideoCapture(args["video"])

    time.sleep(2.0)

    # Connect with Arduino for I2C data stream
    with serial.Serial("/dev/ttyACM0", 9600, timeout=1) as arduino:
        time.sleep(2.0)  # wait for serial to open
        if arduino.isOpen():
            try:
                while True:
                    frame = vs.read()

                    frame = frame[1] if args.get("video", False) else frame

                    if frame is None:
                        break

                    frame = imutils.resize(frame, width=600)
                    blurred = cv2.GaussianBlur(frame, (11, 11), 0)
                    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)

                    mask = cv2.inRange(hsv, lower_mask, upper_mask)
                    mask = cv2.erode(mask, None, iterations=2)
                    mask = cv2.dilate(mask, None, iterations=2)

                    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    cnts = imutils.grab_contours(cnts)
                    center = None

                    if len(cnts) > 0:
                        if True:
                            c = max(cnts, key=cv2.contourArea)

                            if True:
                                ((x,y), radius) = cv2.minEnclosingCircle(c)

                                # Filter on minimum allowable size of object
                                if radius > 15:
                                    # Calculate moment of object to find center
                                    M = cv2.moments(c)
                                    center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))

                                    # Display text & circle on tracked object
                                    cv2.putText(frame, str(center), (int(x), int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                                    cv2.putText(frame, 'Radius: {}'.format(str(radius)), (500, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                                    cv2.circle(frame, (int(x), int(y)), int(radius), (0, 255, 0), 2)

                                    # Send serial data
                                    timeout = time.time()
                                    x_serial = 'CAMX|{}'.format(int(x))
                                    r_serial = 'CAMR|{}'.format(int(radius))
                                    # print('flag1')
                                    arduino.write(x_serial.encode())
                                    print('flag2')
                                    while arduino.inWaiting() == 0:
                                        time_new = time.time()
                                        if time_new > timeout + 5:
                                            break_code = True # Break if no response
                                            break
                                        #pass
                                    if arduino.inWaiting() > 0:
                                        #answer = arduino.readline()
                                        # print(answer)
                                        # time.sleep(1)
                                        print(x_serial)
                                        arduino.flushInput()
                                    timeout = time.time()
                                    arduino.write(r_serial.encode())
                                    while arduino.inWaiting() == 0:
                                        time_new = time.time()
                                        if time_new > timeout + 5:
                                            break_code = True # Break if no response
                                            break
                                        pass
                                    if arduino.inWaiting() > 0:
                                        # answer = arduino.readline()
                                        # print(answer)
                                        # time.sleep(1)
                                        print(r_serial)
                                        arduino.flushInput()

                                '''
                                while arduino.inWaiting() == 0: pass
                                if arduino.inWaiting() > 0:
                                    answer = arduino.readline()
                                    print(answer)
                                    arduino.flushInput()  # remove data after reading
                                else:
                                    pass
                                    '''

                    path = '/home/pi/Documents/Pictures'
                    cv2.imshow("Frame", frame)
                    key = cv2.waitKey(1) & 0xFF

                    if key == ord("q") or break_code == True:
                        cv2.imwrite(os.path.join(path, 'track_img.jpg'), frame)
                        break

            except KeyboardInterrupt:
                print("KeyboardInterrupt has been caught.")



    if not args.get("Video", False):
        vs.stop()

    else:
        vs.release()

    cv2.destroyAllWindows()
    return break_code


def send_test_msg():
    '''
    reg_write_dac = 0x40
    address = 0x60

    channel = 1
    bus = smbus.SMBus(channel)

    msg = 0xFF
    bus.write_i2c_block_data(address, reg_write_dac, msg)

    ser = serial.Serial('/dev/ttyACM0', 9600)
    s = [0,1]
    while True:
        read_serial = ser.readline()
        s[0] = str(int(ser.readline(),16))
        print(s[0])
        print(read_serial)

    ser = serial.Serial('/dev/ttyACM0', 9600)
    ser.write(bytes('0x55', 'utf-8'))
    while True:
        time.sleep(0.5)
        data = ser.readline()
        print(data)
    '''


    pass



# Run Main
if __name__ == '__main__':

    run_code = True
    while run_code:
        run_code = track_ball()