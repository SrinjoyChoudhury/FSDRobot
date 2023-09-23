

import os
import cv2
import socket
import pickle
import numpy as np
from threading import Thread

from gpiozero import LED
from gpiozero.pins.pigpio import PiGPIOFactory
import keyboard

def increase_brightness(img, value=30):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)

    lim = 255 - value
    v[v > lim] = 255
    v[v <= lim] += value

    final_hsv = cv2.merge((h, s, v))
    img = cv2.cvtColor(final_hsv, cv2.COLOR_HSV2BGR)
    return img

def snapShotFunc():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ip = "RASPBERRY.PI.IP"
    port = 6666

    s.bind(('', port))

    counter = 0
    while True:
        x = s.recvfrom(1000000)
        clientip = x[1][0]
        data = x[0]

        data = pickle.loads(data)

        img = cv2.imdecode(data, cv2.IMREAD_COLOR)

        img = cv2.flip(img, 0) # flips image
        img = cv2.flip(img, 1)

        # This function call increases brightness
        #img = increase_brightness(img, value=20)

        cv2.imshow('Img Server', img)

        key = cv2.waitKey(1)  # Reduce the waitKey delay to update the window smoothly


        if key == ord('v'):  # Press 'v' key to save the image
            # Specify the directory and filename where you want to save the image
            save_directory = "Where/You/Want/To/Save/Your/Images"
            filename = "imageNum" + str(counter) + ".png"
            cv2.imwrite(os.path.join(save_directory, filename), img)
            print("Image saved to:", save_directory + filename)
            counter = counter + 1

        if key == 27:  # Press 'Esc' key to exit the program
            break

    cv2.destroyAllWindows()


def controlFunc():
    factory = PiGPIOFactory(host='RASPBERRY.PI.IP') # Pi IP
    pin_26 = LED(26, pin_factory=factory)
    pin_16 = LED(16, pin_factory=factory)
    pin_13 = LED(13, pin_factory=factory)
    pin_12 = LED(12, pin_factory=factory)



    while True:
        if keyboard.is_pressed("s"):
            #Backwards
            pin_12.on()
            pin_16.on()
            #pin_26.on()
        elif keyboard.is_pressed("w"):
            #Forwards
            pin_26.on()
            pin_13.on()
            #pin_16.on()
        elif keyboard.is_pressed("a"):
            #Left
            pin_26.on()
            #pin_16.on()
        elif keyboard.is_pressed("d"):
            #Right
            pin_13.on()
        elif keyboard.is_pressed("q"):
            #Backwards Left
            pin_16.on()
            #pin_26.on()
        elif keyboard.is_pressed("e"):
            #Backwards right
            pin_12.on()
        else:
            pin_12.off()
            pin_13.off()
            pin_26.off()
            pin_16.off()





if __name__ == "__main__":
    Thread(target=snapShotFunc).start()
    Thread(target=controlFunc).start()