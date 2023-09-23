# Author: Srinjoy Choudhury
# Date: September 7th 2023

# Imports
import time

import cv2
import socket
import pickle
import numpy as np
from threading import Thread

import ultralytics
from ultralytics import YOLO
from ultralytics.utils.plotting import Annotator

from gpiozero import LED
from gpiozero.pins.pigpio import PiGPIOFactory
import keyboard

import serial

import pygame
import sys


# Getters setters and arrays used for sending data while threading
image_array = []


def setterFunc(newImage):
    image_array.append(newImage)


def getterFunc():
    return image_array


mobilityArray = []


def setMobilityArray(left, right, current, incoming):
    mobilityArray.append((left, right, current, incoming))


def getMobilityArray():
    return mobilityArray


proximityArray = []


def setProxArray(newMeasurements):
    proximityArray.append(newMeasurements)


def getProxArray():
    return proximityArray


def streamReceive():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ip = "RASPBERRY.PI.IP"  # raspberry pi IP
    port = 6666

    s.bind(('', port))

    counter = 0
    while True:
        x = s.recvfrom(1000000)
        clientip = x[1][0]
        data = x[0]

        data = pickle.loads(data)

        img = cv2.imdecode(data, cv2.IMREAD_COLOR)

        img = cv2.flip(img, 0)  # flips image
        img = cv2.flip(img, 1)

        image_array.append(img)

        key = cv2.waitKey(1)  # Reduce the waitKey delay to update the window smoothly

        if key == 27:  # Press 'Esc' key to exit the program
            break

    cv2.destroyAllWindows()


# This function receives and formats all data being transmitted from the Arduino
def ultrasonicComms():
    # Define the serial port and baud rate
    serial_port = 'COM7'  # Replace 'COMX' with the actual COM port of your HC-05
    baud_rate = 9600

    try:
        # Open the serial port
        ser = serial.Serial(serial_port, baud_rate)
        print(f"Connected to {serial_port} at {baud_rate} baud.")

        while True:
            # Read data from the HC-05 module
            received_data = ser.readline().decode('utf-8').strip()

            # Check if the received data contains the 'values' string
            if 'values' in received_data:
                values_string = received_data.split('values: ')[1]


                values_list = values_string.split(",")
                right = int(values_list[0])
                left = int(values_list[1])
                front = int(values_list[2])

                proxArraySend = [left, front, right]
                # Sending values
                setProxArray(proxArraySend)


    except serial.SerialException:
        print(f"Failed to connect to {serial_port}. Check the COM port and baud rate.")
    except KeyboardInterrupt:
        # Close the serial port when the script is terminated
        ser.close()
        print("Serial port closed.")


# Uses the ultrasonic communication data to create a user interface for our proximity guidance system
def ultrasonicUI():
    import pygame
    import sys

    # Initialize Pygame
    pygame.init()

    # Set up screen dimensions
    screen_width = 800
    screen_height = 600

    # Create the screen
    screen = pygame.display.set_mode((screen_width, screen_height))
    pygame.display.set_caption("Squares with Text on Black Screen")

    # Define colors
    black = (0, 0, 0)
    red = (255, 0, 0)
    green = (0, 255, 0)
    cyan = (0, 255, 255)
    text_color = (255, 255, 255)  # White text color

    # Define the rectangle's properties
    rect_width = 50
    rect_height = 100
    rect_x = (screen_width - rect_width) // 2
    rect_y = (screen_height - rect_height) // 2

    # Define the square dimensions
    square_size = 100

    # Define the position of the green square at the top of the screen
    green_square_x = (screen_width - square_size) // 2
    green_square_y = 10  # Adjust this value as needed to position the green square

    # Initialize the font
    pygame.font.init()
    font = pygame.font.Font(None, 50)  # Choose a font and font size

    # Create text surfaces
    text_surface = font.render("42", True, text_color)

    # Main game loop
    running = True
    while running:

        proxArray = getProxArray()
        lengthOfProxArray = len(proxArray)
        if lengthOfProxArray > 0:
            proxSet = proxArray[lengthOfProxArray - 1]
            proxLeft = proxSet[0]
            proxFront = proxSet[1]
            proxRight = proxSet[2]

            leftThreshold = 20
            rightThreshold = 20
            frontThreshold = 80

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

            leftText = font.render(str(proxLeft), True, text_color)
            rightText = font.render(str(proxRight), True, text_color)
            frontText = font.render(str(proxFront), True, text_color)

            # Fill the screen with black
            screen.fill(black)

            # Draw the red rectangle in the center
            pygame.draw.rect(screen, cyan, (rect_x, rect_y, rect_width, rect_height))

            # Draw the green squares on the left and right

            if proxLeft > leftThreshold or proxLeft == 0:
                pygame.draw.rect(screen, green, (0, (screen_height - square_size) // 2, square_size, square_size))
            else:
                pygame.draw.rect(screen, red, (0, (screen_height - square_size) // 2, square_size, square_size))
            screen.blit(leftText, (square_size // 2 - text_surface.get_width() // 2, (
                    screen_height - square_size) // 2 + square_size // 2 - text_surface.get_height() // 2))

            if proxRight > rightThreshold or proxRight == 0:
                pygame.draw.rect(screen, green, (
                screen_width - square_size, (screen_height - square_size) // 2, square_size, square_size))
            else:
                pygame.draw.rect(screen, red, (
                screen_width - square_size, (screen_height - square_size) // 2, square_size, square_size))
            screen.blit(rightText, (screen_width - square_size // 2 - text_surface.get_width() // 2, (
                    screen_height - square_size) // 2 + square_size // 2 - text_surface.get_height() // 2))

            # Draw the green square at the top of the screen
            if proxFront > frontThreshold and proxFront or 0:
                pygame.draw.rect(screen, green, (green_square_x, green_square_y, square_size, square_size))
            else:
                pygame.draw.rect(screen, red, (green_square_x, green_square_y, square_size, square_size))
            screen.blit(frontText, (green_square_x + square_size // 2 - text_surface.get_width() // 2,
                                    green_square_y + square_size // 2 - text_surface.get_height() // 2))

            # Update the display
            pygame.display.update()

    # Quit Pygame
    pygame.quit()
    sys.exit()


# areaCalculation used in computer vision detection function, to assess how much of each region the
# floor bounding box actually takes up.
def areaCalculation(x1, y1, x2, y2):
    length = y2 - y1
    width = x2 - x1

    return int(length * width)



# Pi General Purpose Input/Output communication configurations
factory = PiGPIOFactory(host='RASPBERRY.PI.IP')
pin_26 = LED(26, pin_factory=factory)
pin_16 = LED(16, pin_factory=factory)
pin_13 = LED(13, pin_factory=factory)
pin_12 = LED(12, pin_factory=factory)


# This function handles the car backing up until we find a left or right to turn into
def backUpUntil():
    movementArray = getMobilityArray()
    proxArray = getProxArray()
    lengthOfMovementArray = len(movementArray)
    lengthOfProxArray = len(proxArray)

    movementSet = movementArray[lengthOfMovementArray - 1]
    proxSet = proxArray[lengthOfProxArray - 1]
    proxLeft = proxSet[0]
    proxFront = proxSet[1]
    proxRight = proxSet[2]

    print("proxLeft: " + str(proxLeft))
    print("proxFront: " + str(proxFront))
    print("proxRight: " + str(proxRight))

    potentialLeft = movementSet[0]
    potentialRight = movementSet[1]
    currentPath = movementSet[2]
    incomingPath = movementSet[3]

    # Experiment with these values
    leftProxThreshold = 10
    rightProxThreshold = 10
    frontProxThreshold = 80

    leftExists = potentialLeft and proxLeft >= leftProxThreshold
    rightExists = potentialRight and proxRight >= rightProxThreshold
    forwardExists = currentPath and proxFront >= frontProxThreshold

    # Back Up until left or right become available.
    while not leftExists and not rightExists:
        pin_12.on()
        pin_16.on()
        pin_26.off()
        pin_13.off()


# Master mobility function, that handles all the motor control comms from my computer to the Pi, as well
# as has a manual control function.
def mobility():
    ##################################### MOBILITY CONTROLS #############################################

    manualOverride = True

    while True:

        # Manual override switch
        if keyboard.is_pressed("i"):
            if manualOverride:
                manualOverride = False
                print("Computer Vision Control")
            else:
                manualOverride = True
                print("In manual")

            time.sleep(30)

        if not manualOverride:
            movementArray = getMobilityArray()
            proxArray = getProxArray()
            lengthOfMovementArray = len(movementArray)
            lengthOfProxArray = len(proxArray)

            # print(lengthOfMovementArray)
            # print(lengthOfProxArray)
            if lengthOfMovementArray > 0 and lengthOfProxArray > 0:
                # Gets the latest movement set
                movementSet = movementArray[lengthOfMovementArray - 1]
                proxSet = proxArray[lengthOfProxArray - 1]
                proxLeft = proxSet[0]
                proxFront = proxSet[1]
                proxRight = proxSet[2]

                print("proxLeft: " + str(proxLeft))
                print("proxFront: " + str(proxFront))
                print("proxRight: " + str(proxRight))

                # print("Setup movement values")
                potentialLeft = movementSet[0]
                potentialRight = movementSet[1]
                currentPath = movementSet[2]
                incomingPath = movementSet[3]

                # Experiment with these values
                leftProxThreshold = 10
                rightProxThreshold = 10
                frontProxThreshold = 80

                leftExists = potentialLeft and proxLeft >= leftProxThreshold
                rightExists = potentialRight and proxRight >= rightProxThreshold
                forwardExists = currentPath and proxFront >= frontProxThreshold

                if forwardExists:
                    # Forward
                    print("FORWARD")
                    pin_26.on()
                    pin_13.on()
                    pin_16.off()
                    pin_12.off()
                else:
                    # For the moment I'm prioritizing left over right for testing purposes.
                    if leftExists:
                        # Left
                        print("LEFT")
                        pin_26.on()
                        pin_13.off()
                        pin_12.off()
                        pin_16.off()
                    elif rightExists:
                        # Right
                        print("RIGHT")
                        pin_13.on()
                        pin_26.off()
                        pin_12.off()
                        pin_16.off()
                    # If NONE exist:
                    else:
                        # Backup if there is no left right or forward.
                        # Backwards
                        print("BACKWARDS")
                        backUpUntil()
                        # pin_12.on()
                        # pin_16.on()
                        # pin_26.off()
                        # pin_13.off()
        else:
            if keyboard.is_pressed("s"):
                # Backwards
                pin_12.on()
                pin_16.on()
                # pin_26.on()
            elif keyboard.is_pressed("w"):
                # Forwards
                pin_26.on()
                pin_13.on()
                # pin_16.on()
            elif keyboard.is_pressed("a"):
                # Left
                pin_26.on()
                # pin_16.on()
            elif keyboard.is_pressed("d"):
                # Right
                pin_13.on()
            elif keyboard.is_pressed("q"):
                # Backwards Left
                pin_16.on()
                # pin_26.on()
            elif keyboard.is_pressed("e"):
                # Backwards right
                pin_12.on()
            else:
                pin_12.off()
                pin_13.off()
                pin_26.off()
                pin_16.off()


# This is the function in which our computer vision model is deployed. We get coordinates of every
# bounding box and adjust the confidence threshold of our model here.
def detectionDeployment():
    # ///////////////////////////////////////// PATH DETECTION ///////////////////////////////////////////////
    model_path = 'PATH/TO/MODEL'

    model = YOLO(model_path)

    manualOverride = False
    while True:
        potentialLeft = False
        potentialRight = False
        incomingPath = False
        currentPath = False

        theImageArray = getterFunc()
        if len(theImageArray) > 0:
            arrayLength = len(theImageArray)
            frame = theImageArray[arrayLength - 1]

            # Adjust confidence threshold here, with conf=
            results = model.predict(frame, show=False, conf=0.64)

            # Grabs the annotated frames the model generates
            annotated_frame = results[0].plot()
            image = np.array(annotated_frame)

            # ////// GETTING COORDINATES OF BBOXES //
            boxCoords = []
            howManyBoxes = 0
            for r in results:
                boxes = r.boxes
                for box in boxes:
                    b = box.xyxy[0]  # get box coordinates in (top, left, bottom, right) format
                    # Convert PyTorch tensor to a numpy array
                    # print(b)
                    howManyBoxes = howManyBoxes + 1
                    boxCoords.append([b[0], b[1], b[2], b[3]])

            # ////////////////////////////////////////// BBOX LOGIC /////////////////////////////////////////////


            for i in range(0, howManyBoxes):
                x1, y1, x2, y2 = boxCoords[i]  # Unpack the coordinates
                x1 = int(x1)
                x2 = int(x2)
                y1 = int(y1)
                y2 = int(y2)

                # If the floor detected is in the left panel in any way, calculate how much of the panel it occupies,
                # by taking it's area, dividing by the total area of the left panel, and multiplying by 100. If it's
                # 25 percent or more, left turn is possible. The area calculation is to account for any glitches, or
                # small overflows.

                lrPanelArea = 76000

                # Left turn panel ////////////
                if x1 < 160:
                    leftx2 = x2
                    if x2 > 160:
                        # x2 is now gonna be 160
                        leftx2 = 160
                    area = areaCalculation(x1, y1, leftx2, y2)
                    areaPercentage = (area / lrPanelArea) * 100
                    if areaPercentage >= 25:
                        cv2.rectangle(image, (int(x1), int(y1)), (int(leftx2), int(y2)), (0, 255, 255), 2)
                        # Draw a filled polygon where your text will go on, a little under x1
                        # Write text: Potential left
                        text = "Potential Left"

                        cv2.putText(image, text, (x1 + 10, y1 + 20), cv2.FONT_ITALIC, 0.5, (0, 255, 255), 2, 2)
                        potentialLeft = True

                # Right turn panel /////////////////

                if x2 > 480:
                    rightx1 = x1
                    if rightx1 < 480:
                        rightx1 = 480
                    area = areaCalculation(rightx1, y1, x2, y2)
                    areaPercentage = (area / lrPanelArea) * 100
                    if areaPercentage >= 25:
                        cv2.rectangle(image, (rightx1, y1), (x2, y2), (0, 255, 255), 2)
                        # Write text: Potential Right
                        text = "Potential Right"
                        cv2.putText(image, text, (rightx1 + 10, y1 + 20), cv2.FONT_ITALIC, 0.5, (0, 255, 255), 2, 2)
                        potentialRight = True

                newx1 = x1
                newy1 = y1
                newy2 = y2
                newx2 = x2

                imPathArea = 102400

                if 160 - newx1 >= 0:
                    newx1 = 160
                if 480 - newx2 <= 0:
                    newx2 = 480
                if 160 - newy1 >= 0:
                    newy1 = 160
                # Don't have to do y2 cause its at a boundary

                area = areaCalculation(newx1, newy1, newx2, newy2)
                areaPercentage = (area / imPathArea) * 100
                if areaPercentage >= 25:
                    cv2.rectangle(image, (newx1, newy1), (newx2, newy2), (0, 255, 0), 2)
                    text = "Immediate Path"
                    cv2.putText(image, text, (newx1 + 10, newy1 + 20), cv2.FONT_ITALIC, 0.5, (0, 255, 0), 2, 2)
                    currentPath = True

                # Incoming Path
                incomingPathArea = 76800
                newx1 = x1
                newy1 = y1
                newy2 = y2
                newx2 = x2

                if 160 - newx1 >= 0:
                    newx1 = 160
                if 480 - newx2 <= 0:
                    newx2 = 480
                if 240 - newy2 <= 0:
                    newy2 = 240

                text = "Incoming Path"
                if newy1 + 20 <= 200:
                    cv2.putText(image, text, (newx1 + 10, newy1 + 20), cv2.FONT_ITALIC, 0.5, (255, 255, 255), 2, 2)
                    incomingPath = True

                area = areaCalculation(newx1, newy1, newx2, newy2)
                areaPercentage = (area / incomingPathArea) * 100
                if areaPercentage >= 10:
                    cv2.rectangle(image, (newx1, newy1), (newx2, newy2), (255, 255, 255), 2)
                    text = "Incoming Path"


                # Updating mobility array
                setMobilityArray(potentialLeft, potentialRight, currentPath, incomingPath)

            # Display Predictions
            cv2.imshow("REALAnnotated", image)

            key = cv2.waitKey(1)  # Reduce the waitKey delay to update the window smoothly
            if key == 27:  # Press 'Esc' key to exit the program
                break

    cv2.destroyAllWindows()


if __name__ == "__main__":
    # Threading all the functions together
    Thread(target=streamReceive).start()
    Thread(target=detectionDeployment).start()
    Thread(target=ultrasonicComms).start()
    Thread(target=ultrasonicUI).start()
    # Waiting a bit to start mobility function, needs to start AFTER model deployment function (detectionDeployment)
    time.sleep(15)
    Thread(target=mobility).start()
