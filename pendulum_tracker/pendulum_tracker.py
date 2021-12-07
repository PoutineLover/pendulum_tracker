import numpy as np
import argparse
import cv2
import math

pivot_x, pivot_y = 0, 0
pivot_created = False

def track_pivot(event,x,y,flags,param):
    global pivot_x
    global pivot_y
    global pivot_created
    if event == cv2.EVENT_LBUTTONUP:
        pivot_x = x
        pivot_y = y
        pivot_created = True


if __name__ == "__main__":
    # Video
    cap = cv2.VideoCapture('QFactor3_Edited.mp4')
    if (cap.isOpened()== False): 
        print("Error opening video  file")

    # Resizing the frame
    ret, frame = cap.read()
    frame = cv2.resize(frame, (0, 0), fx=0.75, fy=0.75)

    # Set mouse events to window
    cv2.namedWindow('Select Pivot')
    cv2.setMouseCallback('Select Pivot', track_pivot)

    # Waits for the user to mark the coordinates of the pivot
    while(not pivot_created):
        cv2.imshow("Select Pivot", frame)
        key = cv2.waitKey(2)
        if key == ord('q'):
            break
    
    cv2.destroyAllWindows()
    print(pivot_x, pivot_y)

    # Variables that are used to print the amplitude at each time w/ uncertainties into a txt file
    file = open('delete_thisblah.txt', 'w')
    counter = 1
    SECONDS_PER_FRAME = 1/30 # s
    TIME_UNCERTAINTY = SECONDS_PER_FRAME/2
    AMPLITUDE_UNCERTAINTY = 2/(math.pi*55) # 2 cm radius of washer over 55 cm length
    file.write("Time Amplitude Time_Unc Amp_Unc \n")
    max_amplitude = 0
    found = False
    two_radians_back = 0
    one_radian_back = 0

    # Used to help count periods
    maximum_counter = 0
    num_complete_oscillations = 0
    num_frames = 0

    # Runs the rest of the video, finding the angle between the pivot and the centroid of the pendulum
    while True:

        # Reads each frame of the video
        ret, frame = cap.read()
        if ret == True:
            # Changes color space to HSV, which makes it easier to track colors
            frame = cv2.resize(frame, (0, 0), fx=0.75, fy=0.75)
            hsvFrame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Colors (Already converted to HSV), lower_red is a light pink, while upper_red is a dark red
            lower_red = np.array([165, 100, 20])
            upper_red = np.array([170, 255, 225])

            # Isolates the pendulum, leaving the rest of the environment black
            # Bitwise_and usually handles two different frames, but it doesn't have to
            mask = cv2.inRange(hsvFrame, lower_red, upper_red)
            mask_result = cv2.bitwise_and(frame, frame, mask=mask)

            # This blurs what is left so that it can be better used to calculate the centroid
            kernel = np.ones((3,3), np.uint8)
            dilate_result = cv2.dilate(mask_result, kernel, iterations=1)

            # Dimensions of result frame
            height = dilate_result.shape[0]
            width = dilate_result.shape[1]

            # Converts each pixel in result to its grayscale version
            rgb = cv2.cvtColor(dilate_result, cv2.COLOR_HSV2BGR)
            gray = cv2.cvtColor(rgb, cv2.COLOR_BGR2GRAY)
            
            #cv2.imshow('Grayscale', result)

            ret, thresh = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY)

            M = cv2.moments(thresh)

            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

            # Draws the location of the pivot and the centre of mass, along with the lines they make
            cv2.circle(frame, (cX, cY), 5, (252, 148, 3), -1)
            cv2.circle(frame, (pivot_x, pivot_y), 5, (0, 0, 255), -1)
            cv2.line(frame, (pivot_x, pivot_y), (cX, cY), (255, 255, 255), 1)
            cv2.line(frame, (pivot_x, pivot_y), (pivot_x, pivot_y + 100), (255, 255, 0), 1)
            cv2.imshow('Centroid', frame)

            # Calculates the angle the pendulum makes with the vertical
            angle_vector = [cX - pivot_x, cY - pivot_y]
            vertical_vector = [0, 100]
            radian = np.arccos(np.dot(angle_vector, vertical_vector)/(np.linalg.norm(angle_vector) * np.linalg.norm(vertical_vector)))
            if (cX < pivot_x):
                radian = -radian

            # Writes the needed information into the same file
            write_string = "{0:.4f}".format(counter * SECONDS_PER_FRAME) + " " + "{0:.4f}".format(radian) + " " + "{0:.4f}".format(TIME_UNCERTAINTY) + " " + "{0:.4f}".format(TIME_UNCERTAINTY) + "\n"
            #print(write_string)
            file.write(write_string)

            # Finds the frame at which amplitude is e^(-pi/3)% of the initial by first identifying 
            # when a frame is at a maximum amplitude. It then compares the measured amplitude
            # to the maximum amplitude at the start of the video
            if counter == 1:
                max_amplitude = radian
                two_radians_back = radian
            else:
                # This is when a maximum exists
                if abs(one_radian_back) > abs(two_radians_back) and abs(one_radian_back) > abs(radian) and not found: 
                    print(f"Maximum found at frame {'{0:.4f}'.format((counter-1) * SECONDS_PER_FRAME)} with amplitude {one_radian_back}")
                    maximum_counter = 1
                    if abs(one_radian_back) < abs(max_amplitude)*math.exp(-math.pi/3):
                        print(f"First e^(-pi/3) amplitude reached at {'{0:.4f}'.format((counter - 1) * SECONDS_PER_FRAME)}")
                        found = True
                    if(maximum_counter % 2 == 0):
                        num_complete_oscillations += 1
                # Updates the previous two radians each time
                two_radians_back = one_radian_back
                one_radian_back = radian
            counter += 1
            num_frames += 1

            # Protocols to exit the recording early
            key = cv2.waitKey(1)
            # Press Q on keyboard to  exit
            if key == ord('q'):
                break
            elif key == ord('p'):
                cv2.waitKey(0) #wait until any key is pressed
        else:
            cap.release()
            cv2.destroyAllWindows()
            break

    print(f"Number of periods found: {num_complete_oscillations}")
    print(f"Total time of video: {num_frames * SECONDS_PER_FRAME}")
    print(f"Average period: {num_frames * SECONDS_PER_FRAME / num_complete_oscillations}")

