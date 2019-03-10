import sys
import traceback
import tellopy
import av
import cv2.cv2 as cv2  # for avoidance of pylint error
import time
import numpy as np
import imutils
from face_follower import FaceFollower

prev_flight_data=""

def print_onscreen_instructions(img,detecting):
    font = cv2.FONT_HERSHEY_SIMPLEX
    if detecting == 0:
        cv2.putText(img, "press s to start following", (20, 20), font, 0.5, (0, 255, 0), 2, cv2.LINE_AA);
    else:
        cv2.putText(img, "press s to STOP following", (20, 20), font, 0.5, (0, 255, 0), 2, cv2.LINE_AA);
    cv2.putText(img,"press q to exit",(20,40),font,0.5,(0,255,0),2,cv2.LINE_AA);
    cv2.putText(img, "U=up, D=down", (20,60), font, 0.5, (0, 255, 0), 2, cv2.LINE_AA);
    cv2.putText(img, "L=left, R=right", (20,80), font, 0.5, (0, 255, 0), 2, cv2.LINE_AA);
    cv2.putText(img, "F=fwd, B=backwd", (20,100), font, 0.5, (0, 255, 0), 2, cv2.LINE_AA);
    cv2.putText(img, "C=clockwise, V=c.clockwise", (20, 120), font, 0.5, (0, 255, 0), 2, cv2.LINE_AA);
    cv2.putText(img, "SPACE to takeoff/land", (20,140), font, 0.5, (0, 255, 0), 2, cv2.LINE_AA);
    cv2.putText(img, "ENTER to STOP all movements", (int(img.shape[1]/6), img.shape[0]-40), font, 1.0, (0, 0, 255), 2, cv2.LINE_AA);
    cv2.putText(img, prev_flight_data, (int(img.shape[1]/5), img.shape[0]-15), font, 0.5, (0, 255, 255), 2, cv2.LINE_AA);
    return img

def flightDataHandler(event, sender, data):
    global prev_flight_data
    text = str(data)
    if prev_flight_data != text:
        prev_flight_data = text

def main():
    drone = tellopy.Tello()
    global detection; # face detectio active flag
    ff = FaceFollower(drone)

    try:
        drone.connect()
        drone.wait_for_connection(60.0)

        container = av.open(drone.get_video_stream())
        drone.subscribe(drone.EVENT_FLIGHT_DATA, flightDataHandler)

        # skip first frames
        frame_skip = 30
        while True:
            for frame in container.decode(video=0):
                if 0 < frame_skip:
                    frame_skip = frame_skip - 1
                    continue
                start_time = time.time()
                image = cv2.cvtColor(np.array(frame.to_image()), cv2.COLOR_RGB2BGR)
                image=imutils.resize(image,width=500)
                
                image=ff.process_frame(image)

                image = imutils.resize(image, width=800)
                image=print_onscreen_instructions(image,ff.is_detecting());
                cv2.imshow("Drone Camera", image)
    
                #print("I found: "+ str(len(faceRects)) +" face(s)")
                but_pressed=cv2.waitKey(1)
                if but_pressed & 0xFF == ord("q"):
                    drone.land()
                    drone.land()
                    drone.land()
                    cv2.destroyAllWindows()
                    drone.quit()
                    break
                else:
                    ff.on_key_pressed(but_pressed)

                if frame.time_base < 1.0/60:
                    time_base = 1.0/60
                else:
                    time_base = frame.time_base
                frame_skip = int((time.time() - start_time)/time_base)
                    

    except Exception as ex:
        exc_type, exc_value, exc_traceback = sys.exc_info()
        traceback.print_exception(exc_type, exc_value, exc_traceback)
        print(ex)
    finally:
        drone.quit()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
