import sys
import traceback
import tellopy
import av
import cv2.cv2 as cv2  # for avoidance of pylint error
import time
import imutils
import argparse
import numpy as np
import mahotas

import imutils
from facedetector import FaceDetector

fd = FaceDetector("haarcascade_frontalface_default.xml")

font=cv2.FONT_HERSHEY_SIMPLEX

def main():
    drone = tellopy.Tello()
    detection=0; # face detectio active flag

    try:
        drone.connect()
        drone.wait_for_connection(60.0)

        container = av.open(drone.get_video_stream())
        # skip first frames
        frame_skip = 30
        while True:
            for frame in container.decode(video=0):
                if 0 < frame_skip:
                    frame_skip = frame_skip - 1
                    continue
                start_time = time.time()
                image = cv2.cvtColor(np.array(frame.to_image()), cv2.COLOR_RGB2BGR)
                image=imutils.resize(image,width=700)
                
                if detection == 0:
                    cv2.putText(image,"press d to start face detection",(20,20),font,0.5,(0,255,0),2,cv2.LINE_AA);
                else:
                    image_bw=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)

                    (faceRects,face_areas) = fd.track_faces(image_bw)

                    ind = 1;
                    for face_area in face_areas:
                        if(ind==1): #distinguish between first and successive faces (follow only the first)
                            color=(0,255,0)
                        else:
                            color=(255,0,0)
                        cv2.rectangle(image, (face_area[0], face_area[1]), (face_area[2], face_area[3]), color, 2)
                        ind=ind+1;

                    cv2.putText(image,"press d to STOP detection",(20,20),font,0.5,(0,255,0),2,cv2.LINE_AA);

    
                cv2.putText(image,"press q to exit",(20,40),font,0.5,(0,255,0),2,cv2.LINE_AA);

                cv2.imshow("Drone Camera", image)
    
                #print("I found: "+ str(len(faceRects)) +" face(s)")
                but_pressed=cv2.waitKey(1)
                if but_pressed & 0xFF == ord("q"):
                    cv2.destroyAllWindows()
                    drone.quit()
                    break
                elif but_pressed & 0xFF == ord("d"):
                    if detection==1:
                        detection=0
                    else:
                        detection=1

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
