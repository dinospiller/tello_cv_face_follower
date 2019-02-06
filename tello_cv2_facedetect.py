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

def main():
    drone = tellopy.Tello()

    try:
        drone.connect()
        drone.wait_for_connection(60.0)

        container = av.open(drone.get_video_stream())
        # skip first 300 frames
        frame_skip = 300
        while True:
            for frame in container.decode(video=0):
                if 0 < frame_skip:
                    frame_skip = frame_skip - 1
                    continue
                start_time = time.time()
                image = cv2.cvtColor(np.array(frame.to_image()), cv2.COLOR_RGB2BGR)
                image=imutils.resize(image,width=700)
                image_bw=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)

                (faceRects,face_areas) = fd.track_faces(image_bw)

                for face_area in face_areas:
                    cv2.rectangle(image, (face_area[0], face_area[1]), (face_area[2], face_area[3]), (0, 255, 0), 2)
    
    
                cv2.imshow("Faces detected", image)
    
                print("I found: "+ str(len(faceRects)) +" face(s)")
                #cv2.waitKey(1)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                   break

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
