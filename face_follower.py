import imutils
from facedetector import FaceDetector
import tellopy
import cv2.cv2 as cv2  # for avoidance of pylint error
from numpy import linalg

class FaceFollower:
    def __init__(self,drone):
        self.drone=drone
        self.detecting=0
        self.following=0
        self.fd = FaceDetector("haarcascade_frontalface_default.xml")
        self.droneflying=0
        self.kpki_vert   = (0.35, 0.05)
        self.kpki_lateral= (0.35, 0.05)
        self.kpki_frontal= (0.8, 0.05)
        self.last_face_followed=(0,0,0,0)

    def on_key_pressed(self,key):
        if key & 0xFF == ord("s"):
            if self.detecting == 1:
                self.stop_drone()
                self.detecting = 0
            else:
                self.detecting = 1

        elif key == 13:  # enter pressed
            self.stop_drone()
        elif key == 32 : #space pressed
            if self.droneflying==0:
                self.stop_drone()
                self.drone.takeoff()
                self.drone.takeoff()#redoundant
                self.droneflying=1
            else:
                self.stop_drone()
                self.drone.land()
                self.drone.land()#redoundant
                self.droneflying=0
        elif key & 0xFF == ord("u"):
            self.stop_drone()
            self.drone.up(30)
        elif key & 0xFF == ord("d"):
            self.stop_drone()
            self.drone.up(-30)
        elif key & 0xFF == ord("l"):
            self.stop_drone()
            self.drone.left(30)
        elif key & 0xFF == ord("r"):
            self.stop_drone()
            self.drone.left(-30)
        elif key & 0xFF == ord("f"):
            self.stop_drone()
            self.drone.forward(30)
        elif key & 0xFF == ord("b"):
            self.stop_drone()
            self.drone.forward(-30)
        elif key & 0xFF == ord("c"):
            self.stop_drone()
            self.drone.clockwise(30)
        elif key & 0xFF == ord("v"):
            self.stop_drone()
            self.drone.clockwise(-30)

    def stop_drone(self):
        self.drone.left(0)
        self.drone.up(0)
        self.drone.forward(0)
        self.drone.clockwise(0)

    def is_detecting(self):
        return self.detecting==1

    def process_frame(self,image):
        image_bw = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        (faceRects, face_areas) = self.fd.track_faces(image_bw)
        (h, w) = image.shape[:2]
        last_rect_center=self.find_center(self.last_face_followed)
        ind = 1;
        min_distance_from_prev=10000;
        for face_area in face_areas:
            # face_area: (upper_left(x,y),lower_right(x,y)): [0,1,2,3]
            # retrieve the rectangle center
            center = self.find_center(face_area)
            # calculate distance from the center of previously-followed target
            distance_from_prev=linalg.norm(((center[0]-last_rect_center[0]),(center[1]-last_rect_center[1])))
            # promote the nearest rectangle
            if(distance_from_prev<=min_distance_from_prev):
                min_distance_from_prev=distance_from_prev
                self.last_face_followed=face_area
            # color all rectangles in green
            cv2.rectangle(image, (face_area[0], face_area[1]), (face_area[2], face_area[3]), (0,255,0), 2)
            ind=ind+1

        if(ind>1): #if we found at least one rectangle
            # color the target rectangle in red
            print("FACE CHOESEN!")
            cv2.rectangle(image, (self.last_face_followed[0], self.last_face_followed[1]), (self.last_face_followed[2], self.last_face_followed[3]), (0, 0, 255), 2)
            rect_width = self.last_face_followed[2] - self.last_face_followed[0]
            rect_center = self.find_center(self.last_face_followed)
            if self.detecting == 1:
                self.tracking_vert_loop((h, w),rect_center)
                self.tracking_lateral_loop((h, w),rect_center)
                self.tracking_frontal_loop((h, w),rect_width)
        else:
            self.stop_drone()

        return image

    def tracking_vert_loop(self,image_size,rect_center):
        image_center = (image_size[0] / 2, image_size[1] / 2)
        vert_error=(image_center[0]-rect_center[0])# remember that up has negative slope in pixels!
        #print("image: V", image_center[0], ",H", image_center[1], " rect: V", rect_center[0], ",H", rect_center[1],"Verr:",vert_error)
        up_strength=int(self.kpki_vert[0]*vert_error)
        #print("up_strength:",up_strength)
        # simple proportional control
        self.drone.up(up_strength)

    def tracking_lateral_loop(self,image_size,rect_center):
        image_center = (image_size[0] / 2, image_size[1] / 2)
        lateral_error=(image_center[1]-rect_center[1])
        clockwise_strength=-(int(self.kpki_lateral[0]*lateral_error))
        #print("lateral_strength:",clockwise_strength)
        # simple proportional control
        self.drone.clockwise(clockwise_strength)

    def tracking_frontal_loop(self,image_size,rect_width):
        image_width=image_size[1]
        frontal_error=image_width/6-rect_width
        frontal_strength=(int(self.kpki_frontal[0]*frontal_error))
        #print("frontal_strength:",frontal_strength)
        # simple proportional control
        self.drone.forward(frontal_strength)

    def find_center(self,rect_vertices):
        rect_center = ((rect_vertices[3] + rect_vertices[1]) / 2, (rect_vertices[2] + rect_vertices[0]) / 2)
        return rect_center