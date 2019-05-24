#!/usr/bin/env python
import rospy
import glob
import face_recognition
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String, Empty
from cv_bridge import CvBridge, CvBridgeError
import rospkg

class FaceRecognizer:
    def __init__(self):
        self.bridge = CvBridge()
        self.rospack = rospkg.RosPack()
        self.image_pub = rospy.Publisher("/face_recognizer/image",Image,queue_size=1)
        self.member_team_recog_pub = rospy.Publisher('/face_recognizer/member_team_detected', Empty, queue_size=1)
        rospy.Subscriber("/camera/rgb/image_raw", Image, self.imageCallback)
        self.known_face_encodings = []
        self.known_face_names = []
        self.load_ddbb()

    def load_ddbb(self):
        path = self.rospack.get_path('follow_person')
        filenames = glob.glob(path + '/people/*.jpg')
        for f in filenames:
            image = face_recognition.load_image_file(f)
            image_face_encoding = face_recognition.face_encodings(image)[0]
            self.known_face_encodings.append(image_face_encoding)
            self.known_face_names.append(f.split('/')[-1].split('.')[0])
            im = cv2.imread(f, 1)
            cv2.imshow("Database Image", im)
            cv2.waitKey(500)
            cv2.destroyAllWindows()
    def imageCallback(self, data):
        cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        small_frame = cv2.resize(cv_image, (0, 0), fx=0.25, fy=0.25)
        rgb_small_frame = small_frame[:, :, ::-1]
        face_locations = []
        face_encodings = []
        face_names = []

        ## Find all the faces and face encodings in the current frame of video
        face_locations = face_recognition.face_locations(rgb_small_frame)
        face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)
        face_names = []
        for face_encoding in face_encodings:
            # See if the face is a match for the known face(s)
            matches = face_recognition.compare_faces(self.known_face_encodings, face_encoding)
            name = "Unknown"
            # If a match was found in self.known_face_encodings, just use the first one.
            # if True in matches:
            #     first_match_index = matches.index(True)
            #     name = self.known_face_names[first_match_index]
            # Or instead, use the known face with the smallest distance to the new face
            face_distances = face_recognition.face_distance(self.known_face_encodings, face_encoding)
            best_match_index = np.argmin(face_distances)
            if matches[best_match_index]:
                name = self.known_face_names[best_match_index]
                face_names.append(name)
        if (name != "Unknown"):
            self.member_team_recog_pub.publish()
        ## Dispsay the results
        for (top, right, bottom, left), name in zip(face_locations, face_names):
            # Scale back up face locations since the frame we detected in was scaled to 1/4 size
            top *= 4
            right *= 4
            bottom *= 4
            left *= 4
            # Draw a box around the face
            cv2.rectangle(cv_image, (left, top), (right, bottom), (0, 0, 255), 2)
            # Draw a label with a name below the face
            cv2.rectangle(cv_image, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
            font = cv2.FONT_HERSHEY_DUPLEX
            cv2.putText(cv_image, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)
        # Display the resulting image
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "rgb8"))
        except CvBridgeError as e:
            print(e)

if __name__ == '__main__':
    rospy.init_node('face_recognizer_node', anonymous=True)
    face_recognizer = FaceRecognizer()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
