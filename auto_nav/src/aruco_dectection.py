import cv2 
import numpy as np
import math
from geometry_msgs.msg import Twist

#Load Aruco detector
parameters = cv2.aruco.DetectorParameters_create()     
aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_5X5_1000)

########## Calibrated camera #############
ret, mtx, dist, rvecs, tvecs = 0,0,0,0,0
mtx = np.matrix('646.27237076  , 0, 330.9224328 ; 0.,648.00983179 ,267.52126283;  0., 0.,1.')

#dist = np.matrix(' 0.04413318,-0.17487235,-0.00741481,-0.0007979, 0.20035146')
#new dist in MI
#dist = np.matrix('0.04754979 ,-0.26103821 ,-0.00308258 ,-0.00268376 , 0.35841967')
#dist = np.matrix('0.00073147 ,-0.10804118, -0.00672545,  0.00825231 , 0.0409711')
dist = np.matrix('0.009118, -0.070379, -0.001604, -0.005629, 0')   # 27/09/2022
#newcameramtx = np.matrix('643.81713867,0.,329.98954658; 0.,648.04632568,264.22654632;  0.,0., 1.')
#newcameramtx in MI

#newcameramtx = np.matrix('864.06524658,0.0,699.20575958;0.0,855.81317139,308.06186712;0.0,0.0,1.0')
newcameramtx = np.matrix('932.099514, 0, 630.482705; 0, 930.715334, 337.952393; 0, 0, 1')#calibration 27/9/2022

##################################################

def isRotationMatrix(R):
    Rt =np.transpose(R)
    shouldBeIdentity = np.dot(Rt,R)
    I = np.identity(3,dtype=R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6



def rotationMarixToEulerAngles(R):
    assert (isRotationMatrix(R))
    
    sy = math.sqrt(R[0,0] * R[0,0] + R[1,0] * R[1,0]) 
    
    singular = sy < 1e-6
    
    if not singular:
        x = math.atan2(R[2,1], R[2,2])
        y = math.atan2(R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
        
    else:
        x = math.atan2(R[1,2], R[1,1])
        y = math.atan2(R[2,0], sy)
        z = 0
        
    return np.array([x, y, z])
    



####################################################
# Load Cap
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH,1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT,720)


while True :
    marker_size = 10
    ret,frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # Change grayscale
       
    # Get Aruco marker
    corners,ids,rejected = cv2.aruco.detectMarkers(gray,aruco_dict,parameters=parameters,cameraMatrix = newcameramtx,distCoeff = dist)
    print(corners)

    if np.all(ids is not None):
        cv2.aruco.drawDetectedMarkers(frame, corners)  # Draw A square around the markers
        
        #ret = cv2.aruco.estimatePoseSingleMarkers(corners, marker_size, newcameramtx, dist)
        rvec_list_all, tvec_list_all, _objPoints= cv2.aruco.estimatePoseSingleMarkers(corners, marker_size, newcameramtx, dist)
        # Draw polygon around the marker
        #int_corners = np.int0(corners)
        rvec = rvec_list_all[0][0]
        tvec = tvec_list_all[0][0]
        
        
        #rvec = ret[0][0,0,:]
        #tvec = ret[1][0,0:]
        
        cv2.aruco.drawAxis(frame, newcameramtx, dist, rvec, tvec, 6)  # Draw Axis
        tvec_str = "x=%f" "y=%f" "z=%f"%(tvec[0],tvec[1],tvec[2])
        cv2.putText(frame,tvec_str,(20,670),cv2.FONT_HERSHEY_PLAIN,2,(0,0,255),2,cv2.LINE_AA) 
        
        rvec_flipped = rvec* -1
        tvec_flipped = tvec* -1
        
        rotation_matrix, jacobian = cv2.Rodrigues(rvec_flipped)
        realworld_tvec = np.dot(rotation_matrix,tvec_flipped)
        #print("rotationMat",rotationMarixToEulerAngles(rotation_matrix))
        roll, pitch, yaw = rotationMarixToEulerAngles(rotation_matrix)
        Angle = "roll=%f pitch=%f yaw=%f"%(math.degrees(roll), math.degrees(pitch), math.degrees(yaw))
        cv2.putText(frame,Angle,(20,710),cv2.FONT_HERSHEY_PLAIN,2,(0,0,255),2,cv2.LINE_AA)
        pub = rospy.Publisher('aruco', Twist(), quene_size = 10)
        rospy.init_node('aruco_pose', anonymous = True)
        arupose = Twist()
        arupose.linear.x = tvec[0]
        arupose.linear.y = tvec[1]
        arupose.linear.z = tvec[2]

        rospy.loginfo(arupose)
        pub.publish(arupose)
        
        #print("tvec:",tvec)
        #print("tvec1:",tvec[0][0])
        #cv2.polylines(frame,int_corners,True,(0,255,0),5)
        #print("rvec :",rvec)
        #print("tvec :", tvec)
        # Aruco Perimeter
        #aruco_perimeter = cv2.arcLength(corners[0], True)
        # Pixel to cm ratio
        #pixel_cm_ratio = aruco_perimeter /20
        #rect = cv2.minAreaRect(corners)
        #(x, y), (w, h), angle = [[corners]]
    """
        for cnt in range (0,len(ids)):
            
            rvec, tvec,_ = cv2.aruco.estimatePoseSingleMarkers(corners,marker_size,newcameramtx,dist)
            
            print("rvec :",rvec)
            print("tvec :", tvec)
            cv2.aruco.drawDetectedMarkers(frame, corners)  # Draw A square around the markers
            cv2.aruco.drawAxis(frame, newcameramtx, dist, rvec[cnt], tvec[cnt], 7)  # Draw Axis
            #cv2.circle(frame, (int(x), int(y)), 5, (0, 0, 255), -1)

        # Draw object boundaries
    """


    cv2.imshow("Image",frame)
    key = cv2.waitKey(1)
    if key == 27:
        break

cap.release()
cv2.destroyAllWindows()