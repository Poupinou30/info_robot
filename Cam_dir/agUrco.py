#Poti code de poupon pour la camera, posez votre plante devant

import cv2
import numpy as np

cap= cv2.VideoCapture(0)

aruco_dict= cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters_create()
fx=1000
fy=1000
cx=320
cy=240
mtx = np.array([[fx, 0, cx], [0, fx, cy], [0,0,1]])
dist = np.zeros((4,1))

while True:
	ret, frame= cap.read()
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
	#ici c'est la commande pour calibrer et pouvoir compenser la distorsion etc
	#ret, mtx, dist, rvecs, tvecs = cv/calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
	if ids is not None:
		
		cv2.aruco.drawDetectedMarkers(frame, corners, ids)
		
		for id_tag in ids :
			if(id_tag[0]==13):
				#print("plante mauve:")
				markerSizeInM=0.02
				
				rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, markerSizeInM,mtx,dist)
				print("Violet Plant coordinates (x,y,z):"+ str(tvec[0][0]) + " and rotation: " + str(rvec) +"\n") #
				
			if(id_tag[0]==36):
				markerSizeInM=0.02
				rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, markerSizeInM,mtx,dist)
				print("White Plant coordinates (x,y,z):"+ str(tvec[0][0]) + " and rotation: " + str(rvec) +"\n") #
			if(id_tag[0]==47):
				markerSizeInM=0.02
				rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, markerSizeInM,mtx,dist)
				print("Solar Panel coordinates (x,y,z):"+ str(tvec[0][0]) + " and rotation: " + str(rvec) +"\n") #
			
	cv2.imshow('Aruco Detection', frame)
	
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break
cap.release()
cv2.destroyAllWindows()
