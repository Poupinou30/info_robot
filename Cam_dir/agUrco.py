import cv2
import numpy as np

cap= cv2.VideoCapture(0)

aruco_dict= cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters_create()
while True:
	ret, frame= cap.read()
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
	
	if ids is not None:
		cv2.aruco.drawDetectedMarkers(frame, corners, ids)
		for id_tag in ids :
			if(id_tag[0]==13):
				print("y a une plante mauve")
			if(id_tag[0]==36):
				print("y a une plante blanche")
	cv2.imshow('Aruco Detection', frame)
	
	if cv2.waitKey(1) & 0xFF == ord('q'):
		break
cap.release()
cv2.destroyAllWindows()
