#Poti code de poupon pour la camera, posez votre plante devant

import cv2
import numpy as np

def detect_tags():
	cap=cv2.VideoCapture(0)
	if not cap.isOpened():
		print("ERROR:la camera ne s'ouvre pas")
		return
	ret,frame = cap.read()
	if not ret:
		print("ERROR:la capture d'image ne se lance pas")
		cap.release()
		return
	aruco_dict= cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
	parameters = cv2.aruco.DetectorParameters_create()
	fx=500
	fy=500
	cx=320
	cy=240
	mtx = np.array([[fx, 0, cx], [0, fx, cy], [0,0,1]])
	dist = np.zeros((4,1))
	
	gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
	corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
	coordinate=[]
	if ids is not None:
		print("il y a " + str(len(ids)) + "plantes !")
		
		markerSizeInM=0.02
		rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, markerSizeInM,mtx,dist)
		for i in range(len(ids)):
			if(ids[i][0]==13 or ids[i][0]==36):
				angle=rvec[i][0][0]*(180.0/np.pi)
				if(angle<=140 and angle>=60 or angle<=-60 and angle>=-140):
					#print("la plante est de traviole attention")
					trav.append(i)
				coord=[]
				coord.append(tvec[i][0].tolist())
				print(coord[0])
				#coord.append(rvec[i][0].tolist())
			#print(coord)
			coordinate.append(coord[0])
			#print(str(coord)+" \n")
		if(len(ids)!=0):
			return coordinate
	else:
		print("Pas de plante")
		print(coordinate)
		return coordinate
		


def data_cam(img_path):
	cap=cv2.VideoCapture(0)
	if not cap.isOpened():
		print("ERROR:la camera ne s'ouvre pas")
		return
	ret,frame = cap.read()
	if not ret:
		print("ERROR:la capture d'image ne se lance pas")
		cap.release()
		return
	cv2.imwrite(img_path, frame)
	cap.release()
	
def calcul_aruco(img_path):
	image=cv2.imread(img_path)
	aruco_dict= cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
	parameters = cv2.aruco.DetectorParameters_create()
	fx=500
	fy=500
	cx=320
	cy=240
	mtx = np.array([[fx, 0, cx], [0, fx, cy], [0,0,1]])
	dist = np.zeros((4,1))
	
	gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
	corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
	#mask = np.zeros_like(frame)
	trav=[]
	
	if ids is not None :
		print("Y a des plantes")
		#print("il y a " + str(len(ids)) + " plantes")
		cv2.aruco.drawDetectedMarkers(image, corners, ids)
		coordinate=[]#ICI POUR LES COORDS DE TOUTES LES PLANTES
		markerSizeInM=0.02
		rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, markerSizeInM,mtx,dist)
		"""
		cv2.imshow('Detected Aruco!', image)
		cv2.waitKey(0)
		cv2.destroyAllWindows()
		"""
		
	else:
		print("Y a pas de plante?")
	
	
def data_cam_old():
	
	cap= cv2.VideoCapture(0)
	aruco_dict= cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
	parameters = cv2.aruco.DetectorParameters_create()
	fx=500
	fy=500
	cx=320
	cy=240
	mtx = np.array([[fx, 0, cx], [0, fx, cy], [0,0,1]])
	dist = np.zeros((4,1))

	while True:
		ret, frame= cap.read()
		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
		mask = np.zeros_like(frame)
		#print(corners)
		#ici c'est la commande pour calibrer et pouvoir compenser la distorsion etc
		#ret, mtx, dist, rvecs, tvecs = cv/calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
		trav=[]
		if ids is not None :
			#print("il y a " + str(len(ids)) + " plantes")
			cv2.aruco.drawDetectedMarkers(frame, corners, ids)
			coordinate=[]#ICI POUR LES COORDS DE TOUTES LES PLANTES
			markerSizeInM=0.02
			rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, markerSizeInM,mtx,dist)
			#print(rvec)
			#print(tvec)
			
			for i in range(len(ids)):
				if(ids[i][0]==13 or ids[i][0]==36):
					angle=rvec[i][0][0]*(180.0/np.pi)
					if(angle<=140 and angle>=60 or angle<=-60 and angle>=-140):
						#print("la plante est de traviole attention")
						trav.append(i)
					coord=[]
					coord.append(tvec[i][0].tolist())
					#print(coord[0])
					#coord.append(rvec[i][0].tolist())
				#print(coord)
				coordinate.append(coord[0])
				#print(str(coord)+" \n")
			if(len(ids)!=0):
				return coordinate
			#print(rvec)
			#print(tvec)
			#print(coordinate)
			#print(trav)
			#print("coordinates of the " + str(len(ids)) + " plants :"   + str(coordinate)+ " \n")
			coord_trav=[] #ICI POUR LES COORDONNEES DES PLANTES QUI SONT DE TRAVERS
			for k in range(len(trav)):
				coord_trav.append(coordinate[k])
				tag_corners=(corners[trav[k]][0])
				"""
				on calcule le centre du tag
				"""
				tag_center = np.mean(tag_corners, axis=0)
				rect_height=10
				rect_width= 10
				"""
				on convertit en taille de pixels
				"""
				"""
				print("haut gauche : " +str(tag_corners[0]))
				print("haut droite : " +str(tag_corners[1]))
				print("bas gauche : " +str(tag_corners[3]))
				print("bas droite : " +str(tag_corners[2]))
				"""
				
				
				cm_per_pixel= markerSizeInM*200/30
				
				h_px= int(rect_height/cm_per_pixel)
				
				w_px= int(rect_width/cm_per_pixel)
				top_left_corner = (int(tag_center[0] - w_px / 2), int(tag_center[1] - h_px / 2))
				bottom_right_corner = (int(tag_center[0] + w_px / 2), int(tag_center[1] + h_px / 2))
				
				#cv2.polylines(mask, [np.int32(tag_corners)], isClosed=True, color=(0,0,255), thickness=4)
				cv2.rectangle(mask, top_left_corner, bottom_right_corner, color=(0,0,255), thickness=4)
				output = cv2.add(frame, mask)
				
				
				
				#print(tag_corners)
			#print("les coords des pots de travers: " + str(coord_trav) + " \n")
			#print(" les tailles des listes: toutes les plantes : " + str(len(coordinate)) + " et juste les plantes de travers : " + str(len(coord_trav) )+ " \n")
			"""
			for i in range(len(ids)):
				r_deg=[]
				for j in range(3):
					r_deg.append(rvec[0][i][j]*(180.0/np.pi))
			#print(r_deg)
			if(r_deg[0]<=185 and r_deg[0]>=175 or r_deg[0]<=-175 and r_deg[0]>=-185):
				print("la plante est droite")
			if(r_deg[0]<=120 and r_deg[0]>=80 or r_deg[0]<=-80 and r_deg[0]>=-120):
				print("la plante est de traviole attention")
			
			for i in range(len(ids)) :
				markerSizeInM=0.02
				rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, markerSizeInM,mtx,dist)
				#print(tvec)
				#coordinate.append(tvec[0][0])
				print(coordinate)
			"""
			
			#print(coordinate)
			"""
			"""
			"""
				if(id_tag[0]==13):
					#print("plante mauve:")
					markerSizeInM=0.02
					
					rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, markerSizeInM,mtx,dist)
					#print("Violet Plant coordinates (x,y,z):"+ str(tvec[0][0]) + " and rotation: " + str(rvec) +"\n") #
					
				if(id_tag[0]==36):
					markerSizeInM=0.02
					rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, markerSizeInM,mtx,dist)
					#print("White Plant coordinates (x,y,z):"+ str(tvec[0][0]) + " and rotation: " + str(rvec) +"\n") #
				if(id_tag[0]==47):
					markerSizeInM=0.02
					rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, markerSizeInM,mtx,dist)
					#print("Solar Panel coordinates (x,y,z):"+ str(tvec[0][0]) + " and rotation: " + str(rvec) +"\n") #
			"""
		
			
		if(len(trav)!=0):
			cv2.imshow('Aruco Detection', output)
		else: 
			cv2.imshow('Aruco Detection', frame)
		
		if cv2.waitKey(1) & 0xFF == ord('q'):
			
			break

	cap.release()
	cv2.destroyAllWindows()
	

if __name__ == "__main__":
	
	detect_tags()
	#ça c'est si tu veux store
	"""
	img_path= 'test.jpg'
	data_cam(img_path)
	calcul_aruco(img_path)
	print("ça marche")
	"""
	#Ne pas enlever c'est la séquence de transfert
	"""
	coordinates= data_cam()
	
	for coord in coordinates: 
		coord_string = "[" + ", ".join(map(str, coord)) + "]" 
		print(coord_string)
	"""
	
