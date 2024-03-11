import numpy as np
import cv2
import matplotlib.pyplot as plt
"""
image= cv2.imread("/home/student/cam_images/img_2012.jpg")

img=cv2.resize(image,(400,400))
img_white = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
window = 'Image'
cv2.imshow(window, img_white)


cv2.waitKey(0)

cv2.destroyAllWindows()


# Opening image
#img = cv2.imread("image.jpg")
   
# OpenCV opens images as BRG 
# but we want it as RGB We'll 
# also need a grayscale version
img_white = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
   
   
# Use minSize because for not 
# bothering with extra-small 
# dots that would look like STOP signs
stop_data = cv2.CascadeClassifier('stop_data.xml')
   
found = stop_data.detectMultiScale(img_white, 
                                   minSize =(20, 20)) #erreur ici
   
# Don't do anything if there's 
# no sign
amount_found = len(found)
   
if amount_found != 0:
       
    # There may be more than one
    # sign in the image
    for (x, y, width, height) in found:
           
        # We draw a green rectangle around
        # every recognized sign
        cv2.rectangle(img_rgb, (x, y), 
                      (x + height, y + width), 
                      (0, 255, 0), 5)
           
# Creates the environment of 
# the picture and shows it
plt.subplot(1, 1, 1)
plt.imshow(img_rgb)
plt.show()
"""

cap = cv2.VideoCapture(0)
#image = cv2.imread("/home/student/cam_images/plant_poupon.jpg")
#img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
#img_gris=cv2.resize(img_gray,(400,400))
#img=cv2.resize(image,(400,400))

while(1):

    _, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    cv2.imshow('test',rgb)
    # define range of white color in HSV
    # change it according to your need !
    #hsv_low=cv2.cvtColor(np.uint8([[[143,188,143]]])[0][0][0],cv2.COLOR_BGR2HSV)
    #hsv_high=cv2.cvtColor(np.uint8([[[152,251,152]]])[0][0][0],cv2.COLOR_BGR2HSV)
    lower_white = np.array([33,54,13], dtype=np.uint8)#110,100,100
    upper_white = np.array([80,136,53], dtype=np.uint8)# 130 255 255
    #110,50,50
    #130,255,255
    
    #50 100 170
    #70 255 255
    
    """
    les valeurs qui trouvent la farde : [36,25,25] to [70,255,255]
    les plantes détectées:[53,74,33] to [89,126,53]
    
    
    43,82,43 ------ 44,151,20
    #76,76,36
    110,86,57 ----105,83,17 (RGB)

    """
    
    #159,129,36 --- 45,77.4,62.4
    
    #67,42,9 --- 21,56.7,26.3
    
    
    # Threshold the HSV image to get only white colors
    mask = cv2.inRange(rgb, lower_white, upper_white)#ça c'est ok
    mask2 = cv2.inRange(rgb, lower_white, upper_white)
    # Bitwise-AND mask and original image
    #res = cv2.bitwise_and(frame,frame, mask= mask)#ça c'est chelou
    rmask= mask>0
    green= np.zeros_like(frame,np.uint8)
    green[rmask]=frame[rmask]
    #cv2.imshow('frame',frame)
    cv2.imshow('mask',mask)
    #cv2.imshow('res',res)
    cv2.imwrite("green_poupon.png",green)
    
    #Ici c'est juste pour avoir les points verts sur le fond blanc
    
    only_green= cv2.bitwise_and(frame, frame, mask=mask)
    
    """
    background=np.zeros(img.shape, img.dtype)
    background=cv2.bitwise_not(background)
    mask_inv=cv2.bitwise_not(mask)
    masked_bg= cv2.bitwise_and(background, background, mask = mask_inv)
    """
    final= cv2.add(frame, only_green) 
    cv2.imshow("img", final)
    """
    cv2.imwrite("green_poupon_v2.png",final)
    
    plot= cv2.imread("green_poupon_v2.png")
    plt.subplot(1, 1, 1)
    plt.imshow(plot)
    plt.show()
    """
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()
