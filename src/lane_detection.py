import cv2
import numpy as np

cap = cv2.VideoCapture('robot_lane.mp4')

while(cap.isOpened()):
 
    ret, frame = cap.read()
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    h = hsv.shape[0]
    w = hsv.shape[1]

    lower_color= np.array([0,0,150])
    upper_color = np.array([360,255,255])
    range = cv2.inRange(hsv, lower_color, upper_color )
    #Thresholding
    #(t,thresh) = cv2.threshold(gray, 170,255,cv2.THRESH_BINARY)
  
    #ROI
    mask = np.zeros_like(range)
    roi = np.array([[
        (0,h),
        (220,0),
        (420,0),
        (w,h) ]], np.int32)
    cv2.fillPoly(mask, roi, 255)
    masked = cv2.bitwise_and(range, mask)

    contours, hierarchy = cv2.findContours(masked, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)

    m = cv2.moments(masked, False)
    try:
        cx,cy = m['m10']/m['m00'], m['m10']/m['m00']
    except ZeroDivisionError:
        cy,cx = h/2,w/2

    error = cx - w/2
    error = round(error,2)
    cv2.circle(masked,(int(cx),int(cy)), 10,(0,255,0),-1)
    
    cv2.putText(masked, str(error),(int(cx),int(cy)),cv2.FONT_HERSHEY_SIMPLEX,1,(0,255,0),2, cv2.LINE_AA)
    if ret == True:

        cv2.imshow('MASK',masked)
        cv2.imshow('Original',frame)

    # Press Q on keyboard to  exit
    if cv2.waitKey(25) & 0xFF == ord('q'):
      break
cap.release()

# Closes all the frames
cv2.destroyAllWindows()