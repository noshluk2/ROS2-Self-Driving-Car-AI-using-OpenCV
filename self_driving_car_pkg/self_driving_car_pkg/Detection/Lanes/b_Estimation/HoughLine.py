import cv2
import numpy as np

cv2.namedWindow("[ProbHough] Prob_houghLines",cv2.WINDOW_NORMAL)
minLineLength = 10
maxLineGap = 20
thresh = 10

def minLineLength_Change(val):
    global minLineLength
    minLineLength = val
def maxLineGap_Change(val):
    global maxLineGap
    maxLineGap = val
def thresh_Change(val):
    global thresh
    thresh = val


cv2.createTrackbar("minLineLength","[ProbHough] Prob_houghLines",minLineLength,100,minLineLength_Change)
cv2.createTrackbar("maxLineGap","[ProbHough] Prob_houghLines",maxLineGap,100,maxLineGap_Change)
cv2.createTrackbar("thresh","[ProbHough] Prob_houghLines",thresh,100,thresh_Change)


def Hough(img):

    # 1. Converting frame to HLS ColorSpace
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    
    # 2. Keeping only Edges of Segmented ROI    
    edges = cv2.Canny(gray,50,150,apertureSize = 3)

    # 3. Applying HoughLines to detect Lines in the image    
    lines = cv2.HoughLines(edges,1,np.pi/180,50)

    # 4. Draw detected lines on the frame    
    if lines is not None:    
        for rho,theta in lines[0]:
            a = np.cos(theta)
            b = np.sin(theta)
            x0 = a*rho
            y0 = b*rho
            x1 = int(x0 + 1000*(-b))
            y1 = int(y0 + 1000*(a))
            x2 = int(x0 - 1000*(-b))
            y2 = int(y0 - 1000*(a))
            
            # 4. Draw detected lines on the frame    
            cv2.line(img,(x1,y1),(x2,y2),(0,0,255),2)
        
        # 5. Display Detected Lines    
        cv2.imshow('[Hough] houghlines.jpg',img)
    return img


def ProbHough(img):
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray,50,200,apertureSize = 3)

    linesP = cv2.HoughLinesP(edges,1,np.pi/180,thresh,None,minLineLength,maxLineGap)
    #if lines is not None:    
        #for x1,y1,x2,y2 in lines[0]:
            #cv2.line(img,(x1,y1),(x2,y2),(0,255,0),2)
    if linesP is not None:
        for i in range(0, len(linesP)):
            l = linesP[i][0]
            cv2.line(img, (l[0], l[1]), (l[2], l[3]), (0,0,255), 2, cv2.LINE_AA)

    cv2.imshow("[ProbHough] Prob_houghLines",img)
