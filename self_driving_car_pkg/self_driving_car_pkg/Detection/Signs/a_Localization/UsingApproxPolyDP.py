import cv2
import imutils

class ShapeDetector:
	def __init__(self):
		pass
	def detect(self, c):
		# initialize the shape name and approximate the contour
		shape = "unidentified"

        # 5. Calculate the contour perimeter to be used as an argument in approxPolyDP
		peri = cv2.arcLength(c, True)

        # 6. Perform approxPolyDP to approximate corners in the contour
		approx = cv2.approxPolyDP(c, 0.04 * peri, True)

		# . if the shape is a triangle, it will have 3 vertices
		if len(approx) == 2:
			shape = "line"

		# 7. if the shape is a triangle, it will have 3 vertices
		if len(approx) == 3:
			shape = "triangle"

		# 8. if the shape has 4 vertices, it is either a square or a rectangle
		elif len(approx) == 4:
			# compute the bounding box of the contour and use the bounding box to compute the aspect ratio
			(x, y, w, h) = cv2.boundingRect(approx)
			ar = w / float(h) # AspectRatio (square) = 1
			shape = "square" if ar >= 0.95 and ar <= 1.05 else "rectangle"
		
        # 9. if the shape is a pentagon, it will have 5 vertices
		elif len(approx) == 5:
			shape = "pentagon"

		# 10. otherwise, we assume the shape is a circle
		else:
			shape = "circle" 
        
        # return the name of the shape
		return shape


def LocalizeSigns(image):
    # load the image and resize it to a smaller factor so that
    # the shapes can be approximated better
    resized = imutils.resize(image, width=300)
    ratio = image.shape[0] / float(resized.shape[0])
    # convert the resized image to grayscale, blur it slightly,
    # and threshold it

    # 1. Cvt frame to grayscale
    gray = cv2.cvtColor(resized, cv2.COLOR_BGR2GRAY)

    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    #thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)[1]

    # 2. Perform CannySegmentation which will give a binary result
    thresh = cv2.Canny(blurred, 100, 200, 3)
    
    cv2.imshow("thresh", thresh)

    # 3. find contours in the thresholded image
    cnts = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    
    cnts = imutils.grab_contours(cnts)
    sd = ShapeDetector()

    # 4. loop over the contours
    for c in cnts:
        # compute the center of the contour, then detect the name of the
        # shape using only the contour
        M = cv2.moments(c)
        if(M["m00"]!=0):
            cX = int((M["m10"] / M["m00"]) * ratio)
            cY = int((M["m01"] / M["m00"]) * ratio)
            shape = sd.detect(c)
            if shape =="circle":
                # multiply the contour (x, y)-coordinates by the resize ratio,
                # then draw the contours and the name of the shape on the image
                c = c.astype("float")
                c *= ratio
                c = c.astype("int")
                cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
                cv2.putText(image, shape, (cX, cY), cv2.FONT_HERSHEY_SIMPLEX,0.5, (255, 255, 255), 2)
                # show the output image
                cv2.imshow("DetectedCircles[ApproxPolyDp]", image)
    
    return image
