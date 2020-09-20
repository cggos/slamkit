import apriltag
import cv2

img = cv2.imread('apriltags.png', cv2.IMREAD_GRAYSCALE) 
options = apriltag.DetectorOptions(families='tag36h11',
                                 border=1,
                                 nthreads=4,
                                 quad_decimate=1.0,
                                 quad_blur=0.0,
                                 refine_edges=True,
                                 refine_decode=False,
                                 refine_pose=False,
                                 debug=False,
                                 quad_contours=True)
detector = apriltag.Detector(options)

result = detector.detect(img)

print len(result)

tf = result[0].tag_family
center = result[0].center

print center