import numpy as np
import imutils
import cv2

class Stitcher:

    def stitch(self, images, ratio=0.75, reprojThresh=4.0,
        showMatches=False):
        # unpack the images, then detect keypoints and extract
        # local invariant descriptors from them
        (img2, img1) = images
        (kpsA, featuresA) = self.detectAndDescribe(img1)
        (kpsB, featuresB) = self.detectAndDescribe(img2)

        # match features between the two images
        M = self.matchKeypoints(kpsA, kpsB,
            featuresA, featuresB, ratio, reprojThresh)

        # if the match is None, then there aren't enough matched
        # keypoints to create a panorama
        if M is None:
            return None

        (matches, H, status) = M
        print (H)
        result = cv2.warpPerspective(img1, H,
            (img1.shape[1] + img2.shape[1], img1.shape[0]))
        result[0:img2.shape[0], 0:img2.shape[1]] = img2

        # check to see if the keypoint matches should be visualized
        if showMatches:
            vis = self.drawMatches(img1, img2, kpsA, kpsB, matches,
                status)

            # return a tuple of the stitched image and the
            # visualization
            return (result, vis)

        # return the stitched image
        return result


    def detectAndDescribe(self, image):
        # convert the image to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # check to see if we are using OpenCV 3.X
            # detect and extract features from the image
        descriptor = cv2.xfeatures2d.SIFT_create()
        (kps, features) = descriptor.detectAndCompute(image, None)


        # convert the keypoints from KeyPoint objects to NumPy
        # arrays
        kps = np.float32([kp.pt for kp in kps])

        # return a tuple of keypoints and features
        return (kps, features)

    def matchKeypoints(self, kpsA, kpsB, featuresA, featuresB,
        ratio, reprojThresh):
        # compute the raw matches and initialize the list of actual
        # matches
        matcher = cv2.DescriptorMatcher_create("BruteForce")
        rawMatches = matcher.knnMatch(featuresA, featuresB, 2)
        matches = []

        # loop over the raw matches
        for m in rawMatches:
            # ensure the distance is within a certain ratio of each
            # other (i.e. Lowe's ratio test)
            if len(m) == 2 and m[0].distance < m[1].distance * ratio:
                matches.append((m[0].trainIdx, m[0].queryIdx))

        # computing a homography requires at least 4 matches
        if len(matches) > 4:
            # construct the two sets of points
            ptsA = np.float32([kpsA[i] for (_, i) in matches])
            ptsB = np.float32([kpsB[i] for (i, _) in matches])

            # compute the homography between the two sets of points
            (H, status) = cv2.findHomography(ptsA, ptsB, cv2.RANSAC,
                reprojThresh)

            # return the matches along with the homograpy matrix
            # and status of each matched point
            return (matches, H, status)

        # otherwise, no homograpy could be computed
        return None

    def drawMatches(self, img1, img2, kpsA, kpsB, matches, status):
        # initialize the output visualization image
        (hA, wA) = img1.shape[:2]
        (hB, wB) = img2.shape[:2]
        vis = np.zeros((max(hA, hB), wA + wB, 3), dtype="uint8")
        vis[0:hA, 0:wA] = img1
        vis[0:hB, wA:] = img2

        # loop over the matches
        for ((trainIdx, queryIdx), s) in zip(matches, status):
            # only process the match if the keypoint was successfully
            # matched
            if s == 1:
                # draw the match
                ptA = (int(kpsA[queryIdx][0]), int(kpsA[queryIdx][1]))
                ptB = (int(kpsB[trainIdx][0]) + wA, int(kpsB[trainIdx][1]))
                cv2.line(vis, ptA, ptB, (0, 255, 0), 1)

        # return the visualization
        return vis



import argparse
import imutils
import cv2
import matplotlib.pyplot as plt
# construct the argument parse and parse the arguments
# ap = argparse.ArgumentParser()
# ap.add_argument("-f", "--first", required=True,
#     help="path to the first image")
# ap.add_argument("-s", "--second", required=True,
#     help="path to the second image")
# args = vars(ap.parse_args())
# load the two images and resize them to have a width of 400 pixels
# (for faster processing)
img1 = cv2.imread('./053.jpg')
img2 = cv2.imread('./054.jpg')
# img1 = imutils.resize(img1, width=400)
# img2 = imutils.resize(img2, width=400)
# img1 = cv2.Canny(img1,100,200)
# img2 = cv2.Canny(img2,100,200)

# stitch the images together to create a panorama
stitcher = Stitcher()
(result, vis) = stitcher.stitch([img1, img2], showMatches=True)

# show the images

plt.imshow(result)
plt.show()