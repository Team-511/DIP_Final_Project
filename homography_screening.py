## list of homographyies obtained
import numpy as np
import matplotlib.pyplot as plt

class hs(list1,img):
    self.list1 = list1
    self.img = img
    def list_to_matrix():
        print(self.list1)
        n = np.matrix(self.list1[0][0])
        # print (n)
    def C_and_Ch(img,M):
        width,height = img.shape[1],img.shape[0]
        C = [[0,0],[0,width],[height,0],[height,width]]
        warped_img = cv2.warpPerspective(img, M)
        Ch = [[0,0],[0,width],[height,0],[height,width]]
        sum = 0
        for item in range(C):
            sum += (Ch[item] - C[item])**2
        retrun sum
    def argmin(img):
        min = 1*np.exp(7)
        i = 0
        for value in list1:
            M  = list_to_matrix(value)
            sq_sum.append(C_and_Ch(img,M))
            if (sq_sum[i]<min):
                min = sq_sum[i]
                store = i

## read list of homographies
img = cv2.imread('./example1.jpg')
hs = hs(list1,img)
M = list_to_matrix()
sum = hs.C_and_Ch(img,M)


