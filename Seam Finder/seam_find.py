'''
This files finds a seam between two images by minimizing a predefined cost function.
The cost of the seam defines the alignment quality of the two images.
'''
import cv2 as cv
import numpy as np
import maxflow
from matplotlib import pyplot as plt


#to find the coordinates for the intersection of the two images
#the offset is chosen for showing the seam between the stitched images.
def intersect(im1,off1,im2,off2):
	return tuple(np.abs(np.subtract(np.minimum(np.add(off1, im1), np.add(off2, im2)), np.maximum(off1, off2))))


#to find the size of the resultant overlapped image
def union(im1,off1,im2,off2):
	return tuple(np.abs(np.subtract(np.maximum(np.add(off1, im1), np.add(off2, im2)), np.minimum(off1, off2))))



#function to stitch the images - the last parameters is for showing the seam in the resultant image
def stitch_image(image, patch, offset, show_seam):
    row_overlap, col_overlap = intersect(image1.shape, (0, 0), image2.shape, offset)
    image1_overlap = image1[offset[0]:offset[0]+row_overlap, offset[1]:offset[1]+col_overlap]
    image2_overlap = image2[:row_overlap, :col_overlap]  #retrieving the overlapped regions in the two images

    #creating a graph of the overlapped regions
    #each pixel is represented as a node
    g = maxflow.Graph[float]()
    nodes = g.add_grid_nodes((row_overlap, col_overlap))

    # adding edges between the overlapped nodes and non-overlapping part of the images(source/sink,ie,terminal egdes)
    source, sink = (np.inf, 0), (0, np.inf)
    g.add_grid_tedges(nodes[1:, -1], *sink)     # top right 
    g.add_grid_tedges(nodes[-1,:-1], *sink)     # bottom right 
    g.add_grid_tedges(nodes[:-1, 0], *source)   # bottom left
    g.add_grid_tedges(nodes[0,  1:], *source)   # top left

    # edges in overlap, right and down respectively
    #for now cost is taken as the normalized difference between the intensity levels of the images
    cost = np.abs(image1_overlap.astype(float) - image2_overlap.astype(float)) / 256
    #weights are calculated as the sum of weights of adjacent pixels
    #structure defines the neighborhood of the pixel
    #symmetric = True implies edges in both directions.
    g.add_grid_edges(nodes, weights=(cost + np.roll(cost, -1, axis=1)), structure=np.array([[0, 0, 1]]), symmetric=True)
    #g.add_grid_edges(nodes, weights=(cost + np.roll(cost, -1, axis=0)), structure=np.array([[0, 0, 1]]).T, symmetric=True)
    g.add_grid_edges(nodes, weights=(cost + np.roll(cost, -1, axis=0)), structure=np.array([[0, 0, 1]]), symmetric=True)

    #calculate the cut set such that the cost is minimized.
    max_flow = g.maxflow()
    print('maxflow: {}'.format(max_flow))

    # min-cut would divide the graph into two parts
    # the segments to which the nodes of the overlapped region belong to are returned.
    segments = g.get_grid_segments(nodes)
    overlap = np.where(segments, image2_overlap, image1_overlap)
    #print(segments)
    #to show the seam adjacent pixels are XOR-ed and added so that pixels belonging to different segments 
    #have a seam between them.
    if show_seam:
        seam = np.logical_or.reduce([np.logical_xor(segments, np.roll(segments,  1, axis=0)),
                                    np.logical_xor(segments, np.roll(segments,  1, axis=1)),
                                    np.logical_xor(segments, np.roll(segments, -1, axis=0)),
                                    np.logical_xor(segments, np.roll(segments, -1, axis=1))])
        seam[:, :1] = seam[:1, :] = seam[:, -1:] = seam[-1:, :] = False
        overlap = np.where(seam, [[0]], overlap)

    # copy the old image, patch, overlap to a new one
    res_img = np.ndarray(union(image1.shape, (0, 0), image2.shape, offset))
    res_img[         :              image1.shape[0],          :              image1.shape[1]] = image1[:, :]
    res_img[offset[0]:offset[0] +   image2.shape[0], offset[1]:offset[1] +   image2.shape[1]] = image2[:, :]
    res_img[offset[0]:offset[0] + overlap.shape[0], offset[1]:offset[1] + overlap.shape[1]] = overlap[:, :]
    return res_img.astype(np.uint8) 

image1 = cv.imread('night1.png', 0)
#patch = np.copy(image)
image2 = cv.imread('night2.png', 0)
#image = stitch_patch(image, patch, (400, 100), 1)
res_image = stitch_image(image1, image2, (50, 700),1) # the offset value has been given manually for now.
#image = stitch_patch(image, patch, (200, 600), 1)
#image = stitch_patch(image, patch, (0, 250), 1)
#cv2.imwrite('Stiched_Image.png',image)

fig,ax = plt.subplots(1,3)
ax[0].imshow(image1,'gray')
ax[0].set_title('Image 1')
ax[0].axis('off')
ax[1].imshow(image2,'gray')
ax[1].axis('off')
ax[1].set_title('Image 2')
ax[2].imshow(res_image,'gray')
ax[2].axis('off')
ax[2].set_title('Stitched Image')
#ax.imshow(stitch_image,'gray')
plt.savefig('Stiched_Image.jpg')

#cv.imshow("Result", image)
#cv.waitKey(0)
