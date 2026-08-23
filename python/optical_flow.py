"""
Step #1: Import the data -- and organize it such that timestamps align with images
Step #2: Gaussian Pyramid -- create a Gaussian pyramid for each image in the sequence: https://github.com/sunny1110/image_pyramid/blob/master/Gaussian%20and%20Laplacian%20Pyramids.ipynb
Step #3: Interframe Difference Pyr-LK algorithm -- compute the optical flow between consecutive frames using the Lucas-Kanade method on the Gaussian pyramid

"""

import os
import numpy as np
import cv2
import pandas as pd

base_path = '/Users/monika/Documents/cavers/data/rec_diablo_1/THERMAL'
# using gaussian kernel for gaussian blur function
gaussianKernel = np.array([[1, 2, 1], [2, 4, 2], [1, 2, 1]])
gaussianScale = 16.0

def load_timestamps(folder):
    # get associated timestamps from csv file
    times = pd.read_csv(os.path.join(folder, 'data.csv'))
    return times['Timestamp'].values    
    
def get_timestamps(times, idx):
    return times[idx]
    
def read_image(folder, filename):
    return cv2.imread(os.path.join(folder, filename))

# grayscale the image
def rgb2gray(image):
    r, g, b = image[:, :, 0], image[:, :, 1], image[:, :, 2]
    gray = 0.299 * r + 0.587 * g + 0.114 * b
    return gray

def convert2GrayScale(image):
    if(len(image.shape)>2):
        print("Converting to grayscale")
        return rgb2gray(image)
    else:
        print("Image is already grayscale")
        return image

def convolve(image, imageFilter, scaleValue):
    blurredList = []
    for i in range(image.shape[0]):
        for j in range(image.shape[1]):
            filterPixels = imageFilter[max(0, 1-i): imageFilter.shape[0]+min(0, image.shape[0]-i), max(0, 1-j): imageFilter.shape[1]+min(0, image.shape[1]-j)]
            startX = ((j+max(0, 1-j))-1)
            startY = ((i+max(0, 1-i))-1)
            imagePixels = image[startY:startY+filterPixels.shape[0], startX:startX+filterPixels.shape[1]]
            flatImage = np.array(imagePixels).flatten()
            flatFilter = np.array(filterPixels).flatten()
            blurredPixel = (np.dot(flatImage, flatFilter))/scaleValue
            blurredList.append(blurredPixel)
    
    return np.array(blurredList).reshape(image.shape)

def gaussianBlur(image):
    return convolve(image, gaussianKernel, gaussianScale)

def scaleDownImage(image):
    return image[1:image.shape[0]:2, 1:image.shape[1]:2]

def scaleUp(image):
    return np.insert(np.insert(image, np.arange(1, image.shape[0]+1), 0, axis=0), np.arange(1, image.shape[1]+1), 0, axis=1)

def scaleUpImage(image):
    scaledImage = scaleUp(image)
    ScaledUpImage = gaussianBlur(scaledImage)*4
    return ScaledUpImage

def imageDifference(image1, image2):
    return np.subtract(image1[0:image2.shape[0], :image2.shape[1]], image2)

def constructPyramids(image, imageLabel, folderName, N=5):
    levelImage = image
    gaussianPath = folderName+"/gaussian_pyramid/"+imageLabel+"_gaussian_level_"
    for i in range(N):
        blurredLevelImage = gaussianBlur(levelImage)
        scaledDownLevelImage = scaleDownImage(blurredLevelImage)
        cv2.imwrite(gaussianPath+str(i)+".jpg", scaledDownLevelImage)
        scaledUpLevelImage = scaleUpImage(scaledDownLevelImage)
        differenceImage = imageDifference(levelImage, scaledUpLevelImage)
        cv2.imwrite(gaussianPath+str(i)+"_difference.jpg", differenceImage)
        levelImage = scaledDownLevelImage

timestamps = load_timestamps(base_path)

