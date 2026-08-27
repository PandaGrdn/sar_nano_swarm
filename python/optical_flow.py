"""
Step #1: Import the data -- and organize it such that timestamps align with images
Step #2: Gaussian Pyramid -- create a Gaussian pyramid for each image in the sequence: https://github.com/sunny1110/image_pyramid/blob/master/Gaussian%20and%20Laplacian%20Pyramids.ipynb
Step #3: Interframe Difference Pyr-LK algorithm -- determine differences between consecutive frames using the LK method to reduce storage and computation requriements
Step #4: Compute the optical flow between consecutive frames using the Lucas-Kanade method on the Gaussian pyramid

"""

import os
import numpy as np
import cv2
import pandas as pd
import matplotlib.pyplot as plt

base_path = '/Users/monika/Documents/cavers/data/rec_diablo_1/THERMAL'
image_path = '/Users/monika/Documents/cavers/data/rec_diablo_1/THERMAL/data'
# using gaussian kernel for gaussian blur function
gaussianKernel = np.array([[1, 2, 1], [2, 4, 2], [1, 2, 1]])
gaussianScale = 16.0

def track_and_visualize(image_prev, image_next, corners, lk_params=None,
                          title="Pyr-LK tracks", figsize=(8, 8)):
    """
    Run Pyr-LK optical flow from `image_prev` to `image_next` starting at
    `corners`, then draw the resulting tracks (line = motion path,
    circle = current position) and display with matplotlib.
 
    This is a fixed, single-shot version of the video-tutorial snippet:
    since image_prev/image_next are two static frames (not a live video
    stream), there's no `while(1)` loop, no `old_gray`/`p0` update step,
    and no cv2.imshow -- optical flow is computed once and the result is
    rendered as an image.
 
    image_prev, image_next : 2D grayscale arrays (any float/uint8 dtype),
                              same shape -- e.g. your pyramid-processed
                              image1 / image4
    corners                 : Nx1x2 (or Nx2) array from
                              cv2.goodFeaturesToTrack, found on image_prev
    lk_params               : optional dict overriding the default LK params
 
    Returns:
        good_new, good_old : Nx2 arrays of successfully tracked point
                              positions in image_next / image_prev
    """
    if lk_params is None:
        lk_params = dict(
            winSize=(15, 15),
            maxLevel=2,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03),
        )
 
    if corners is None or len(corners) == 0:
        print("No corners to track.")
        return np.empty((0, 2)), np.empty((0, 2))
 
    def to_uint8(img):
        img = np.asarray(img, dtype=np.float64)
        img = 255 * (img - img.min()) / (img.max() - img.min() + 1e-8)
        return img.astype(np.uint8)
 
    # calcOpticalFlowPyrLK requires 8-bit input images (unlike
    # goodFeaturesToTrack, which also accepts float32) -- see
    # OpenCV assertion: img.depth() == CV_8U in buildOpticalFlowPyramid
    prev8 = to_uint8(image_prev)
    next8 = to_uint8(image_next)
    corners = np.asarray(corners, dtype=np.float32).reshape(-1, 1, 2)
 
    p1, st, err = cv2.calcOpticalFlowPyrLK(prev8, next8, corners, None, **lk_params)
 
    if p1 is None:
        print("Tracking failed for all points.")
        return np.empty((0, 2)), np.empty((0, 2))
 
    st = st.reshape(-1)
    good_new = p1.reshape(-1, 2)[st == 1]
    good_old = corners.reshape(-1, 2)[st == 1]
 
    # build a uint8 3-channel canvas (from image_next) to draw color tracks on
    canvas = cv2.cvtColor(next8, cv2.COLOR_GRAY2BGR)
    tracks = np.zeros_like(canvas)  # separate layer for the motion lines
 
    n_points = max(len(good_new), 1)
    colors = np.random.randint(0, 255, (n_points, 3))
 
    for i, (new, old) in enumerate(zip(good_new, good_old)):
        a, b = new.ravel()
        c, d = old.ravel()
        a, b, c, d = int(round(a)), int(round(b)), int(round(c)), int(round(d))
        color = colors[i].tolist()
        tracks = cv2.line(tracks, (a, b), (c, d), color, 2)
        canvas = cv2.circle(canvas, (a, b), 5, color, -1)
 
    img = cv2.add(canvas, tracks)
 
    fig, ax = plt.subplots(figsize=figsize)
    ax.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    ax.set_title(f"{title} ({len(good_new)} points tracked)")
    ax.axis("off")
    plt.tight_layout()
    plt.show()
 
    return good_new, good_old

def load_data(folder):
    # get associated timestamps from csv file
    times = pd.read_csv(os.path.join(folder, 'data.csv'))
    print(times)
    return times['Timestamp'].values, times['Image_Name'].values
    
def get_timestamps(times, idx):
    return times[idx]
    
def read_image(folder, filename):
    return cv2.imread(os.path.join(folder, filename))

# grayscale the image
def rgb2gray(image):
    r, g, b = image[:, :, 0], image[:, :, 1], image[:, :, 2]
    gray = 0.299 * r + 0.587 * g + 0.114 * b
    return gray

def convert2GrayScale(image, alpha=1.3, beta=0):
    image = cv2.convertScaleAbs(image, alpha=alpha, beta=beta)
    if(len(image.shape)>2):
        print("Converting to grayscale")
        return rgb2gray(image).astype(np.float32)
    else:
        print("Image is already grayscale")
        return image.astype(np.float32)

def convolve(image, imageFilter, scaleValue):
    image = np.asarray(image, dtype=np.float64)
    imageFilter = np.asarray(imageFilter, dtype=np.float64)
    blurred = cv2.filter2D(image, ddepth=-1, kernel=imageFilter,
                            borderType=cv2.BORDER_REPLICATE)
    return blurred / scaleValue

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

def constructPyramids(image, imageLabel, folderName, N=1):
    levelImage = image
    gaussianPath = folderName+"/gaussian_pyramid/"+imageLabel+"_gaussian_level_"
    returningImage = levelImage
    for i in range(N):
        blurredLevelImage = gaussianBlur(levelImage)
        scaledDownLevelImage = scaleDownImage(blurredLevelImage)
        cv2.imwrite(gaussianPath+str(i)+".jpg", scaledDownLevelImage)
        scaledUpLevelImage = scaleUpImage(scaledDownLevelImage)
        returningImage = scaledDownLevelImage
        differenceImage = imageDifference(levelImage, scaledUpLevelImage)
        cv2.imwrite(gaussianPath+str(i)+"_difference.jpg", differenceImage)
        levelImage = scaledDownLevelImage
    return returningImage

def display_rk(Rk, img_final=None, image_bg=None, title="Change-detection mask Rk", figsize=(10, 5)):
    """
    Display the Rk mask. If `image_bg` (e.g. one of the original frames) is
    given, shows it side-by-side with the mask overlaid in red so you can
    see where the detections landed in context.
 
    Rk       : 2D array of 0/1 values (as returned by compute_rk)
    image_bg : optional 2D grayscale image, same shape as Rk, for context
    """
    Rk = np.asarray(Rk)
 
    if image_bg is None:
        fig, ax = plt.subplots(figsize=(figsize[0] / 2, figsize[1]))
        ax.imshow(Rk, cmap="gray", vmin=0, vmax=1)
        ax.set_title(title)
        ax.axis("off")
    else:
        image_bg = np.asarray(image_bg, dtype=np.float64)
        img_final = np.asarray(img_final, dtype=np.float64)
        fig, axes = plt.subplots(2, 2, figsize=figsize)
 
        axes[0][0].imshow(image_bg, cmap="gray")
        axes[0][0].set_title("Reference image")
        axes[0][0].axis("off")
        
        axes[1][0].imshow(img_final, cmap="gray")
        axes[1][0].set_title("Reference image")
        axes[1][0].axis("off")
 
        overlay = np.stack(
            [image_bg, image_bg, image_bg], axis=-1
        )
        overlay = 255 * (overlay - overlay.min()) / (overlay.max() - overlay.min() + 1e-8)
        overlay = overlay.astype(np.uint8)
        overlay[Rk == 1] = [255, 0, 0]  # highlight detected pixels in red
 
        axes[0][1].imshow(overlay)
        axes[0][1].set_title(title)
        axes[0][1].axis("off")
 
    plt.tight_layout()
    plt.show()
    return fig


 
def display_corners(image, corners, title="Detected corners", figsize=(6, 6),
                     marker_color="yellow", marker_size=40):
    """
    Display `corners` (as returned by cv2.goodFeaturesToTrack) overlaid on
    `image`.
 
    image   : 2D grayscale array (e.g. grayImage1)
    corners : output of cv2.goodFeaturesToTrack -- shape (N, 1, 2), or
              already-reshaped (N, 2). Can be None (no corners found).
    """
    image = np.asarray(image)
 
    fig, ax = plt.subplots(figsize=figsize)
    ax.imshow(image, cmap="gray")
 
    if corners is not None:
        pts = np.asarray(corners).reshape(-1, 2)
        ax.scatter(pts[:, 0], pts[:, 1], s=marker_size,
                   edgecolors=marker_color, facecolors="none", linewidths=1.5)
        n_found = len(pts)
    else:
        n_found = 0
 
    ax.set_title(f"{title} ({n_found} found)")
    ax.axis("off")
    plt.tight_layout()
    plt.show()
    return fig

def clean_mask_to_J(Rk, open_ksize=3, close_ksize=7):
    """
    Morphological open (erode->dilate) removes small speckle noise;
    morphological close (dilate->erode) fills small gaps inside a real
    target blob. What remains is J(x,y), the approximate detection region.
    """
    mask = (Rk.astype(np.uint8)) * 255
 
    open_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (open_ksize, open_ksize))
    close_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (close_ksize, close_ksize))
 
    opened = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel)
    closed = cv2.morphologyEx(opened, cv2.MORPH_CLOSE, close_kernel)
 
    J = (closed > 0).astype(np.uint8)
    return J
 
def calc_vels(new_pts, old_pts):
    # returns x vel, y vel in pixels per frames
    vels_x = np.zeros(len(new_pts))
    vels_y = np.zeros(len(new_pts))
    for i, (new, old) in enumerate(zip(new_pts, old_pts)):
        x, y = old.ravel()
        x1, y1 = new.ravel()
        vels_x[i] = (x1-x)/4
        vels_y[i] = (y1-y)/4
        # print(f"Point {i}:")
        # print(vels[i].ravel())
    return vels_x, vels_y
        

timestamps, image_names = load_data(base_path)
print(image_names[0])

# interframe difference method: select 4 consecutive frams to compare between

threshold_s = 0.001
L = 2

for i in range(43, 47, 4):
    image1 = read_image(image_path, image_names[i])
    image2 = read_image(image_path, image_names[i+1])
    image3 = read_image(image_path, image_names[i+2])
    image4 = read_image(image_path, image_names[i+3])
    
    grayImage1 = convert2GrayScale(image1)
    grayImage2 = convert2GrayScale(image2)
    grayImage3 = convert2GrayScale(image3)
    grayImage4 = convert2GrayScale(image4)
    
    image1 = constructPyramids(grayImage1, image_names[i].split('.')[0], image_path)
    image2 = constructPyramids(grayImage2, image_names[i+1].split('.')[0], image_path)
    image3 = constructPyramids(grayImage3, image_names[i+2].split('.')[0], image_path)
    image4 = constructPyramids(grayImage4, image_names[i+3].split('.')[0], image_path)
    
    # calculate the dynamic threshold portion (ds1, ds2)
    ds1, ds2 = 0, 0
    W = image1.shape[0]
    H = image1.shape[1]
    
    weight = L/(W * H)
    for x in range(W):
        for y in range(H):
            ds1 += abs(image1[x][y] - image2[x][y])
            ds2 += abs(image3[x][y] - image4[x][y])
    
    ds1 *= weight
    ds2 *= weight
    
    # fill Rk
    r1 = -1
    r2 = -1
    Rk = np.zeros((W,H))
    ind = 0
    
    for x in range(W):
        for y in range(H):
            diff_1 = abs(image1[x][y] - image2[x][y])
            diff_2 = abs(image3[x][y] - image4[x][y])
            
            r1 = 0 if diff_1 < (threshold_s + ds1) else 1
            r2 = 0 if diff_2 < (threshold_s + ds2) else 1
            
            Rk[x][y] = 1 if(r1 == 1 and r2 == 1) else 0
    
    J = clean_mask_to_J(Rk)
    print("here")
    print(np.count_nonzero(Rk == 1))
    display_rk(J, image1, image4)
    
    # get corners
    mask_u8 = J.astype(np.uint8) * 255
    print(J)
    gray_img = convert2GrayScale(image1)
    mask_u8 = J.astype(np.uint8) * 255
    corners = cv2.goodFeaturesToTrack(
        image1.astype(np.float32),   # image1, not grayImage1 -- matches J's resolution
        maxCorners=10, qualityLevel=0.01, minDistance=20, mask=mask_u8
    )
    display_corners(image1, corners)   # display on the same image you searched, for consistent coordinates
    
    # tracking the corner points
    
    new_points, old_points = track_and_visualize(image1.astype(np.float32), image4.astype(np.float32), corners, lk_params=None,
                          title="Pyr-LK tracks", figsize=(8, 8))
    
    x_vel, y_vel = calc_vels(new_points, old_points)
    
    
    # need to implement culstering if there is a lot of noise in the data (also needed if including more points)
    of_x_vel = x_vel.mean()
    of_y_vel = y_vel.mean()
    
    print(of_x_vel, of_y_vel)
    # should combine with the imu data to covert to actual vel approximations