"""
Foreground detection (four-frame difference, Rk) -> approximate target
region J(x,y) -> Pyramidal Lucas-Kanade optical flow on that region.

Pipeline:
  1. Denoise the 4 input frames (Gaussian blur).
  2. Four-interframe difference -> Rk mask (Rk==1 -> foreground/moving,
     Rk==0 -> background), using the dynamic threshold (threshold_s + ds).
  3. Morphological open+close on Rk to remove speckle noise and fill small
     holes -> this cleaned-up mask *is* J(x,y), the approximate region(s)
     where the moving target is.
  4. Extract good-to-track feature points restricted to J(x,y), and run
     Pyramidal LK optical flow on just those points between two frames
     (e.g. frame i and frame i+1, or i+2 and i+3 -- whichever consecutive
     pair you're tracking motion across).
"""

import numpy as np
import cv2
import matplotlib.pyplot as plt

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

def convert2GrayScale(image):
    if(len(image.shape)>2):
        print("Converting to grayscale")
        return rgb2gray(image)
    else:
        print("Image is already grayscale")
        return image

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

def constructPyramids(image, imageLabel, folderName, N=3):
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
 
        

timestamps, image_names = load_data(base_path)
print(image_names[0])

# interframe difference method: select 4 consecutive frams to compare between

threshold_s = 0.001
L = 0.002

# ----------------------------------------------------------------------
# Step 1-2: four-frame difference -> Rk
# ----------------------------------------------------------------------

def denoise(image, ksize=(5, 5), sigma=1.0):
    """Simple Gaussian denoising step (replace with your own pyramid blur
    if you'd rather reuse `gaussianBlur`/`convolve` from your script)."""
    return cv2.GaussianBlur(image.astype(np.float32), ksize, sigma)


def compute_rk(image1, image2, image3, image4, threshold_s, L=0.01):
    """
    Four-interframe difference with a dynamic threshold:

        ds1 = weight * sum(|image1 - image2|),  weight = L / (W*H)
        ds2 = weight * sum(|image3 - image4|)
        r1  = 1 if |image1 - image2| >= threshold_s + ds1 else 0
        r2  = 1 if |image3 - image4| >= threshold_s + ds2 else 0
        Rk  = 1 if (r1 == 1 and r2 == 1) else 0

    Note: ds1/ds2 use `weight = L/(W*H)`, i.e. `ds = L * mean(diff)`, NOT
    `L * sum(diff)`. Using the raw sum (as your original script did --
    it computed `weight` but never actually applied it) makes ds1/ds2
    scale with image resolution and blow up to values far larger than
    any real per-pixel intensity difference, which makes Rk always 0.
    Using the mean keeps the dynamic threshold in the same units/scale
    as the per-pixel differences it's being compared against.

    Vectorized (no pixel-by-pixel Python loop).
    """
    image1 = np.asarray(image1, dtype=np.float64)
    image2 = np.asarray(image2, dtype=np.float64)
    image3 = np.asarray(image3, dtype=np.float64)
    image4 = np.asarray(image4, dtype=np.float64)

    diff_1 = np.abs(image1 - image2)
    diff_2 = np.abs(image3 - image4)

    W, H = image1.shape
    weight = L / (W * H)

    ds1 = weight * diff_1.sum()
    ds2 = weight * diff_2.sum()

    r1 = diff_1 >= (threshold_s + ds1)
    r2 = diff_2 >= (threshold_s + ds2)

    Rk = (r1 & r2).astype(np.uint8)
    return Rk


# ----------------------------------------------------------------------
# Step 3: clean Rk up into J(x,y), the approximate target region(s)
# ----------------------------------------------------------------------

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


def get_target_bounding_boxes(J, min_area=20):
    """Optional: connected components of J -> bounding boxes of each
    candidate moving target, useful for visualization/logging."""
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(
        J.astype(np.uint8), connectivity=8
    )
    boxes = []
    for label in range(1, num_labels):  # skip background label 0
        x, y, w, h, area = stats[label]
        if area >= min_area:
            boxes.append((x, y, w, h, area))
    return boxes


# ----------------------------------------------------------------------
# Step 4: Pyr-LK optical flow restricted to J(x,y)
# ----------------------------------------------------------------------

def track_region_with_pyrlk(frame_prev, frame_next, J, max_corners=200,
                             quality_level=0.01, min_distance=7,
                             win_size=(15, 15), max_level=3):
    """
    Find good feature points inside the mask J (the approximate target
    region) in frame_prev, then track them into frame_next using
    OpenCV's pyramidal Lucas-Kanade (cv2.calcOpticalFlowPyrLK).

    Returns:
        pts_prev : Nx2 array, tracked points in frame_prev
        pts_next : Nx2 array, corresponding tracked points in frame_next
        status   : Nx1 array, 1 if the point was tracked successfully
    """
    prev_gray = frame_prev.astype(np.uint8) if frame_prev.dtype != np.uint8 else frame_prev
    next_gray = frame_next.astype(np.uint8) if frame_next.dtype != np.uint8 else frame_next

    mask_u8 = (J > 0).astype(np.uint8) * 255

    pts_prev = cv2.goodFeaturesToTrack(
        prev_gray, maxCorners=max_corners, qualityLevel=quality_level,
        minDistance=min_distance, mask=mask_u8
    )

    if pts_prev is None or len(pts_prev) == 0:
        return np.empty((0, 2)), np.empty((0, 2)), np.empty((0,), dtype=np.uint8)

    lk_params = dict(winSize=win_size, maxLevel=max_level,
                      criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.03))

    pts_next, status, err = cv2.calcOpticalFlowPyrLK(
        prev_gray, next_gray, pts_prev, None, **lk_params
    )

    status = status.reshape(-1)
    pts_prev = pts_prev.reshape(-1, 2)
    pts_next = pts_next.reshape(-1, 2)

    return pts_prev[status == 1], pts_next[status == 1], status[status == 1]


# ----------------------------------------------------------------------
# End-to-end orchestration + visualization
# ----------------------------------------------------------------------

def detect_and_track(image1, image2, image3, image4, threshold_s=10.0, L=0.01,
                      track_between=(0, 1)):
    """
    Full pipeline:
      1. denoise all 4 frames
      2. Rk = four-frame difference (foreground/background)
      3. J  = cleaned-up Rk -> approximate target region
      4. track feature points inside J using Pyr-LK, between the frame
         pair specified by `track_between` (indices into
         [image1, image2, image3, image4])

    Returns dict with Rk, J, bounding boxes, and tracked point pairs.
    """
    frames = [denoise(image1), denoise(image2), denoise(image3), denoise(image4)]

    Rk = compute_rk(*frames, threshold_s=threshold_s, L=L)
    J = clean_mask_to_J(Rk)
    boxes = get_target_bounding_boxes(J)

    i, j = track_between
    pts_prev, pts_next, status = track_region_with_pyrlk(frames[i], frames[j], J)

    return {
        "Rk": Rk,
        "J": J,
        "boxes": boxes,
        "pts_prev": pts_prev,
        "pts_next": pts_next,
        "status": status,
    }


def display_detection_and_flow(frame_bg, J, pts_prev, pts_next, boxes=None,
                                 title="Detected target region + optical flow"):
    """Visualize: reference frame with J region overlaid in red, target
    bounding boxes drawn, and optical flow vectors (prev->next) as arrows."""
    frame_bg = np.asarray(frame_bg, dtype=np.float64)
    vis = np.stack([frame_bg] * 3, axis=-1)
    vis = 255 * (vis - vis.min()) / (vis.max() - vis.min() + 1e-8)
    vis = vis.astype(np.uint8).copy()

    overlay = vis.copy()
    overlay[J == 1] = [255, 0, 0]
    vis = cv2.addWeighted(overlay, 0.35, vis, 0.65, 0)

    if boxes:
        for (x, y, w, h, area) in boxes:
            cv2.rectangle(vis, (x, y), (x + w, y + h), (0, 255, 0), 2)

    fig, ax = plt.subplots(figsize=(8, 6))
    ax.imshow(vis)
    if len(pts_prev) > 0:
        ax.quiver(pts_prev[:, 0], pts_prev[:, 1],
                   pts_next[:, 0] - pts_prev[:, 0], pts_next[:, 1] - pts_prev[:, 1],
                   angles='xy', scale_units='xy', scale=1, color='yellow', width=0.003)
    ax.set_title(title)
    ax.axis("off")
    plt.tight_layout()
    plt.show()
    return fig


# ----------------------------------------------------------------------
# Demo with synthetic frames
# ----------------------------------------------------------------------

def _textured_patch(shape, seed, n_blobs=300, amp_range=(20, 80), r_range=(2, 5)):
    """Small random Gaussian blobs -> enough local texture/gradient for LK
    to lock onto (a smooth, featureless blob is a hard/degenerate case)."""
    rng = np.random.default_rng(seed)
    img = np.zeros(shape, dtype=np.float64)
    for _ in range(n_blobs):
        cx, cy = rng.uniform(0, shape[1]), rng.uniform(0, shape[0])
        r = rng.uniform(*r_range)
        amp = rng.uniform(*amp_range)
        ys, xs = np.meshgrid(np.arange(shape[0]), np.arange(shape[1]), indexing="ij")
        img += amp * np.exp(-(((xs - cx) ** 2 + (ys - cy) ** 2) / (2 * r ** 2)))
    return img


def _demo():
    shape = (200, 260)
    background = _textured_patch(shape, seed=5)
    template = _textured_patch((40, 40), seed=99, n_blobs=60) * 1.5
    timestamps, image_names = load_data(base_path)
    i = 0

    def add_target(bg, dx, y0=80, x0=60):
        out = bg.copy()
        out[y0:y0 + 40, x0 + dx:x0 + dx + 40] += template
        return out

    # # simulate a small textured target moving 5px/frame to the right
    # image1 = add_target(background, 0)
    # image2 = add_target(background, 5)
    # image3 = add_target(background, 10)
    # image4 = add_target(background, 15)
    
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

    result = detect_and_track(image1, image2, image3, image4,
                               threshold_s=0.5, L=0.001, track_between=(0, 1))

    print(f"Rk flagged pixels: {result['Rk'].sum()}")
    print(f"J (cleaned) flagged pixels: {result['J'].sum()}")
    print(f"Bounding boxes found: {result['boxes']}")
    print(f"Points tracked: {len(result['pts_prev'])}")
    if len(result['pts_prev']) > 0:
        flow = result['pts_next'] - result['pts_prev']
        print(f"Mean flow vector (ground truth dx=5, dy=0): {flow.mean(axis=0)}, "
              f"std: {flow.std(axis=0)}")

    display_detection_and_flow(image1, result['J'], result['pts_prev'],
                                result['pts_next'], result['boxes'])


if __name__ == "__main__":
    _demo()