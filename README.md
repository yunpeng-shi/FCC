## Scalable Cluster-Consistency Statistics for Robust Multi-Object Matching (3DV 2021 Oral Presentation)

FCC is a very useful algorithm for filtering out wrong keypoint matches using cycle-consistency constraints. It is fast, accurate and memory efficient. It is purely based on sparse matrix operations and is completely decentralized. As a result, it is scalable to large matching matrix (millions by millions, as those in large scale SfM datasets e.g. Photo Tourism). It uses a special reweighting scheme, which can be viewed as a message passing procedure, to refine the classification of good/bad keypoint matches. The filtering result is often better than Spectral and SDP based methods and can be several order of magnitude faster.

To use our code, please cite the following paper:
Yunpeng Shi, Shaohan Li, Tyler Maunu, Gilad Lerman. Scalable Cluster-Consistency Statistics for Robust Multi-Object Matching, International Conference on 3D Vision (3DV), 2021

## Usage

Checkout the demo code ``Demo_FCC.m``. A sample output is as follows:

```
>> Demo_FCC
generate initial camera adjacency matrix
creat camera intrinsic matrices. f (focal length) is set to 5000 pixel sizes
generate 3d point cloud (a sphere)
generate camera locations from 3d gaussian dist with radius constraints
generating 2d keypoints from camera projection matrices
generating and corrupting keypoint matches
start running FCC
iteration 1 Completed!
iteration 2 Completed!
iteration 3 Completed!
iteration 4 Completed!
iteration 5 Completed!
iteration 6 Completed!
iteration 7 Completed!
iteration 8 Completed!
iteration 9 Completed!
iteration 10 Completed!
Elapsed time is 0.782890 seconds.
classification error (Jaccard distance) = 0.031733
precision rate = 0.973654
recall rate = 0.994319

```

![histogram](https://github.com/yunpeng-shi/FCC/blob/main/hist.jpg)

## Flexible Input and Informative Output

The function ``FCC.m`` takes matching matrix (Adjacency matrix of the keypoint matching graph, where the indices of keypoints (nodes) are grouped by images) as input. In principle, the input can also be a SIFT feature (or other features) similarity matrix (so not necessarily binary). This function outputs the statistics matrix that tells you for each keypoint match its probability of being a good match. Thus, it contains the confidence information, not just classification results. One can set different threshold levels (tradeoff between precision and recall) for the statistics matrix to obtain the filtered matches, depending on the tasks. 

## A novel Synthetic Model

We provide a new synthetic model that realistically mirror the real scenario, and allows control of different parameters. Please check ``FCC_synthetic_data.m``. It generates a set of synthetic cameras, images, 3d points and 2d keypoints. It allows user to control the sparsity in camera correspondences and keypoint matches, and the corruption level and corruption mode (elementwise or inlier-outlier model) for keypoint matches.

