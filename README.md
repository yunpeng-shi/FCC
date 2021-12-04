## Scalable Cluster-Consistency Statistics for Robust Multi-Object Matching (3DV 2021 Oral Presentation)

This is a very useful algorithm for filtering out wrong keypoint matches using cycle-consistency constraints. It is fast, accurate and memory efficient. It is purely based on sparse matrix operations and is completely decentralized. As a result, it is scalable to large matching matrix (millions by millions, as those in large scale SfM datasets e.g. Photo Tourism). It uses a special reweighting scheme, which can be viewed as a message passing procedure, to refine the classification of good/bad keypoint matches. The filtering result is often better than Spectral and SDP based methods and can be several order of magnitude faster than previous cycle-consistency-based global optimization methods.

To use our code, please cite the following paper:
Yunpeng Shi, Shaohan Li, Tyler Maunu, Gilad Lerman. Filtering by Cluster Consistency for Multi-object Matching, International Conference on 3D Vision (3DV), 2021

## Usage

Checkout the demo code ``Demo_FCC.m``.

## Flexible Input and Informative Output

The function ``FCC.m`` takes matching matrix (Adjacency matrix of the keypoint matching graph, where the keypoints (nodes) are grouped by images) as input. In principle, the input can also be a SIFT feature (or other features) similarity matrix (so not necessarily binary). This function outputs the statistics matrix that tells you for each keypoint match its probability of being a good match. Thus, it contains the confidence information, not just classification results. One can set different threshold levels (tradeoff between precision and recall) for the statistics matrix to obtain the filtered matches, depending on the tasks. 

## A novel Synthetic Model

We provide a new synthetic model that realistically mirror the real scenario, and allowing control of different parameters. Please check ``FCC_synthetic_data.m``. It generates a set of synthetic cameras, images, 3d points and 2d keypoints. It allows user to control the sparsity in camera correspondences and keypoint matches, and the corruption level and corruption mode (elementwise or inlier-outlier model) for keypoint matches.

