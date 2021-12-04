## Filtering by Cluster Consistency for Multi-object Matching (3DV 2021 Oral Presentation)

This is a very useful algorithm for filtering out wrong keypoint matches using cycle-consistency constraints. It is fast, accurate and memory efficient. It is purely based on sparse matrix operations and is completely decentralized. As a result, it is scalable to large matching matrix (millions by millions, as those in large scale SfM datasets e.g. Photo Tourism). It uses a special reweighting scheme, which can be viewed as a message passing procedure, to refine the classification of good/bad keypoint matches. The filtering result is often better than Spectral and SDP based methods and can be several order of magnitude faster than previous cycle-consistency-based global optimization methods.

To use our code, please cite the following paper:
Yunpeng Shi, Shaohan Li, Tyler Maunu, Gilad Lerman. Filtering by Cluster Consistency for Multi-object Matching, Internation Conference on 3D Vision (3DV), 2021

## Usage

Checkout the demo code ``Demo_FCC.m``.
