rng(1);

% parameters for generating our novel synthetic data that mimics real data
n=100; % number of cameras
n_pt=100; % number of 3D points
p=0.5; % probability for connecting two cameras
q0 = 0.5; % probability for removing a keypoint match
q1 = 0.5; % probability of adding a wrong match
q2 = 0; % probability for completely corrupting keypoint matches between two images

% obtaining the observed keypoint matches and ground truth ones
% mat_size is a vector, whose i-th element is the number of keypoints in
% i-th image
% (i,j)-th block of XMat (denoted as Xij) is the matching matrix encoding the keypoint
% matches between i-th and j-th images. For example, Xij(k,l)=1 if and only
% if the k-th keypoint in image i is matched to the l-th keypoint in image
% j
[XMat, XMat_gt, mat_size] = FCC_synthetic_data(n,n_pt,p,q0,q1,q2);
        
X_g = XMat_gt.*XMat; % good matches within observed ones, unknown, only for evaluation
X_b = XMat - XMat_gt.*XMat; % bad matches within observed ones, unknown, only for evaluation
%%%% FCC

% parameters for running FCC
n_iter=10; % number of iterations
path_length = 2; % count paths that connects keypoints i,k in 2 steps, and j,k in 2 steps (longer paths are not needed as we use message-pssing)
n_batch=8; % number of batches (use more number of batches if your computer has small memory)
rounding = 0; % default with no additional thresholding

% FCC takes corrupted keypoint matching matrix X, and outputs a statistics
% matrix S (supported on the nonzero elements of X). S(i,j) is between 0 and 1, and is interpreted as the
% confidence/probability that X(i,j) is a good keypoint match.
tic
S = FCC(XMat, mat_size, path_length, n_iter, n_batch, rounding);
X_est = S>0.5; % obtain the refined keypoint match by selecting matches in X with high S-values. 
               % Choose threshold as 0.9 or 0.99 if precision is a lot more important than recall.
toc

% Compute Jaccard distance as an error metric:
Xcap = X_g.*X_est;
count_cap = full(sum(Xcap, 'all')); % union of good and estimated matches
count_cup = full(sum(X_g, 'all') + sum(X_est, 'all') - count_cap); % difference between good and estimated matches
JD = 1-count_cap/count_cup;
fprintf('Jaccard distance = %f\n',JD)

% Compute precision
count_est = full(sum(X_est, 'all'));
PR = count_cap / count_est;
fprintf('precision rate = %f\n',PR)

% Compute recall
count_good = sum(X_g, 'all');
RC = count_cap / count_good;
fprintf('recall rate = %f\n',RC)



