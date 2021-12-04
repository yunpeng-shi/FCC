%% Author: Yunpeng Shi

%% Given input matching matrix X, outputs the statistics matrix S


%% Input:

%% X: (i,j)-th block of XMat (denoted as Xij) is the matching matrix encoding the keypoint
%%    matches between i-th and j-th images. For example, Xij(k,l)=1 if and only if the k-th keypoint 
%%    in image i is matched to the l-th keypoint in image
%% mat_size: a vector of length n, whose i-th element is the number of keypoints in camera i
%% path_length: count paths that connects keypoints i,k in 2 steps, and j,k in 2 steps 
%%              (longer paths are not needed as we use message-pssing)
%% n_iter: number of iterations. default is 10. For large datasets one may use a smaller number 
%%         and use iterative thresholding to accelarate the convergence
%% n_batch: number of batches (use more number of batches if your computer has small memory)
%% rounding: default = 0, namely no iterative thresholding. Rounding is in general not recommended, except for large datasets
%%           one may want to accelerate convergence and sacrifies a little bit accuracy.
%% thresholds: a vector of length n_iter that stores the threshold in each iteration. ONLY use when rounding = 1


%% Output:

%% S: S(i,j) is between 0 and 1, and is interpreted as the
%%    confidence/probability that X(i,j) is a good keypoint match.



function S = FCC(X, mat_size, path_length, n_iter, n_batch, rounding, thresholds)

    disp('start running FCC');
    n = length(mat_size);
    Xind = [0,cumsum(mat_size)];
    N = length(X);


    IndX = find(X>0);
    [IndX_i,IndX_j] = ind2sub(size(X),IndX);
    m = length(IndX_i);


    batch_size_end = mod(m,n_batch);
    if batch_size_end == 0
        batch_size = m/n_batch;
        batch_size_vec = batch_size*ones(1,n_batch);
    else
        batch_size_end = mod(m,n_batch-1);
        batch_size = (m-batch_size_end)/(n_batch-1);
        batch_size_vec = [batch_size*ones(1,n_batch-1), batch_size_end];
        
    end
    

    batch_ind = [0,cumsum(batch_size_vec)];


    for t=1:n_iter


        Xr = X^path_length;
        S_vec_long = zeros(m,1);
        for batch = 1:n_batch

            ind_range = (batch_ind(batch)+1):batch_ind(batch+1);

            Xri = Xr(:,IndX_i(ind_range));
            Xri_t = Xri';    
            clear Xri;

            Xrj = Xr(:,IndX_j(ind_range));       
            Xrj_t = Xrj';
            clear Xrj;

            Xrij = Xri_t.*Xrj_t;
            S1_vec = sum(Xrij,2);


            S1_plus_S2_vec = zeros(batch_size_vec(batch),1);

            for k = 1:n
                Xri_blk = Xri_t(:,(Xind(k)+1): Xind(k+1));
                Xrj_blk = Xrj_t(:,(Xind(k)+1): Xind(k+1));
                S1_plus_S2_vec = S1_plus_S2_vec + sum(Xri_blk,2).*sum(Xrj_blk,2);
            end

            S_vec = S1_vec./(S1_plus_S2_vec+1e-4);

            clear Xri_t;
            clear Xrj_t;

            S_vec_long(ind_range) = S_vec;
        end
        clear Xr;

       

        S = sparse(IndX_i, IndX_j, S_vec_long, N,N);
       
        X=S;
        
        if rounding ==1
            X=(S>thresholds(t));
        end
        
        fprintf('iteration %d Completed!\n',t)
        
    end


    
end