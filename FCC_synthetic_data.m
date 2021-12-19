
%% Author: Yunpeng Shi

%% This is novel synthetic model for generating synthetic images and keypoing matches
%% It allows control of a variety of parameters 



%% Input parameters:
%% n: number of cameras
%% n_pt: number of 3D points
%% p: probability for connecting two cameras
%% q0: probability for removing a keypoint match (for elementwise corruption)
%% q1: probability for adding a wrong match (for elementwise corruption)
%% q2: probability for completely corrupting keypoint matches between two images (for inlier-outlier corruption model)


%% Output:

%% XMat: (i,j)-th block of XMat (denoted as Xij) is the matching matrix encoding the keypoint
%%         matches between i-th and j-th images. For example, Xij(k,l)=1 if and only if the k-th keypoint 
%%         in image i is matched to the l-th keypoint in image

%% XMat_gt: The ground truth matching matrix
%% mat_size: a vector of length n, whose i-th element is the number of keypoints in camera i




function[XMat, XMat_gt, mat_size] = FCC_synthetic_data(n,n_pt,p,q0,q1,q2)


    G = rand(n,n) < p;
    G = tril(G,-1);
    disp('generate initial camera adjacency matrix')
    %AdjMat = G + G'; 
    [Ind_j, Ind_i] = find(G==1);
    m = length(Ind_i);

    
    disp('create camera intrinsic matrices. f (focal length) is set to 5000 pixel sizes')
    % camera intrinsic matrix (image dimension is 1000 by 1000 pixels)
    f=5000; px=500; py=500;
    K=[f,0,px; 0,f,py; 0,0,1];

    disp('generate 3d point cloud (a sphere)')
    mu=[0,0,0]';
    Sigma = diag([1,1,1]);
    pts = (mvnrnd(mu,Sigma,n_pt))';
    pts_norm = sqrt(sum(pts.^2,1)); % Compute norms of columns
    pts = bsxfun(@rdivide,pts,pts_norm); % Normalization: 3d points on unit sphere
  


    disp('generate camera locations from 3d gaussian dist with radius constraints')
    T_gt = 10*randn(3,n);
    T_norm = sqrt(sum(T_gt.^2,1));
    T_scale = T_norm./(1+T_norm); % rescale each camera location
    T_gt = bsxfun(@rdivide,T_gt,T_scale); % make sure all locations are outside of the unit sphere


    %scatter3(T_gt(1,:),T_gt(2,:),T_gt(3,:))


    %initialize camera rotations 
    R_gt = zeros(3,3,n);

    %initialize camera projection matrix

    P_gt = zeros(3,4,n);

    %homogeneous coordinates of 3d pts
    pts_h = [pts; ones(1,n_pt)];


    % define functions for generating random camera orientations
    ssc = @(v) [0 -v(3) v(2); v(3) 0 -v(1); -v(2) v(1) 0];
    RU = @(A,B) eye(3) + ssc(cross(A,B)) + ssc(cross(A,B))^2*(1-dot(A,B))/(norm(cross(A,B))^2);

    %projected 2d coordinates of 3d points
    kpt = cell(1,n);
    %visible keypoints indices
    kpt_visible = cell(1,n);
    %visible keypoints coordinates
    kpt_visible_coord = cell(1,n);


    %initialize variables for partial perm
    %each relative partial perm Xij has size mi by mj
    %each absolute partial perm Pi has size mi by n_pt

    %vector that stores mi
    mat_size = zeros(1,n);
    %ground truth abs perm in cell structure
    Abs_perm_gt = cell(1,n);
    %ground truth abs perm in a block matrix (sum m_i by n_pt)
    PMat = [];
    
    disp('generating 2d keypoints from camera projection matrices');

    for i = 1:n
        
        % camera orientation points at the original, with additional 2d random
        % rotation around the orentation
        theta_temp=unifrnd(0,2*pi);
        R_temp = zeros(3,3);
        R_temp(1:2,1:2)=[cos(theta_temp),-sin(theta_temp); sin(theta_temp),cos(theta_temp)];
        R_temp(3,:)=[0,0,1];
        R_trans = RU([0,0,1]',-T_gt(:,i)./norm(T_gt(:,i)));
        R_gt(:,:,i)=R_temp*R_trans';
        
        % project back to SO(3)
        [U, ~, V]= svd(R_gt(:,:,i));
        S0 = diag([1,1,det(U*V')]);
        R_gt(:,:,i) = U*S0*V';
        % camera projection matrix
        P_gt(:,:,i) = K*[R_gt(:,:,i),-R_gt(:,:,i)*T_gt(:,i)];
        kpt_coord=P_gt(:,:,i)*pts_h;
        kpt_coord(1,:)= kpt_coord(1,:)./ kpt_coord(3,:);
        kpt_coord(2,:)= kpt_coord(2,:)./ kpt_coord(3,:);
        % keypoint coordinates [u,v,1]'
        kpt{i} = [kpt_coord(1:2,:);ones(1,n_pt)];
        % visibility of 3d point p by camera i is determined by (p-ti)'p<0
        % since we have a sphere object centered at 0
        visible_mat = (repmat(T_gt(:,i),[1,n_pt])-pts).*pts;
        visible_vec = sum(visible_mat);
        %make sure pixel coordinates are within the image pixel limits (1000 by
        %1000)
        kpt_visible{i} = find(visible_vec<0 & (kpt{i}(1,:)<=1000) & (kpt{i}(2,:)<=1000)& ...
            (kpt{i}(1,:)>=0) & (kpt{i}(2,:)>=0));
        kpt_visible_coord{i} = kpt{i}(:,kpt_visible{i});
        mat_size(i) = length(kpt_visible{i});
        Abs_perm_gt{i} = sparse(mat_size(i),n_pt);
        abs_ind = sub2ind([mat_size(i),n_pt],(1:mat_size(i)), kpt_visible{i});
        Abs_perm_gt{i}(abs_ind)=1;
        PMat = [PMat; Abs_perm_gt{i}];
        
       
    end







    AdjMat=sparse(n,n);
    % refine the graph topology by visibility
    for k=1:m
        i=Ind_i(k); j=Ind_j(k);
        num_common = length(intersect(kpt_visible{i}, kpt_visible{j}));
        if num_common>=8  %%%%%%%% criterion for connecting two images
           AdjMat(i,j)=1;
           AdjMat(j,i)=1;
        end
    end

    G = tril(AdjMat,-1);
    [Ind_j, Ind_i] = find(G==1);
    m = length(Ind_i);


    N=sum(mat_size);
    Xind = [0,cumsum(mat_size)]; % accumlative index (useful for assigning values in block matrix)
    Rel_perm_gt = cell(1,m); %%ground truth rel perm in cell structure
    Rel_perm = cell(1,m);   %corrupted given rel perm in cell structure
    XMat_gt = sparse(N,N); %ground truth rel perm in a block matrix (sum m_i by sum m_i)
    XMat = sparse(N,N); %corrupted given rel perm in a block matrix (sum m_i by sum m_i)
    
   
    disp('generating and corrupting keypoint matches');
    

    for k=1:m

        
        i=Ind_i(k); j=Ind_j(k);
        Rel_perm_gt{k} = Abs_perm_gt{i}*(Abs_perm_gt{j})';     
        XMat_gt( Xind(i)+1: Xind(i+1),  Xind(j)+1: Xind(j+1))=Rel_perm_gt{k};
        XMat_gt( Xind(j)+1: Xind(j+1),  Xind(i)+1: Xind(i+1))=(Rel_perm_gt{k})';
        Rel_perm{k} = Rel_perm_gt{k};

        temp = rand;
        if temp<q2     % with prob q2, completely corrupt the partial perm by randomly permute rows/columns of the matrix
           Rel_perm{k} = Rel_perm{k}(randperm(mat_size(i)), :);
           Rel_perm{k} = Rel_perm{k}(:, randperm(mat_size(j)));
        else
            max_row_vec = max(Rel_perm{k},[],2);

            nonzero_row = find(max_row_vec>0);
            zero_row = find(max_row_vec==0);

            for l = nonzero_row'  % if there is a match for the keypoint
                temp = rand;
                if temp<=q0   % with prob q0 remove it
                Rel_perm{k}(l,:)=0;
                end
            end
            max_col_vec = max(Rel_perm{k},[],1);
            zero_col = find(max_col_vec==0); % the list of keypoints in image j that are not matched to image i
            n_col = length(zero_col);

            for l = zero_row'   % if there is no match for the keypoint
                if n_col>0
                    temp = rand;
                    if temp<=q1   % with prob q1 add a random match
                        rand_col=randi(n_col);
                        Rel_perm{k}(l,zero_col(rand_col))=1; % make sure that we only add a match to an unmatched keypoint
                        n_col = n_col-1;
                        zero_col(rand_col)=[];           
                    end
                end
            end
        end

        XMat( Xind(i)+1: Xind(i+1),  Xind(j)+1: Xind(j+1))=Rel_perm{k};   % store data in a big block matrix (easy format for spectral and SDP methods)
        XMat( Xind(j)+1: Xind(j+1),  Xind(i)+1: Xind(i+1))=(Rel_perm{k})';
        
        

    end

end



