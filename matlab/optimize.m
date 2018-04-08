function opt_tf = optimize(icp,corr)
    % Find an optimal transform between source and target
    %   icp.correspondences - a n x 2 matrix where n is the number of points in source_pc and the first
    %   column is the index in source pc and the second column is the index in target pc
    %   icp.source_pc - the source_pc points
    %   icp.target_pc - the target_pc points
    %   opt_tf - is the resulting homogeneous transform produced by finding the optimal transformation
    if nargin < 2
        corr = icp.correspondences;
    end
    % make new source pt_cloud P and target pt_cloud based off of correspondences
    source_corr = corr(:,1);
    target_corr = corr(:,2);
    P = icp.source_pc(:,source_corr);
    X = icp.target_pc(:,target_corr);
    
    % From GetTransformation as part of Maani's SVD function (requires AA MATLAB package)
    % Derived from class lecture on ICP
    % P and X are 3xN matrixes for N points in 3 dimensions, i.e. each column
    % corresponds to a single point. See Besl & McKay (1992) section III.C.
    
    mu_p = sum(P,2)/size(P,2);
    mu_x = sum(X,2)/size(X,2);

    Sigma = (P*X')/size(P,2) - mu_p*mu_x';

    A = (Sigma - Sigma');
    Delta = [A(2,3) A(3,1) A(1,2)]';

    tr = trace(Sigma);
    Q = [tr Delta'; Delta (Sigma + Sigma' - tr*eye(3))];

    [V,D] = eig(Q);
    [~,index] = max(diag(D));
    q_r = V(:,index);

    R = quat2dcm(q_r')';
    t = mu_x - R * mu_p;

    opt_tf = [R, t; 0 0 0 1];
%     opt_tf = eye(4);
end
