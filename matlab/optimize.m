function opt_tf = optimize(icp)
    % Find an optimal transform between source and target
    %   icp.correspondences - a n x 2 matrix where n is the number of points in source_pc and the first
    %     column is the index in source pc and the second column is the index in target pc
    %   icp.source_pc - the source_pc points
    %   icp.target_pc - the target_pc points
    %   opt_tf - is the resulting homogeneous transform produced by finding the optimal transformation
    opt_tf = eye(4);
end
