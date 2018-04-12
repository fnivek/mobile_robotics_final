function [converged, errors] = is_converged(icp)
    % Checks if the algorithm has converged
    %   icp.itter - the current iteration number
    %   icp.source_pc - the source point cloud
    %   icp.target_pc - the target point cloud
    %   converged - a bool representing if convergence has occurred or not

    % Persistent vars
    persistent last_err;
    % global result;

    % Calculate error
    src_cor = icp.correspondences(:,1);
    tar_cor = icp.correspondences(:,2);
    [n,~] = size(icp.correspondences);
    dis = zeros(1,n);
    for i=1:n
        src_ind = src_cor(i);
        tar_ind = tar_cor(i);
        dis(i) = icp.euc_dist(tar_ind, src_ind) ^ 2;
    end
    
    err = sum(dis) / n;
    if last_err
        err_diff = abs(last_err - err);
    else
        err_diff = err;
    end

    % Set convergence bool
    converged = (icp.itter > icp.max_itters) || (err < icp.err_converged_eps) || (err_diff < icp.err_diff_converged_eps);

    % Update last error
    last_err = err;
    errors = icp.result.err;
    errors = [errors, err];
end
