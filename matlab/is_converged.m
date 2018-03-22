function converged = is_converged(icp)
    % Checks if the algorithm has converged
    %   icp.itter - the current iteration number
    %   icp.source_pc - the source point cloud
    %   icp.target_pc - the target point cloud
    %   converged - a bool representing if convergence has occurred or not

    % Persistent vars
    persistent last_err;

    % Calculate error
    err = 0;
    err_diff = abs(last_err - err);

    % Set convergence bool
    converged = (icp.itter > icp.max_itters); % || err < icp.err_converged_eps || err_diff < icp.err_diff_converged_eps;

    % Update last error
    last_err = err;
end
