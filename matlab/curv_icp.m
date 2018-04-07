function [final_tf, tfed_pc] = curv_icp(source_pc, target_pc, varargin) % guess, max_itters, err_converged_eps, err_diff_converged_eps)
    % Performs ICP with curvature
    %   source_pc - The source point cloud for ICP
    %   target_pc - The target point cloud for ICP
    %   final_tf - the final homogeneous transform

    % Handle optional args
    p = inputParser;
    addParameter(p, 'guess', eye(4));
    addParameter(p, 'max_itters', 100);
    addParameter(p, 'err_converged_eps', 1e-6);
    addParameter(p, 'err_diff_converged_eps', 1e-6);
    parse(p, varargin{:});

    % Initialize
    clear is_converged;
    icp.itter = 0;
    icp.source_pc = source_pc;
    icp.target_pc = target_pc;
    icp.final_tf = p.Results.guess;
    icp.max_itters = p.Results.max_itters;
    icp.err_converged_eps = p.Results.err_converged_eps;
    icp.err_diff_converged_eps = p.Results.err_diff_converged_eps;
    icp.nn_search_radius = 0.03;
    global result;
    global last_err;
    last_err = 0;

    % Apply guess
    icp.source_pc = transform_pc(icp.final_tf, icp.source_pc);

    % Calculate features
    [icp.source_feats] = calc_features(icp.source_pc, icp.nn_search_radius);
    [icp.target_feats] = calc_features(icp.target_pc, icp.nn_search_radius);

    tic;
    icp.correspondences = find_correspondences(icp);
    result.time = [result.time,toc];
    % Check convergence
    while ~is_converged(icp)
        result.ite = result.ite + 1;
        tic;
        
        % Optimize
        inc_tf = optimize(icp);

        % Apply transform
        icp.source_feats = transform_feature(inc_tf, icp.source_feats);
        icp.source_pc = icp.source_feats(1:3, :);

        % Update final transform
        % TODO: Check if accumulating correctly
        icp.final_tf = inc_tf * icp.final_tf;

        % Update loop vars
        icp.itter = icp.itter + 1;

        % Find correspondences
        icp.correspondences = find_correspondences(icp);

        % Display
        viz_current_itter(icp);
        % pause;
        
        % Update result
        result.time = [result.time,toc];
    end

    final_tf = icp.final_tf;
    tfed_pc = icp.source_pc;
end
