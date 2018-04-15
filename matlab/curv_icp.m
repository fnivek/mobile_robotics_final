function [final_tf, tfed_pc, result] = curv_icp(source_pc, target_pc, varargin) % guess, max_itters, err_converged_eps, err_diff_converged_eps)
    % Performs ICP with curvature
    %   source_pc - The source point cloud for ICP
    %   target_pc - The target point cloud for ICP
    %   final_tf - the final homogeneous transform

    % Handle optional args
    p = inputParser;
    addParameter(p, 'guess', eye(4));
    addParameter(p, 'max_itters', 20);
    addParameter(p, 'err_converged_eps', 1e-3);
    addParameter(p, 'err_diff_converged_eps', 1e-4);
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
    % icp.euc_dist = [];
    % global result;
    icp.result={};
    icp.result.time=[];
    icp.result.ite=1;
    icp.result.err=[];
    % global Param;
    % makeVideo = Param.mV;
    % pauseLen = Param.pauseLen;
    % % Initialize video
    % if makeVideo
    %     try
    %         votype = 'avifile';
    %         vo = avifile('video.avi', 'fps', min(5, 1/pauseLen));
    %     catch
    %         votype = 'VideoWriter';
    %         vo = VideoWriter('video');
    %         set(vo, 'FrameRate', min(5, 1/pauseLen));
    %         open(vo);
    %     end
    % end

    % Apply guess
    icp.source_pc = transform_pc(icp.final_tf, icp.source_pc);

    % Calculate features
    [icp.source_feats] = calc_features(icp.source_pc, icp.nn_search_radius);
    [icp.target_feats] = calc_features(icp.target_pc, icp.nn_search_radius);

    tic;
    [icp.correspondences,icp.euc_dist] = find_correspondences(icp);
    icp.result.time = [icp.result.time,toc];
    % Check convergence
    [converged, icp.result.err]=is_converged(icp);
    while ~converged
        icp.result.ite = icp.result.ite + 1;
        tic;

        % Optimize
        % inc_tf = optimize(icp);
        inc_tf = ransac_optimize(icp);

        % Apply transform
        icp.source_feats = transform_feature(inc_tf, icp.source_feats);
        icp.source_pc = icp.source_feats(1:3, :);

        % Update final transform
        % TODO: Check if accumulating correctly
        icp.final_tf = inc_tf * icp.final_tf;

        % Update loop vars
        icp.itter = icp.itter + 1;

        % Find correspondences
        [icp.correspondences,icp.euc_dist] = find_correspondences(icp);

        % Display
        viz_current_itter(icp);
        % if pauseLen == inf
        %     pause;
        % elseif pauseLen > 0
        %     pause(pauseLen);
        % end
        % % pause;
        % if makeVideo
        %     F = getframe(gcf);
        %     switch votype
        %       case 'avifile'
        %         vo = addframe(vo, F);
        %       case 'VideoWriter'
        %         writeVideo(vo, F);
        %       otherwise
        %         error('unrecognized votype');
        %     end
        % end
        % Update result
        icp.result.time = [icp.result.time,toc];

        % Check convergence
        [converged, icp.result.err] = is_converged(icp);
    end

    final_tf = icp.final_tf;
    tfed_pc = icp.source_pc;
    result = icp.result;

    % % Make video
    % if makeVideo
    %     fprintf('Writing video...');
    %     switch votype
    %       case 'avifile'
    %         vo = close(vo);
    %       case 'VideoWriter'
    %         close(vo);
    %       otherwise
    %         error('unrecognized votype');
    %     end
    %     fprintf('done\n');
    % end
end


