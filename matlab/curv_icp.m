function [final_tf] = curv_icp(source_pc, target_pc)
  % Performs ICP with curvature
  %   source_pc - The source point cloud for ICP
  %   target_pc - The target point cloud for ICP
  %   final_tf - the final homogeneous transform

  % Initialize
  final_tf = eye(4);
  clear is_converged;
  itter = 0;

  % Calculate features
  [source_feats] = calc_features(source_pc);
  [target_feats] = calc_features(target_pc);

  % Check convergence
  while ~is_converged(itter, source_pc, target_pc)
    % Find correspondences
    correspondences = find_correspondences(source_pc, source_feats, target_pc, target_feats);

    % Optimize
    inc_tf = optimize(correspondences, source_pc, target_pc);

    % Apply transform

    % Update final transform
    % TODO: Check if accumulating correctly
    final_tf = inc_tf * final_tf;

    % Update loop vars
    itter = itter + 1;
  end
end
