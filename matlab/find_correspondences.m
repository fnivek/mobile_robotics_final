function correspondences = find_correspondences(source_pc, source_feats, target_pc, target_feats)
  % Finds correspondences between the source and target using the features
  %   source_pc - the source_pc points
  %   source_feats - the features of the source pc
  %   target_pc - the target_pc points
  %   target_feats - the features of the target pc
  %   correspondences - a n x 2 matrix where n is the number of points in source_pc and the first
  %     column is the index in source pc and the second column is the index in target pc
  correspondences = [1 1];
end
