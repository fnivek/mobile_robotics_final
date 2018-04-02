function [tfed_feat] = transform_feature(tf, feat)
    [~, n] = size(feat);

    % Multiply components individually
    %   Note: the difference between vectors and ponts [x y z 0] vs [x y z 1]
    homogeneous_xyz = tf * [feat(1:3, :); ones(1, n)];
    homogeneous_norm = tf * [feat(4:6, :); zeros(1, n)];
    homogeneous_pa = tf * [feat(7:9, :); zeros(1, n)];

    % Reconstruct
    tfed_feat = [homogeneous_xyz(1:3, :); ...
                 homogeneous_norm(1:3, :); ...
                 homogeneous_pa(1:3, :); ...
                 feat(10:11, :)];
end
