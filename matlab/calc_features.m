function feats = calc_features(pc, nn_search_radius)
    % Calculate the curvature features
    %   pc - the input point cloud
    %   feats - the calculated features
    [~, n] = size(pc);
    feats = zeros(11, n);

    % Copy xyz
    feats(1:3, :) = pc;

    % Build a KD-Tree
    tree = KDTreeSearcher(pc');
    neighbors = rangesearch(tree, pc', nn_search_radius);

    for i = 1:n
        [~, num_neighbors] = size(neighbors{i});
        if num_neighbors <= 3
            feats(4:end, i) = NaN;
            continue;
        end
        [coeffs, ~, lambdas] = pca(pc(:, neighbors{i})');

        feats(4:6, i) = coeffs(:, 3); % normal
        feats(7:9, i) = coeffs(:, 1); % principle_axis_curv
        feats(10, i) = lambdas(1);% / num_neighbors; % k1
        feats(11, i) = lambdas(2);% / num_neighbors; % k2
    end
end
