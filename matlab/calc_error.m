function mse = calc_error(source_pc, target_pc)

    tree = KDTreeSearcher(target_pc');
    [neighbors, D] = knnsearch(tree, source_pc');
    [n,~] = size(D);
    mse = sum(D.^2) / n;
end
