function [tfed_pc] = transform_pc(tf, pc)
    [~, n] = size(pc);
    homogeneous_pc = tf * [pc; ones(1, n)];
    tfed_pc = homogeneous_pc(1:3, :);
end
