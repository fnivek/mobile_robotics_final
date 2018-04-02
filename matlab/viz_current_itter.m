function viz_current_itter(icp)
    % Setup figure
    figure(3);
    clf;
    hold on;
    title('Itteration ' + string(icp.itter));
    axis equal;

    % Plot the clouds
    plot3(icp.source_pc(1, :), icp.source_pc(2, :), icp.source_pc(3, :), 'k.');
    plot3(icp.target_pc(1, :), icp.target_pc(2, :), icp.target_pc(3, :), 'r.');

    % Plot coorespondences
    for coor = icp.correspondences'
        source_pt = icp.source_pc(:, coor(1));
        target_pt = icp.target_pc(:, coor(2));
        plot3([source_pt(1), target_pt(1)], ...
              [source_pt(2), target_pt(2)], ...
              [source_pt(3), target_pt(3)], ...
              'b');
    end
end
