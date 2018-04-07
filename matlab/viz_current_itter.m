function viz_current_itter(icp, varargin)
    % Handle optional args
    p = inputParser;
    addParameter(p, 'plot_clouds', true);
    addParameter(p, 'cloud_skip', 1);
    addParameter(p, 'plot_correspondences', true);
    addParameter(p, 'correspondences_skip', 10);
    addParameter(p, 'plot_feats', true);
    addParameter(p, 'feats_skip', 10);
    parse(p, varargin{:});
    % Init
    plot_clouds = p.Results.plot_clouds;
    cloud_skip = p.Results.cloud_skip;
    plot_correspondences = p.Results.plot_correspondences;
    correspondences_skip = p.Results.correspondences_skip;
    plot_feats = p.Results.plot_feats;
    feats_skip = p.Results.feats_skip;

    % Setup figure
    figure(3);
    clf;
    hold on;
    title('Itteration ' + string(icp.itter));
    axis equal;

    % Plot the clouds
    if plot_clouds
        plot3(icp.source_pc(1, 1:cloud_skip:end), icp.source_pc(2, 1:cloud_skip:end), icp.source_pc(3, 1:cloud_skip:end), 'k.');
        plot3(icp.target_pc(1, 1:cloud_skip:end), icp.target_pc(2, 1:cloud_skip:end), icp.target_pc(3, 1:cloud_skip:end), 'r.');
    end

    % Plot correspondences
    if plot_correspondences
        for coor = icp.correspondences(1:correspondences_skip:end, :)'
            source_pt = icp.source_pc(:, coor(1));
            target_pt = icp.target_pc(:, coor(2));
            plot3([source_pt(1), target_pt(1)], ...
                  [source_pt(2), target_pt(2)], ...
                  [source_pt(3), target_pt(3)], ...
                  'b');
        end
    end

    % Plot normals and Principal axis of curvature
    if plot_feats
        for feat = icp.target_feats(:, 1:feats_skip:end)
            p = feat(1:3);
            % Normal
            n = feat(4:6) / 100;
            plot3([p(1) p(1) + n(1)], [p(2) p(2) + n(2)], [p(3) p(3) + n(3)], 'g');
            % Principal axis of curvature
            pc_axis = feat(7:9) / 100;
            plot3([p(1) p(1) + pc_axis(1)], [p(2) p(2) + pc_axis(2)], [p(3) p(3) + pc_axis(3)], 'c');
        end
        for feat = icp.source_feats(:, 1:feats_skip:end)
            p = feat(1:3);
            % Normal
            n = feat(4:6) / 100;
            plot3([p(1) p(1) + n(1)], [p(2) p(2) + n(2)], [p(3) p(3) + n(3)], 'g');
            % Principal axis of curvature
            pc_axis = feat(7:9) / 100;
            plot3([p(1) p(1) + pc_axis(1)], [p(2) p(2) + pc_axis(2)], [p(3) p(3) + pc_axis(3)], 'c');
        end
    end
end
