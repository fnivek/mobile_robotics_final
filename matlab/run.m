function run(model,scene)
    % Runs the curvature ICP code
    %   data_file - The filename to load data from

    % Set some reasonable defaults
    plot_flag = true;
    ds_ratio.source = 1;  %downsample ratio, 1 for keeping raw data density
    ds_ratio.target = 10;
    if ~exist('model', 'var')
      model = 'data/model/toy_downsample.mat';
    end
    if ~exist('scene', 'var')
      scene = 'data/scene/scene1_easy.mat';
    end

    % Load the data
    [source_pc, target_pc] = load_data(model,scene,plot_flag,ds_ratio);

    % Run curvature ICP
    [final_tf, tfed_pc] = curv_icp(source_pc, target_pc);

    % Display results
end
