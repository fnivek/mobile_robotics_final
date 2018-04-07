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

    % Results
    global result;
    result.time = [];
    result.ite = 1;
    result.err = [];

    % Run curvature ICP
    [final_tf, tfed_pc] = curv_icp(source_pc, target_pc);

    % Display results

    acc_time = zeros(1, result.ite);
    acc_time(1) = result.time(1);
    for i=2:result.ite
        acc_time(i) = acc_time(i - 1) + result.time(i);
    end
    acc_time = acc_time / 60;
    save new_result result acc_time;
    figure;
    stairs(1:result.ite, acc_time);
    title('Total computation time');
    xlabel('iteration');
    ylabel('accumulated time (min)')
    figure;
    plot(1:result.ite, result.err);
    title('Mean Squared Error');
    xlabel('iteration');
end
