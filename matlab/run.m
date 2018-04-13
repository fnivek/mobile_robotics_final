function run(type,makeVideo,model,scene,ds_ratio_source,ds_ratio_target)
    % Runs the curvature ICP code
    % data_file - The filename to load data from

    % Include other folders
    addpath('./example_icp');
    init

    % Set some reasonable defaults
    %global result;
    global Param;
    %result.time = [];
    %result.ite = 1;
    %result.err = [];
    Param.mV = makeVideo;
    Param.pauseLen = 0.3;


    plot_flag = false;

    if ~exist('type', 'var')
        type = 'mts';
    end
    if ~exist('model', 'var')
      model = 'data/model/toy_downsample.mat';
    end
    if ~exist('scene', 'var')
      scene = 'data/scene/scene1_easy.mat';
    end
    if ~exist('ds_ratio_source', 'var')
      ds_ratio_source = 10;
    end
    if ~exist('ds_ratio_target', 'var')
      ds_ratio_target = 10;
    end
    if ~exist('makeVideo','var') || isempty(makeVideo)
        makeVideo = false;
    end
    ds_ratio.source = ds_ratio_source;
    ds_ratio.target = ds_ratio_target;
    switch type
        case 'mtm'
            [source_pc, target_pc, gt_trans] = generate_model_trans(model);
            source_pc = downsample_pc(source_pc,ds_ratio.source);
            target_pc = downsample_pc(target_pc,ds_ratio.target);
            [final_tf, tfed_pc, result] = curv_icp(source_pc, target_pc);
            disp('Running gicp');
            target.Location = target_pc';
            source.Location = source_pc';
            gicp_final_tf = gicp_fmin(target, source);
            gicp_tfed_pc = transform_pc(gicp_final_tf, source_pc);
        case 'mts'
            [source_pc, target_pc] = load_data(model,scene,plot_flag);
            source_pc = downsample_pc(source_pc,ds_ratio.source);
            target_pc = downsample_pc(target_pc,ds_ratio.target);
            target_pc(:, remove_table(target_pc)) = [];
            [final_tf, tfed_pc, result] = curv_icp(source_pc, target_pc);
            disp('Running gicp');
            target.Location = target_pc';
            source.Location = source_pc';
            gicp_final_tf = gicp_fmin(target, source);
            gicp_tfed_pc = transform_pc(gicp_final_tf, source_pc);
        case 'ftf'

            frame_length = 20;
            scene{frame_length} = {};
            tf{frame_length} = {};
            tf{1} = eye(4);
            for i = 1 : frame_length
                frame_data = load(strcat('data/ftf_scene/',int2str(i),'.mat'));
                scene{i} = frame_data.scene;
                scene{i} = downsample_pc(scene{i},ds_ratio.source);
            end
            for i = 1 : frame_length-1
                source_pc = scene{i+1};
                target_pc = scene{i};
                % Do we actually want to remove the table?
                target_pc(:, remove_table(target_pc)) = [];
                [tf{i+1},~] = curv_icp(source_pc,target_pc);
                tf{i+1} = tf{i+1} * tf{i};
            end
            for i = 1 : frame_length
                temp_pc = scene{i};
                temp_pc(4,:) = 1;
                transformed_pc = tf{i} * temp_pc ;
                plot3(transformed_pc(1,:),transformed_pc(2,:),transformed_pc(3,:),'k.');
                hold on
            end
    end

    % Remove the table from the target_pc
    % target_pc(:, remove_table(target_pc)) = [];

    % Results

    if type == string('mts') || type == string('mtm')
        figure;
        hold on;
        plot3(target_pc(1, :), target_pc(2, :), target_pc(3, :), 'k.');
        plot3(tfed_pc(1, :), tfed_pc(2, :), tfed_pc(3, :), 'r.');
        plot3(gicp_tfed_pc(1, :), gicp_tfed_pc(2, :), gicp_tfed_pc(3, :), 'b.');
        title('GICP vs Curvature ICP');
    end

    if type == string('mtm')
        % Compute error
        disp('GICP point to point error')
        calc_error(gicp_tfed_pc, target_pc)
        disp('Curvature ICP point to point error')
        calc_error(tfed_pc, target_pc)

        disp('Ground truth TF')
        gt_trans
        disp('GICP TF and SE3 Norm')
        gicp_final_tf
        norm(logm(gt_trans \ gicp_final_tf))
        disp('Curvature TF and SE3 Norm')
        final_tf
        norm(logm(gt_trans \ final_tf))
        disp('<Curvature TF, GICP TF> SE3 Norm')
        norm(logm(gicp_final_tf \ final_tf))
    end

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
    plot(2:result.ite, result.err(2:result.ite));
    title('Mean Squared Error');
    xlabel('iteration');
end
