% Include other folders
addpath('./example_icp');
init

gt_tfs = {};
gicp_p2p_error = [];
curv_p2p_error = [];
gicp_se3_error = [];
curv_se3_error = [];

for i = 1:100
    [source_pc, target_pc, gt_trans] = generate_model_trans('data/model/toy_downsample.mat');
    gt_tfs{i} = gt_trans;
    source_pc = downsample_pc(source_pc, 3);
    target_pc = downsample_pc(target_pc, 3);
    target.Location = target_pc';
    source.Location = source_pc';

    disp('Running curvature icp')
    [final_tf, tfed_pc, result] = curv_icp(source_pc, target_pc);
    disp('Running gicp');
    gicp_final_tf = gicp_fmin(target, source);
    gicp_tfed_pc = transform_pc(gicp_final_tf, source_pc);

    % Compute error
    disp('GICP point to point error')
    gicp_p2p_error = [gicp_p2p_error, calc_error(gicp_tfed_pc, target_pc)]
    disp('Curvature ICP point to point error')
    curv_p2p_error = [curv_p2p_error, calc_error(tfed_pc, target_pc)]

    disp('Ground truth TF')
    gt_trans
    disp('GICP TF and SE3 Norm')
    gicp_final_tf
    gicp_se3_error = [gicp_se3_error, norm(logm(gt_trans \ gicp_final_tf))]
    disp('Curvature TF and SE3 Norm')
    final_tf
    curv_se3_error = [curv_se3_error, norm(logm(gt_trans \ final_tf))]
    disp('<Curvature TF, GICP TF> SE3 Norm')
    norm(logm(gicp_final_tf \ final_tf))

    % Plot
    figure(1);
    clf;
    hold on;
    plot(1:i, curv_se3_error, 'r')
    plot(1:i, gicp_se3_error, 'b')
    title('SE3 error');
    legend;

    figure(2);
    clf;
    hold on;
    plot(1:i, curv_p2p_error, 'r')
    plot(1:i, gicp_p2p_error, 'b')
    title('P2P error');
    legend;
end
