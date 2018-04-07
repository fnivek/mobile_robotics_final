function [model_after_trans, M] = generate_model_trans(model)
    plot_flag = false;
    ds_ratio.source = 1;  %downsample ratio, 1 for keeping raw data density
    ds_ratio.target = 10;
    if ~exist('model', 'var')
      model = 'data/model/toy_downsample.mat';
    end
    % Load the data
    [source_pc, ~] = load_data(model,'data/scene/scene1_easy.mat',plot_flag,ds_ratio);
    source_pc(4,:) = 1;
    tran = randn(1,3) * 0.1;
    rot = randn(1,3) * pi;
    M = makehgtform;
    M = makehgtform('translate',tran,'xrotate',rot(1),'yrotate',rot(2),'zrotate',rot(3));
    model_after_trans = M * source_pc;
    plot3(source_pc(1,:),source_pc(2,:),source_pc(3,:),'r.');
    hold on
    plot3(model_after_trans(1,:),model_after_trans(2,:),model_after_trans(3,:),'k.');
    axis equal
    model_after_trans = model_after_trans(1:3,:);
end