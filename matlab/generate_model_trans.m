function [model_after_trans, model_before_trans, M ] = generate_model_trans(model)
    plot_flag = false;
    % Load the data
    [source_pc, ~] = load_data(model,'data/scene/scene1_easy.mat',plot_flag);
    source_pc(4,:) = 1;
    tran = randn(1,3) * 0.1;
    rot = randn(1,3) * pi;
    M = makehgtform;
    M = makehgtform('translate',tran,'xrotate',rot(1),'yrotate',rot(2),'zrotate',rot(3));
    model_after_trans = M * source_pc;
    % plot3(source_pc(1,:),source_pc(2,:),source_pc(3,:),'r.');
    % axis equal
    % figure(2)
    % plot3(model_after_trans(1,:),model_after_trans(2,:),model_after_trans(3,:),'k.');
    % axis equal
    model_after_trans = model_after_trans(1:3,:);
    model_before_trans = source_pc(1:3,:);
end