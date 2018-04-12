function [source_pc, target_pc] = load_data(model,scene,plot,ds_ratio)
    % Loads a source and target point cloud
    %   filename - file name of the data to load
    source_pc_struct = load(model);
    fn = fieldnames(source_pc_struct);
    source_pc = getfield(source_pc_struct,fn{1});
    
    target_pc_struct = load(scene);
    fn = fieldnames(target_pc_struct);
    target_pc = getfield(target_pc_struct,fn{1}); 
    
    %Downsample happens here 
    source_pc = downsample_pc(source_pc,ds_ratio.source);
    target_pc = downsample_pc(target_pc,ds_ratio.target);
    
    if(plot)
        figure(1)
        plot3(source_pc(1, :), source_pc(2, :), source_pc(3, :), 'r.');
        title("Source Point Cloud")
        axis equal
        figure(2)
        plot3(target_pc(1, :), target_pc(2, :), target_pc(3, :), 'k.');
        title("Target Point Cloud")
        axis equal
    end
end
