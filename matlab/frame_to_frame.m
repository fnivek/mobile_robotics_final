function frame_to_frame(make_video)
    ds_ratio.source = 10;  %downsample ratio, 1 for keeping raw data density
    ds_ratio.target = 10;
    frame_length = 20;
    scene{frame_length} = {};
    tf{frame_length} = {};
    tf{1} = eye(4);
    for i = 1 : frame_length
        frame_data = load(strcat('data/ftf_scene/',int2str(i),'.mat'));
        scene{i} = frame_data.scene;
    end
    for i = 1 : frame_length-1
        [tf{i+1},~] = run(false,make_video,scene{i+1},scene{i},ds_ratio);
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