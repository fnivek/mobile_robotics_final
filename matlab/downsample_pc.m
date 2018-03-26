function output_pc = downsample_pc(raw_pc,downsample_ratio)
    n = size(raw_pc,2);
    random_choice = randperm(n,fix(n/downsample_ratio));
    output_pc = raw_pc(:,random_choice);


end