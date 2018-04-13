function opt_tf = ransac_optimize(icp)
% Find an optimal transform between source and target
    %   icp.correspondences - a n x 2 matrix where n is the number of points in source_pc and the first
    %   column is the index in source pc and the second column is the index in target pc
    %   icp.source_pc - the source_pc points
    %   icp.target_pc - the target_pc points
    %   opt_tf - is the resulting homogeneous transform produced by finding the optimal transformation

% Extract the RANSAC coefficients
coeff.numPtsToSample = 6;     % the minimum number of correspondences needed to fit a model
coeff.iterNum = 2000;         % number of iterations to run the RANSAC sampling loop
coeff.thDist = .12;           % inlier distance threshold; units are in pixels
coeff.requiredInliers = length(icp.correspondences) * 0.2;

opt_tf = eye(4);
besterr = inf;

for i = 1:coeff.iterNum
    % Take random sample of correspondences
    
    % convert to original source cloud index
    orig_corr(:,1) = icp.source_pc_sample_index(icp.correspondences(:,1));
    orig_corr(:,2) = icp.correspondences(:,2);
%     rsample = randperm(length(icp.correspondences),coeff.numPtsToSample);
    rsample = randperm(length(orig_corr),coeff.numPtsToSample);
%     sample_corr = icp.correspondences(rsample,:);
    sample_corr = orig_corr(rsample,:);

    % Random sample transform of source point cloud
    sample_tf = optimize(icp,sample_corr);
    guess_pc = transform_pc(sample_tf,icp.source_pc);

    % Euclidean distance calculation
%     error_pc = (icp.target_pc(:,icp.correspondences(:,2)) - guess_pc(:,icp.correspondences(:,1))).^2;
    error_pc = (icp.target_pc(:,orig_corr(:,2)) - guess_pc(:,orig_corr(:,1))).^2;
    error = sqrt(sum(error_pc,1));

    % Inlier threshold calculation
    numinlier = 0;
    for j = 1:1:length(error)
        if error(j) < coeff.thDist
            numinlier = numinlier + 1;
        end
    end

    % Calclate how good the model is by minimizing avg distance error
    if numinlier > coeff.requiredInliers
        avgerror = mean(error);
        if avgerror < besterr
            besterr = avgerror;
            opt_tf = sample_tf;
        end
    end

%     if numinlier > maxinlier
%         maxinlier = numinlier;
%         opt_tf = sample_tf;
%     end
end
% Probably remove, just for edge case if inliers not found
if opt_tf == 0
    disp('this sucks')
%     opt_tf = optimize(icp,icp.correspondences);
    opt_tf = optimize(icp,orig_corr);
end

end
