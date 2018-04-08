function table_indicies = remove_table(pc)
    % Removes the largest plannar region
    %   Uses ransac to find the largest planar region
    %   pc - input point cloud

    % Algorithm params
    num_pts_in_plane = 3; % Number of points needed to make a plane
    max_itters = 1000; % Maximum number of iterations
    thres = 0.01; % Distance an inlier can be from plane
    required_num_inliers = 100; % The required number of points to be considered a plane

    % Init
    [~, num_pts] = size(pc);
    % best_err = inf;
    table_indicies = [];
    best_num_inliers = 0;

    % Perform RANSAC
    for i = 1:max_itters
        % Sample points
        samp = pc(:, randi([1, num_pts], 1, num_pts_in_plane));
        % Fit the samples
        [normal, ~, pt] = affine_fit(samp');
        d = pt * normal;

        % Find all inliers
        num_inliers = 0;
        inliers = abs(normal' * pc - d) < thres;
        num_inliers = sum(inliers);

        % If there are enough points then fit the model to all inliers
        if num_inliers > required_num_inliers
            % inlier_indices = find(inliers == 1);
            % inlier_pc = pc(:, inlier_indices);
            % [normal, ~, pt] = affine_fit(inlier_pc');

            % % Calculate the error
            % err = sse(normal, pt * normal, inlier_pc) / num_inliers;
            % % Update the best
            % if err < best_err
            %     best_err = err
            %     num_inliers
            %     table_indicies = inlier_indices;
            % end

            % Find the model with the most inliers
            if  num_inliers > best_num_inliers
                best_num_inliers = num_inliers;
                table_indicies = find(inliers == 1);
            end
        end
    end
end

% https://www.mathworks.com/matlabcentral/fileexchange/43305-plane-fit
function [n,V,p] = affine_fit(X)
    %Computes the plane that fits best (lest square of the normal distance
    %to the plane) a set of sample points.
    %INPUTS:
    %
    %X: a N by 3 matrix where each line is a sample point
    %
    %OUTPUTS:
    %
    %n : a unit (column) vector normal to the plane
    %V : a 3 by 2 matrix. The columns of V form an orthonormal basis of the
    %plane
    %p : a point belonging to the plane
    %
    %NB: this code actually works in any dimension (2,3,4,...)
    %Author: Adrien Leygue
    %Date: August 30 2013

    %the mean of the samples belongs to the plane
    p = mean(X,1);

    %The samples are reduced:
    R = bsxfun(@minus,X,p);
    %Computation of the principal directions if the samples cloud
    [V,D] = eig(R'*R);
    %Extract the output from the eigenvectors
    n = V(:,1);
    V = V(:,2:end);
end

% function err = sse(normal, d, pts)
%     err = normal' * pts - d;
%     err = err.^2;
%     err = sum(err);
% end
