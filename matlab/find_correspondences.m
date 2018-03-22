function correspondences = find_correspondences(source_pc, source_feats, target_pc, target_feats)
  % Finds correspondences between the source and target using the features
  %   source_pc - the source_pc points
  %   source_feats - the features of the source pc
  %   target_pc - the target_pc points
  %   target_feats - the features of the target pc
  %   correspondences - a n x 2 matrix where n is the number of points in source_pc and the first
  %     column is the index in source pc and the second column is the index in target pc
  correspondences = [1 1];


  %% Please refer to the following link for the fomulae of the distances.
  %  https://doi.org/10.1109/TPAMI.2017.2648803
    lambdap = 0.5;
    lambdan = 0.3;
    lambdac = 0.2;
    [~,m]=size(source_pc);
    [~,n]=size(target_pc);
    dis = zeros(n,m);
    for i=1:m
        % Calculate D_p, the euclidean distance between points.
        for j=1:n
         tmp_matrix = sqrt((target_pc(1,j)-source_pc(1,i))^2 + (target_pc(2,j)-source_pc(2,i))^2 + (target_pc(3,j)-source_pc(3,i))^2);
         dis(j,i) = tmp_matrix;
        end
        R = max(dis(:,i));
        % Normalize D_p.
        dis(:,i) = dis(:,i)/R;
        % Calculate D_n and D_c for each point pair.
        for j=1:n
            ni = source_feats(1:3,i);
            nj = target_feats(1:3,j);
            feati = source_feats(:,i);
            featj = target_feats(:,j);
            dn = Dn(ni,nj);
            dc = Dc(feati, featj, 0.01);
            % The distance metric.
            dis(j,i) = lambdap*dis(j,i) + lambdan*dn + lambdac*dc;
        end
    end

    % Find the minimum distance, find the correspondences.
    correspondences = zeros(m,1);
    min_dist = zeros(m,1);
    for i = 1:m
        [correspondences(i),~] = find(dis(:,i) == min(dis(:,i)));
        min_dist(i) = min(dis(:,i));
    end
    % The left column is the indices of the source points (1:number_of_source_points).
    % The right column is the corresponding indices of the target points.
    left=(1:m)';
    correspondences = [left,correspondences];

end


function dn = Dn(ni,nj)
% Distance between normals of the two points.
%   ni - Normal vector of the i-th source point.
%   nj - Normal vector of the j-th target point.
    dn = 1-ni'*nj;
end


function dc = Dc(feati, featj, thetak)
% Distance between curvatures of the two points.
%   feati - All the features of the i-th source point.
%   featj - All the features of the j-th target point.
%   thetak - A threshold to determine if k1 and k2 are similar or not.
    kmax = max(abs(feati(7)),abs(feati(8)));
    if abs(feati(7)-feati(8)) < thetak
        dc = (abs(feati(7) - featj(7))+abs(feati(8)) - featj(8)) / kmax;
    else
        ei1 = feati(4:6);
        ei2 = cross(feati(1:3), ei1);
        Ci = [ei1,ei2,feati(1:3)];
        Qi = Ci*diag([feati(7),feati(8),0])*Ci';
        ej1 = featj(4:6);
        ej2 = cross(featj(1:3), ej1);
        Cj = [ej1,ej2,featj(1:3)];
        Qj = Cj*diag([featj(7),featj(8),0])*Cj';
        dc = norm((Qi-Qj),2) / kmax;
    end
end
