function run(data_file)
  % Runs the curvature ICP code
  %   data_file - The filename to load data from

  % Set some reasonable defaults
  if ~exist('data_file', 'var')
    data_file = 'data.mat';
  end

  % Load the data
  [source_pc, target_pc] = load_data(data_file);

  % Run curvature ICP
  curv_icp(source_pc, target_pc);

  % Display results
end
