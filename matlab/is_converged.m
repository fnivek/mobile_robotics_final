function converged = is_converged(itter, source_pc, target_pc)
  % Checks if the algorithm has converged
  %   itter - the current iteration number
  %   source_pc - the source point cloud
  %   target_pc - the target point cloud
  %   converged - a bool representing if convergence has occurred or not

  % Persistent vars
  persistent last_err;

  % Constants
  MAX_ITTERS = 100;
  ERR_CONVERGED_EPS = 1e-6;
  ERR_DIFF_CONVERGED_EPS = 1e-6;

  % Calculate error
  err = 0;
  err_diff = abs(last_err - err);

  % Set convergence bool
  converged = (itter > MAX_ITTERS); % || err < ERR_CONVERGED_EPS || err_diff < ERR_DIFF_CONVERGED_EPS;

  % Update last error
  last_err = err;
end
