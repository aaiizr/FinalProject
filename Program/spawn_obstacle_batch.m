function [obs_states, obs_goals, obstacles, radius, M_obs, obstacle_types] = spawn_obstacle_batch(obs_states, obs_goals, obstacles, radius, M_obs, obstacle_types, datang, obstacles_per_spawn, r_dyn)
    
    fprintf('Spawning batch of %d obstacles...\n', obstacles_per_spawn);
    
    for batch_idx = 1:obstacles_per_spawn
        % Add new obstacle state
        new_obs_idx = M_obs + 1;
        
        % Expand arrays if needed
        if size(obs_states, 2) < new_obs_idx
            obs_states = [obs_states, zeros(12, 1)];
            obs_goals = [obs_goals, zeros(3, 1)];
        end
        
        % Set spawn position based on datang parameter (same logic as original)
        if datang == 1 %% Dari Depan
                start_pos = [180+randn*10; 180+randn*10; 65+randn*10];
            
                % Set goal to another position
                end_pos = [-40+randn*1; -40+randn*1; 60+randn*10];
                elseif datang == 2
                    end_pos = [580+randn*20; 580+randn*20; 62+randn*2];
            
                    % Set goal to another position
                    start_pos = [30+randn*20; 30+randn*20; 62+randn*2];
                else
                     end_pos = [300+randn*10; 60+randn*10; 55+randn*8];
            
                    % Set goal to another position
                    start_pos = [100+randn*1; 210+randn*1; 55+randn*8];
                end
        
        % Initialize new obstacle
        obs_states(1:3, new_obs_idx) = start_pos;
        obs_states(4:12, new_obs_idx) = zeros(9,1);
        obs_goals(:, new_obs_idx) = end_pos;
        
        % Update obstacle arrays
        obstacles = [obstacles, start_pos];
        radius = [radius; r_dyn];
        M_obs = new_obs_idx;
        
        % Update obstacle types if in mixed mode
        if exist('obstacle_types', 'var')
            obstacle_types = [obstacle_types, 2]; % 2 = dynamic
        end
        
        fprintf('  - Obstacle %d/%d spawned at [%.1f, %.1f, %.1f]\n', ...
            batch_idx, obstacles_per_spawn, start_pos(1), start_pos(2), start_pos(3));
    end
end
