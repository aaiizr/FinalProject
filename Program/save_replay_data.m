function save_replay_data(replay_data, end_type)
    % Save the replay data with timestamp
    timestamp = datestr(now, 'yyyymmdd_HHMMSS');
    filename = sprintf('UAV_3D_Replay_%s_%s_%s.mat', ...
        replay_data.settings.apf_name, replay_data.settings.mode_str, timestamp);
    
    % Remove spaces and special characters
    filename = regexprep(filename, '[^\w\.]', '_');
    
    % Trim arrays to actual size
    actual_iter = replay_data.actual_iterations;
    replay_data.dynamic.agent_positions = replay_data.dynamic.agent_positions(:, :, 1:actual_iter);
    replay_data.dynamic.agent_orientations = replay_data.dynamic.agent_orientations(:, :, 1:actual_iter);
    replay_data.dynamic.agent_assignments = replay_data.dynamic.agent_assignments(:, 1:actual_iter);
    replay_data.dynamic.agent_status = replay_data.dynamic.agent_status(:, 1:actual_iter);
    replay_data.dynamic.agent_crashed = replay_data.dynamic.agent_crashed(:, 1:actual_iter);
    
    if isfield(replay_data.dynamic, 'obstacle_positions')
        replay_data.dynamic.obstacle_positions = replay_data.dynamic.obstacle_positions(:, :, 1:actual_iter);
        replay_data.dynamic.obstacle_orientations = replay_data.dynamic.obstacle_orientations(:, :, 1:actual_iter);
        replay_data.dynamic.obstacle_states = replay_data.dynamic.obstacle_states(:, :, 1:actual_iter);
    end
    
    % Save additional info
    replay_data.save_info = struct();
    replay_data.save_info.end_type = end_type;
    replay_data.save_info.save_time = datestr(now);
    replay_data.save_info.total_frames = actual_iter;
    
    save(filename, 'replay_data', '-v7.3');
    
    file_info = dir(filename);
    fprintf('\n3D Replay data saved: %s\n', filename);
    fprintf('File size: %.2f MB\n', file_info.bytes / (1024*1024));
    fprintf('Total frames: %d\n', actual_iter);
    fprintf('To replay with EXACT same visualization: replay_uav_simulation(''%s'')\n', filename);
end
