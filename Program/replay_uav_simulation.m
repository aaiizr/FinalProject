function replay_uav_simulation(filename)
    % Replay with EXACT same visualization as original simulation
    
    if nargin < 1
        [file, path] = uigetfile('UAV_3D_Replay_*.mat', 'Select 3D Replay File');
        if isequal(file, 0)
            return;
        end
        filename = fullfile(path, file);
    end
    
    fprintf('Loading 3D replay data: %s\n', filename);
    load(filename, 'replay_data');
    
    % Create new figure for replay - EXACT same setup as original
    replay_fig = figure('Name', sprintf('3D Replay: %s - %s', ...
        replay_data.settings.apf_name, replay_data.settings.mode_str), ...
        'Position', [100, 100, 1400, 900]);
    
    % Setup environment EXACTLY like original
    setup_exact_environment(replay_data);
    
    % Create control panel
    create_replay_controls_exact(replay_fig, replay_data);
    
    fprintf('3D Replay loaded with EXACT original visualization!\n');
    fprintf('- Same quadcopter graphics for agents\n');
    fprintf('- Same cylinder/obstacle graphics\n');
    fprintf('- Same trajectory colors and styles\n');
    fprintf('- Same target markers and labels\n');
end
