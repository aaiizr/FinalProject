function replay_latest_uav()
    files = dir('UAV_3D_Replay_*.mat');
    if isempty(files)
        fprintf('No UAV replay files found!\n');
        return;
    end
    
    [~, idx] = max([files.datenum]);
    latest_file = files(idx).name;
    
    fprintf('Loading latest UAV replay: %s\n', latest_file);
    replay_uav_simulation(latest_file);
end
 