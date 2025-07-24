function create_replay_controls_exact(fig, replay_data)
    % Create control panel for replay with EXACT same graphics
    
    % Control panel
    panel = uipanel('Parent', fig, 'Title', 'Replay Controls', ...
                   'Position', [0.02, 0.02, 0.96, 0.15]);
    
    % Current frame
    current_frame = 1;
    max_frames = replay_data.actual_iterations;
    
    % Control elements
    frame_slider = uicontrol('Parent', panel, 'Style', 'slider', ...
                            'Position', [50, 60, 300, 20], ...
                            'Min', 1, 'Max', max_frames, 'Value', 1, ...
                            'SliderStep', [1/max_frames, 10/max_frames]);
    
    frame_text = uicontrol('Parent', panel, 'Style', 'text', ...
                          'Position', [50, 35, 100, 20], ...
                          'String', sprintf('Frame: %d/%d', 1, max_frames));
    
    time_text = uicontrol('Parent', panel, 'Style', 'text', ...
                         'Position', [160, 35, 100, 20], ...
                         'String', sprintf('Time: %.2fs', 0));
    
    play_button = uicontrol('Parent', panel, 'Style', 'pushbutton', ...
                           'Position', [370, 50, 60, 30], ...
                           'String', 'Play', 'FontWeight', 'bold');
    
    uicontrol('Parent', panel, 'Style', 'text', ...
             'Position', [450, 65, 50, 15], 'String', 'Speed:');
    speed_slider = uicontrol('Parent', panel, 'Style', 'slider', ...
                            'Position', [450, 45, 100, 20], ...
                            'Min', 0.1, 'Max', 5, 'Value', 1);
    
    uicontrol('Parent', panel, 'Style', 'pushbutton', ...
             'Position', [570, 50, 60, 30], ...
             'String', 'Reset', ...
             'Callback', @(~,~) reset_animation());
    
    % Initialize graphics - EXACT same as your original
    agent_handles = cell(replay_data.settings.N, 1);
    agent_text_handles = cell(replay_data.settings.N, 1);
    trajectory_handles = cell(replay_data.settings.N, 1);
    obstacle_handles = [];
    obstacle_circle_handles = [];
    
    % Create agent quadcopters - using YOUR exact function
    cmap = replay_data.static_elements.colors;
    for j = 1:replay_data.settings.N
        initial_pos = squeeze(replay_data.dynamic.agent_positions(j,:,1));
        initial_assignment = replay_data.dynamic.agent_assignments(j,1);
        
        if initial_assignment > 0 && initial_assignment <= size(cmap,1)
            col = cmap(initial_assignment,:);
        else
            col = [0 0 1];
        end
        
        % Use YOUR exact quadcopter creation function
        agent_handles{j} = create_quadcopter_graphics(initial_pos, zeros(3,1), col, 1.5);
        
        % Agent text labels - exact same
        agent_text_handles{j} = text(initial_pos(1)-1, initial_pos(2)-1, initial_pos(3)-1, ...
                                    sprintf('%d', j), 'FontSize', 10, 'FontWeight', 'bold');
        
        % Initialize trajectory handles
        trajectory_handles{j} = [];
    end
    
    % Create dynamic obstacle quadcopters if needed
    if strcmp(replay_data.static_elements.obstacles.type, 'dynamic') || replay_data.settings.mixed == 1
        M_obs = replay_data.settings.M_obs;
        if isfield(replay_data.dynamic, 'obstacle_positions') && M_obs > 0
            obstacle_handles = cell(M_obs, 1);
            obstacle_circle_handles = gobjects(M_obs, 1);
            
            for k = 1:M_obs
                initial_pos = squeeze(replay_data.dynamic.obstacle_positions(k,:,1));
                initial_angles = squeeze(replay_data.dynamic.obstacle_orientations(k,:,1));
                
                % Dynamic obstacle quadcopter - YOUR exact function
                obstacle_handles{k} = create_quadcopter_graphics(initial_pos, initial_angles, [1 0 0], 2);
                
                % Safety circle - YOUR exact code
                r_dyn = 6;  % Same as your code
                theta_circle = linspace(0, 2*pi, 50);
                circle_x = initial_pos(1) + r_dyn * cos(theta_circle);
                circle_y = initial_pos(2) + r_dyn * sin(theta_circle);
                circle_z = initial_pos(3) * ones(size(theta_circle));
                obstacle_circle_handles(k) = plot3(circle_x, circle_y, circle_z, 'r--', 'LineWidth', 1.5);
            end
        end
    end
    
    % Animation variables
    is_playing = false;
    timer_obj = [];
    
    % Update function - maintains EXACT same visualization
    function update_frame(frame_num)
        frame_num = round(max(1, min(frame_num, max_frames)));
        current_frame = frame_num;
        
        % Update UI
        set(frame_slider, 'Value', frame_num);
        set(frame_text, 'String', sprintf('Frame: %d/%d', frame_num, max_frames));
        set(time_text, 'String', sprintf('Time: %.2fs', (frame_num-1) * replay_data.settings.dt));
        
        % Update agents - EXACT same as your original
        for j = 1:replay_data.settings.N
            pos = squeeze(replay_data.dynamic.agent_positions(j,:,frame_num));
            angles = squeeze(replay_data.dynamic.agent_orientations(j,:,frame_num));
            assignment = replay_data.dynamic.agent_assignments(j, frame_num);
            is_done = replay_data.dynamic.agent_status(j, frame_num);
            is_crashed = replay_data.dynamic.agent_crashed(j, frame_num);
            
            % Color based on assignment - EXACT same logic
            if assignment > 0 && assignment <= size(cmap,1)
                col = cmap(assignment,:);
            else
                col = [0 0 1];
            end
            
            % Update quadcopter - YOUR exact function
            if isvalid(agent_handles{j})
                update_quadcopter_graphics(agent_handles{j}, pos, angles);
            end
            
            % Update text position - EXACT same
            set(agent_text_handles{j}, 'Position', pos - [1;1;1]);
            
            % Update trajectories with color changes - EXACT same as your code
            if frame_num > 1
                prev_assignment = replay_data.dynamic.agent_assignments(j, frame_num-1);
                if assignment ~= prev_assignment || isempty(trajectory_handles{j})
                    % New trajectory segment
                    traj_data = squeeze(replay_data.dynamic.agent_positions(j,:,1:frame_num));
                    if assignment > 0 && assignment <= size(cmap,1)
                        traj_col = cmap(assignment,:);
                    else
                        traj_col = [0.5 0.5 0.5];
                    end
                    
                    new_line = plot3(traj_data(1,:), traj_data(2,:), traj_data(3,:), ...
                                   'LineWidth', 1.5, 'Color', traj_col);
                    
                    if isempty(trajectory_handles{j})
                        trajectory_handles{j} = new_line;
                    else
                        trajectory_handles{j}(end+1) = new_line;
                    end
                end
            end
        end
        
        % Update dynamic obstacles - EXACT same as your original
        % Update dynamic obstacles with variable count
if ~isempty(obstacle_handles) || (static == 0)
    % Get number of obstacles for this frame
    if isfield(replay_data.dynamic, 'num_obstacles_per_frame')
        current_num_obstacles = replay_data.dynamic.num_obstacles_per_frame(frame_num);
    else
        current_num_obstacles = size(replay_data.dynamic.obstacle_positions, 1);
    end
    
    % Expand obstacle handles if needed
    while length(obstacle_handles) < current_num_obstacles
        k = length(obstacle_handles) + 1;
        
        % Create new obstacle graphics
        initial_pos = squeeze(replay_data.dynamic.obstacle_positions(k,:,frame_num));
        initial_angles = squeeze(replay_data.dynamic.obstacle_orientations(k,:,frame_num));
        
        if size(initial_pos, 1) == 1
            initial_pos = initial_pos';
        end
        if size(initial_angles, 1) == 1
            initial_angles = initial_angles';
        end
        
        % Create new obstacle
        obstacle_handles{k} = create_quadcopter_graphics(initial_pos, initial_angles, [1 0 0], 2);
        
        % Create safety circle
        r_dyn = 6;
        theta_circle = linspace(0, 2*pi, 50);
        circle_x = initial_pos(1) + r_dyn * cos(theta_circle);
        circle_y = initial_pos(2) + r_dyn * sin(theta_circle);
        circle_z = initial_pos(3) * ones(size(theta_circle));
        
        if length(obstacle_circle_handles) < k
            obstacle_circle_handles(k) = plot3(circle_x, circle_y, circle_z, 'r--', 'LineWidth', 1.5);
        end
    end
    
    % Update all current obstacles
    for k = 1:current_num_obstacles
        if k <= length(obstacle_handles) && k <= size(replay_data.dynamic.obstacle_positions, 1)
            pos = squeeze(replay_data.dynamic.obstacle_positions(k,:,frame_num));
            angles = squeeze(replay_data.dynamic.obstacle_orientations(k,:,frame_num));
            
            if size(pos, 1) == 1
                pos = pos';
            end
            if size(angles, 1) == 1
                angles = angles';
            end
            
            % Hide obstacles that don't exist yet (all zeros)
            if all(pos == 0)
                if isvalid(obstacle_handles{k})
                    set(obstacle_handles{k}, 'Visible', 'off');
                end
                if k <= length(obstacle_circle_handles) && isvalid(obstacle_circle_handles(k))
                    set(obstacle_circle_handles(k), 'Visible', 'off');
                end
                continue;
            else
                if isvalid(obstacle_handles{k})
                    set(obstacle_handles{k}, 'Visible', 'on');
                end
                if k <= length(obstacle_circle_handles) && isvalid(obstacle_circle_handles(k))
                    set(obstacle_circle_handles(k), 'Visible', 'on');
                end
            end
            
            % Update obstacle quadcopter
            if isvalid(obstacle_handles{k})
                update_quadcopter_graphics(obstacle_handles{k}, pos, angles);
            end
            
            % Update safety circle
            if k <= length(obstacle_circle_handles) && isvalid(obstacle_circle_handles(k))
                r_dyn = 6;
                theta_circle = linspace(0, 2*pi, 50);
                circle_x = pos(1) + r_dyn * cos(theta_circle);
                circle_y = pos(2) + r_dyn * sin(theta_circle);
                circle_z = pos(3) * ones(size(theta_circle));
                set(obstacle_circle_handles(k), 'XData', circle_x, 'YData', circle_y, 'ZData', circle_z);
            end
        end
    end
end
        
        % drawnow;
    end
    
    % Play/pause and other control functions (same as before)
    function play_pause_callback(~, ~)
        if is_playing
            if ~isempty(timer_obj) && isvalid(timer_obj)
                stop(timer_obj);
                delete(timer_obj);
            end
            set(play_button, 'String', 'Play');
            is_playing = false;
        else
            speed = get(speed_slider, 'Value');
            timer_period = 0.05 / speed;
            
            timer_obj = timer('Period', timer_period, 'ExecutionMode', 'fixedRate', ...
                             'TimerFcn', @(~,~) advance_frame());
            start(timer_obj);
            set(play_button, 'String', 'Pause');
            is_playing = true;
        end
    end
    
    function advance_frame()
        if current_frame < max_frames
            update_frame(current_frame + 1);
        else
            if ~isempty(timer_obj) && isvalid(timer_obj)
                stop(timer_obj);
                delete(timer_obj);
            end
            set(play_button, 'String', 'Play');
            is_playing = false;
        end
    end
    
    function reset_animation()
        if is_playing
            play_pause_callback([], []);
        end
        update_frame(1);
    end
    
    % Set callbacks
    set(frame_slider, 'Callback', @(src,~) update_frame(get(src, 'Value')));
    set(play_button, 'Callback', @play_pause_callback);
    
    % Initial update
    update_frame(1);
end
