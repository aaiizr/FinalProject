function setup_exact_environment(replay_data)
    % Setup EXACTLY like your original simulation
    
    % Floor - exact same as your code
    fill3([0 200 200 0],[0 0 200 200],[0 0 0 0],[0.3 0.3 0.3]);
    hold on; grid on;
    
    % Axes - exact same
    xlabel("x"); ylabel("y"); zlabel("z");
    xlim([0 200]); ylim([0 200]); zlim([0 100]);
    
    % Use your exact view settings
    view(replay_data.settings.xyp, replay_data.settings.zp);
    
    % Targets - EXACT same as your code
    colors = replay_data.static_elements.target_colors;
    for t = 1:replay_data.settings.M
        target_pos = replay_data.static_elements.targets{t};
        
        plot3(target_pos(1), target_pos(2), target_pos(3), 'o', ...
              'MarkerSize', 15, 'MarkerFaceColor', colors(t,:), ...
              'MarkerEdgeColor', 'black', 'LineWidth', 2);
        
        % Target label - exact same
        text(target_pos(1)+5, target_pos(2)+5, target_pos(3)+5, ...
             sprintf('Target %d', t), 'FontSize', 12, 'FontWeight', 'bold', ...
             'Color', colors(t,:), 'BackgroundColor', 'white', ...
             'EdgeColor', 'black');
    end
    
    % Obstacles - EXACT same as your original code
    if strcmp(replay_data.static_elements.obstacles.type, 'static')
        % Static obstacles (cylinders) - using your exact function
        Cpos = replay_data.static_elements.obstacles.Cpos;
        radius = replay_data.static_elements.obstacles.radii;
        
        for i = 1:size(Cpos, 1)
            create_cylinder(radius(i), Cpos(i,:), [0.45, 0.58, 0.76]);
        end
        
    elseif replay_data.settings.mixed == 1
        % Mixed mode - static cylinders only (dynamic obstacles animated separately)
        if isfield(replay_data.static_elements.obstacles, 'static_indices')
            static_indices = replay_data.static_elements.obstacles.static_indices;
            Cpos = replay_data.static_elements.obstacles.Cpos;
            radius = replay_data.static_elements.obstacles.radii;
            
            for idx = 1:length(static_indices)
                i = static_indices(idx);
                create_cylinder(radius(idx), Cpos(idx,:), [0.45, 0.58, 0.76]);
            end
        end
    end
    
    title(sprintf('3D Replay: %s - %s', replay_data.settings.apf_name, replay_data.settings.mode_str));
end
