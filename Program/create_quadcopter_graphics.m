  function hgroup = create_quadcopter_graphics(pos, angles, color, scale)
        if nargin < 4
            scale = 1;
        end
        
        % Quadcopter parameters
        arm_length = 0.2 * scale;
        body_radius = 0.4 * scale;
        prop_radius = 0.2 * scale;
        
        % Create graphics group
        hgroup = hgtransform('Parent', gca);
        
        % Body (central sphere)
        [xs, ys, zs] = sphere(10);
        body = surf(xs*body_radius, ys*body_radius, zs*body_radius, ...
                    'Parent', hgroup, ...
                    'FaceColor', color, ...
                    'EdgeColor', 'none', ...
                    'FaceAlpha', 0.8);
        
        % Arms and propellers
        angles_arm = [45, 135, 225, 315] * pi/180;
        for i = 1:4
            % Arm
            x_arm = [0, arm_length*cos(angles_arm(i))];
            y_arm = [0, arm_length*sin(angles_arm(i))];
            z_arm = [0, 0];
            plot3(x_arm, y_arm, z_arm, 'k-', 'LineWidth', 2*scale, 'Parent', hgroup);
            
            % Propeller circle
            theta_prop = linspace(0, 2*pi, 20);
            x_prop = arm_length*cos(angles_arm(i)) + prop_radius*cos(theta_prop);
            y_prop = arm_length*sin(angles_arm(i)) + prop_radius*sin(theta_prop);
            z_prop = zeros(size(theta_prop)) + 0.05*scale;
            fill3(x_prop, y_prop, z_prop, [0.3 0.3 0.3], ...
                  'Parent', hgroup, 'FaceAlpha', 0.5, 'EdgeColor', 'none');
        end
        
        % Direction indicator (front)
        plot3([0, arm_length*1.2], [0, 0], [0, 0], 'r-', ...
              'LineWidth', 3*scale, 'Parent', hgroup);
        
        % Apply position and rotation
        update_quadcopter_graphics(hgroup, pos, angles);
    end
    