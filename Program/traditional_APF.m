 %% Traditional APF Implementation
    function [Ft, Fatt, Frep_total] = traditional_APF(j, target, pos, static, goal, all_obs, all_radius, gains_att, gains_rep, params)
        % Traditional Artificial Potential Field
        % Simple attractive force to goal and repulsive forces from obstacles
        
        %% Attractive Force
        d_goal = norm(goal - pos);
        if d_goal > eps
            Fatt = gains_att.kp * (goal - pos) / d_goal;
            
            % Limit force near goal
            if d_goal < params.R_STOP * 3
                scale = d_goal / (params.R_STOP * 3);
                Fatt = Fatt * scale;
            end
        else
            Fatt = zeros(3,1);
        end
        
        %% Repulsive Force
        Frep_total = zeros(3,1);
        d0 = 10;  % Influence distance
        
        for k = 1:size(all_obs,2)
            p_obs = all_obs(:,k);
            r_obs = all_radius(k);
            
            if static == 1 && k <= size(all_obs,2)
                % Cylinder obstacle
                dx = pos(1:2) - p_obs(1:2);
                dist_2d = norm(dx);
                if dist_2d < d0 && pos(3) < p_obs(3)
                    if dist_2d < r_obs
                        dist_2d = 0.1;  % Avoid division by zero
                    end
                    
                    % Traditional repulsive force
                    Frep_k = gains_rep.krepp * (1/(dist_2d-r_obs) - 1/d0) * (1/(dist_2d-r_obs))^2;
                    if dist_2d > eps
                        Frep_k = Frep_k * [dx/dist_2d; 0];
                    else
                        Frep_k = [100; 100; 0];  % Emergency repulsion
                    end
                    Frep_total = Frep_total + Frep_k;
                end
            else
                % Sphere obstacle
                dist = norm(pos - p_obs) - r_obs;
                if dist < d0 && dist > 0
                    % Traditional repulsive force
                    Frep_k = gains_rep.krepp * (1/dist - 1/d0) * (1/dist)^2;
                    direction = (pos - p_obs) / norm(pos - p_obs);
                    Frep_total = Frep_total + Frep_k * direction;
                elseif dist <= 0
                    % Emergency repulsion
                    direction = (pos - p_obs);
                    if norm(direction) < eps
                        direction = [1; 0; 0];
                    else
                        direction = direction / norm(direction);
                    end
                    Frep_total = Frep_total + 100 * direction;
                end
            end
        end
        
        %% Total Force
        Ft = Fatt + Frep_total;
    end
    