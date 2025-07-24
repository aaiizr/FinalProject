%% VAPF Implementation
    function [Ft, Fatt, Frep_total] = velocity_APF(j, target, pos, static, goal, vcur, all_obs, all_radius, obs_speeds, gains_att, gains_rep, params)
        % Velocity-based APF considering relative velocities
        
        %% Attractive Force
        d_goal = norm(goal - pos);
        
        if d_goal > eps
            % Basic attractive force
            Fatt = gains_att.kp * (goal - pos);
            
            % Damping to prevent oscillation
            Fatt = Fatt - gains_att.kdamp * vcur;
            
            % Limit force near goal
            if d_goal < params.R_STOP * 3
                scale = d_goal / (params.R_STOP * 3);
                Fatt = Fatt * scale;
            end
        else
            Fatt = zeros(3,1);
        end
        
        %% Velocity-based Repulsive Force
        Frep_total = zeros(3,1);
        d0 = 30;  % Influence distance
        collision_time_threshold = 5;  % seconds
        
        for k = 1:size(all_obs,2)
            p_obs = all_obs(:,k);
            r_obs = all_radius(k);
            v_obs = obs_speeds(:,k);
            
            % Relative position and velocity
            rel_pos = pos - p_obs;
            rel_vel = vcur - v_obs;
            
            if static == 1 && k <= size(all_obs,2)
                % Cylinder obstacle (static)
                dx = pos(1:2) - p_obs(1:2);
                dist_2d = norm(dx);
                
                if dist_2d < d0 && pos(3) < p_obs(3)
                    % Check collision course
                    if dist_2d > eps && dot(vcur(1:2), -dx) > 0
                        % Moving toward obstacle
                        time_to_collision = dist_2d / (norm(vcur(1:2)) + eps);
                        
                        if time_to_collision < collision_time_threshold
                            % Velocity-based repulsion
                            urgency = 1 - time_to_collision / collision_time_threshold;
                            Frep_k = gains_rep.krepp * urgency * (1/(dist_2d-r_obs))^2;
                            
                            if dist_2d > eps
                                Frep_k = Frep_k * [dx/dist_2d; 0];
                            else
                                Frep_k = [100; 100; 0];
                            end
                            Frep_total = Frep_total + Frep_k;
                        end
                    end
                end
            else
                % Sphere obstacle (possibly moving)
                dist = norm(rel_pos) - r_obs;
                
                if dist < d0 && dist > 0
                    % Check if on collision course
                    if norm(rel_vel) > eps && dot(rel_vel, -rel_pos) > 0
                        % Calculate time to closest approach
                        t_closest = -dot(rel_pos, rel_vel) / (norm(rel_vel)^2);
                        
                        if t_closest > 0 && t_closest < collision_time_threshold
                            % Predicted closest distance
                            closest_pos = rel_pos + t_closest * rel_vel;
                            closest_dist = norm(closest_pos) - r_obs;
                            
                            if closest_dist < r_obs
                                % Collision predicted
                                urgency = 1 - t_closest / collision_time_threshold;
                                Frep_k = gains_rep.krepp * urgency * (1/dist)^2;
                                
                                direction = rel_pos / norm(rel_pos);
                                Frep_total = Frep_total + Frep_k * direction;
                            end
                        end
                    end
                    
                    % Add basic repulsion for close obstacles
                    if dist < r_obs * 2
                        Frep_k = gains_rep.krepp * (1/dist - 1/d0) * (1/dist)^2;
                        direction = rel_pos / norm(rel_pos);
                        Frep_total = Frep_total + Frep_k * direction;
                    end
                elseif dist <= 0
                    % Emergency repulsion
                    direction = rel_pos;
                    if norm(direction) < eps
                        direction = [1; 0; 0];
                    else
                        direction = direction / norm(direction);
                    end
                    Frep_total = Frep_total + 150 * direction;
                end
            end
        end
        
        %% Total Force
        Ft = Fatt + Frep_total;
    end