function flag = near_obstacle_du2019(p, all_obs, rad, params_dist, v, v_obs_all)
        flag = false;
        for k = 1:size(all_obs,2)
            p_obs = all_obs(:,k);
            if k <= size(v_obs_all,2)
                v_obs = v_obs_all(:,k);
            else
                v_obs = zeros(3,1);  % Static obstacle
            end
            r_obs = rad(k);
            
            % Compute safety distance for this obstacle
            rsafe = compute_safety_distance_du2019(p, v, p_obs, v_obs, r_obs, params_dist);
            
            % Check actual distance
            dist = norm(p - p_obs) - r_obs;  % Distance to surface
            
            if dist < rsafe
                flag = true;
                return;
            end
        end
    end