function rsafe = compute_safety_distance_du2019(p, v, p_obs, v_obs, r_obs, params_dist)
        % Relative velocity and position
        Ev = v - v_obs;              % Relative velocity
        Ed = p_obs - p;              % Relative position (from UAV to obstacle)
        
        % Norms
        norm_Ev = norm(Ev) + eps;
        norm_Ed = norm(Ed) + eps;
        
        % Angle between relative velocity and position (δ in paper)
        cos_delta = dot(Ev, Ed) / (norm_Ev * norm_Ed);
        cos_delta = max(-1, min(1, cos_delta));  % Clamp to [-1, 1]
        delta = acos(cos_delta);
        
        % Parameters from paper
        r_min_safe = params_dist.rminsafe;  % Include obstacle radius
        kv = params_dist.kw;
        ka = params_dist.ka;
        wmax = params_dist.wmax;
        beta = params_dist.beta;         % β in paper 
        theta1 = params_dist.theta1;     % θ₁ in paper
        
        % Variable part (Eq. 7)
        r_var_safe = r_min_safe + (kv / (ka + wmax)) * norm_Ev * cos(delta);
        
        % Safety distance based on angle (Eq. 9)
        if abs(delta) >= 0 && abs(delta) < beta
            % Use variable safety distance
            rsafe = r_var_safe;
        elseif abs(delta) >= beta && abs(delta) <= (pi/2 + theta1)
            % Connection region - linear interpolation
            t = (abs(delta) - beta) / ((pi/2 + theta1) - beta);
            rsafe = (1 - t) * r_var_safe + t * r_min_safe;
        else
            % Use minimum safety distance
            rsafe = r_min_safe;
        end
    end