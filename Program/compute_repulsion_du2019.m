 function Frep = compute_repulsion_du2019(p, v, p_obs, v_obs, r_obs, h_obs, params_dist, gains_rep, static)
        % Initialize
        Frep = zeros(3,1);
        
        % For static obstacles, check if UAV is above obstacle height
        if static == 1 && p(3) > h_obs
            return;  % No repulsion if well above obstacle
        end
        
        % Relative velocity and position
        Ev = v - v_obs;              % Relative velocity
        Ed = p_obs - p;              % Relative position (from UAV to obstacle)
        
        % Distance to obstacle surface
        if static == 1
            % Cylinder: project to nearest point on cylinder surface
            dx = p(1:2) - p_obs(1:2);
            d_horiz = norm(dx);
            if d_horiz > eps
                n_horiz = dx / d_horiz;
            else
                n_horiz = [1; 1];
            end
            q_xy = p_obs(1:2) + n_horiz * r_obs;
            z_clamp = min(max(p(3), 0), h_obs);
            q = [q_xy; z_clamp];
            dE_actual = norm(p - q);
        else
            % Sphere: distance to center minus radius
            dE_actual = norm(Ed) - r_obs;
            dE_actual = max(dE_actual, eps);  % Avoid negative distances
        end
        
        % Safety distance
        rsafe = compute_safety_distance_du2019(p, v, p_obs, v_obs, r_obs, params_dist);
        % Check if within safety distance
        if dE_actual >= rsafe
            return;  % No repulsion needed
        end
        
        % Norms
        norm_Ev = norm(Ev) + eps;
        norm_Ed = norm(Ed) + eps;
        
        % Angle between relative velocity and position (δ)
        cos_delta = dot(Ev, Ed) / (norm_Ev * norm_Ed);
        cos_delta = max(-1, min(1, cos_delta));
        delta = acos(cos_delta);
        
        % Threat Level (Eq. 12)
        
        if cos_delta > 0
            if static == 0
            TH_level = (1/dE_actual - 1/rsafe) * norm_Ev * cos_delta;
            else
            TH_level = (1/dE_actual - 1/rsafe) * norm_Ev *cos_delta;
            end
        else
            TH_level = 0;
        end
        
        % Unit vector from UAV to obstacle
        e_dE = Ed / norm_Ed;
        
        % Position Repulsion (Eq. 13)
        if dE_actual < rsafe && delta >= 0 && delta <= pi/2
            Fp_rep = gains_rep.krepp * TH_level * (-e_dE);
        else
            Fp_rep = zeros(3,1);
        end
        
        % Speed Repulsion (Eq. 14) 
        if dE_actual < rsafe && delta > pi/2 && delta < 3*pi/2
            e_v = Ev / norm_Ev;
            % The perpendicular component from Eq. 14
            if abs(cos_delta) > eps
                perp = -e_dE * (1/cos_delta) + e_v;
            else
                % When cos(δ) ≈ 0, use simplified perpendicular
                perp = e_v;
            end
            perp_norm = norm(perp) + eps;
            perp = perp / perp_norm;
            
            Fv_rep = gains_rep.krepv * TH_level * perp;
        else
            Fv_rep = zeros(3,1);
        end
        
        % Total repulsion (Eq. 15)
        Frep = Fp_rep + Fv_rep;
        
        % For static obstacles, optionally remove vertical component
        if static == 1 && norm(v_obs) < eps
            Frep(3) = 0;
        end
    end
  