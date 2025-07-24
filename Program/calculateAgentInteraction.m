function F_agent = calculateAgentInteraction( ...
    pos_uav, d_goal, vel_uav, agents_pos, agents_vel, params_dist, gains_rep, gains_att)
% calculateAgentInteraction  Compute repulsive or attractive force per agent
    F_agent = zeros(3,1);
    nAgents = size(agents_pos,2);
    r_safe_agent = 5;  % safe radius
    
       
    for a = 1:nAgents
        p_a   = agents_pos(:,a);
        v_a   = agents_vel(:,a);
        
        
        % relative vectors
        Ev   = vel_uav - v_a;
        dE   = p_a - pos_uav;
        normEv = norm(Ev) + eps;
        normdE = norm(dE) + eps;
        
        % distance from UAV surface to agent
        dE_actual = max(normdE - r_safe_agent, 0.1);
        
        % compute adaptive rsafe (simplified: variable or min)
        cos_d = dot(Ev,dE)/(normEv*normdE);
        cos_d = max(-1,min(1,cos_d));
        delta = acos(cos_d);
        
       
        
% Add obstacle radius to safety distance
rsafe = 1;
        
        % decide repulsion vs attraction
        if dE_actual < rsafe
            % --- repulsion ---
            if cos_d>0
                TH = (1/dE_actual - 1/rsafe)*normEv*cos_d;
            else
                TH = 0;
            end
            % position repulsion
            if delta>=0 && delta<=pi/2
                Fp = 15 * gains_rep.krepp * TH * (-dE / normdE);
            else
                Fp = zeros(3,1);
            end
            % speed repulsion
            if delta>pi/2 && delta<3*pi/2
                if abs(cos_d)>eps
                    perp = -dE/normdE*(1/cos_d) + Ev/normEv;
                else
                    perp = Ev/normEv;
                end
                perp = perp / (norm(perp)+eps);
                Fv = 15* gains_rep.krepv * TH * perp;
            else
                Fv = zeros(3,1);
            end
            F_int = Fp + Fv;
        else
            % --- attraction (weak) towards agent to maintain cohesion ---
            % F_int = gains_att.kp * (p_a - pos_uav);
            F_int = zeros(3,1);
        end

        if d_goal < 6
            F_int = zeros(3,1);
        end
        
        F_agent = F_agent + F_int;
        
    end
end