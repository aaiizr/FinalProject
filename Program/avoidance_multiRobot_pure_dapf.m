function [Ft, Fatt, Frep_total] = avoidance_multiRobot_pure_dapf( ...
    j, target, pos, static, goal, vcur, ...
    Kform, ...
    all_obs, all_radius, Fcache, agenlain, group, ...
    params, ...
    params_dist, gains_att, gains_rep, ...
    desired_speed, all_speeds, speed_agent, agent_radius, agent_pos, all_agent_goals)

%% 3) Attractive Force (Equations 17-21)
    d_goal = norm(goal - pos);
    r_min_safe = params_dist.rminsafe;
        kv = params_dist.kw;
        ka = params_dist.ka;
        wmax = params_dist.wmax;
        beta = params_dist.beta;
        theta1 = params_dist.theta1;
    
    % Position attractive force (Equation 18)
    if d_goal > eps
        F_att_p = gains_att.kp * (goal - pos);
    else
        F_att_p = zeros(3,1);
    end
    
    % Speed attractive force (Equation 19)
    Vo = desired_speed;
    if d_goal > eps
        desired_velocity = Vo * (goal - pos) / d_goal;
    else
        desired_velocity = zeros(3,1);
    end
    F_att_v = gains_att.kv * (desired_velocity - vcur);
    
    % Check if in collision avoidance mode
    in_collision_avoidance = false;
    for k = 1:size(all_obs,2)
        p_obs = all_obs(:,k);
        if k <= size(all_speeds, 2), v_obs = all_speeds(:,k); else, v_obs = zeros(3,1); end
        r_obs = all_radius(k);
        
        % Calculate rsafe for this check
        Ev = vcur - v_obs;
        dE = p_obs - pos;
        norm_Ev = norm(Ev) + eps;
        norm_dE = norm(dE) + eps;
        cos_delta = dot(Ev, dE) / (norm_Ev * norm_dE);
        cos_delta = max(-1, min(1, cos_delta));
        delta = acos(cos_delta);
        
        r_min_safe = params_dist.rminsafe;
        % Variable part of safety distance (Equation 7)
r_var_safe = r_min_safe + (kv/(ka + wmax)) * norm_Ev * cos(delta);

% Calculate connection part parameters for equation (8)
% Variable safety distance at boundary angle β
r_var_safe_at_beta = r_min_safe + (kv/(ka + wmax)) * norm_Ev * cos(beta);

% Connection part of safety distance (Equation 8)
% Based on geometric construction from Figure 1
OM_distance = r_var_safe_at_beta; % Distance at boundary angle β
angle_OQM = beta + theta1; % Based on the geometric construction shown in Figure 1

% Connection part of safety distance (Equation 8)
if sin(angle_OQM) ~= 0
    r_connection_safe = OM_distance * sin(beta - theta1) / sin(angle_OQM);
else
    r_connection_safe = r_min_safe; % Fallback for degenerate case
end

% Ensure r_connection_safe is within reasonable bounds
r_connection_safe = max(r_connection_safe, r_min_safe);
r_connection_safe = min(r_connection_safe, r_var_safe_at_beta);

% Determine rsafe based on angle δ (Equation 9)
if abs(delta) <= beta
    % Use variable safety distance
    rsafe = r_var_safe;
elseif abs(delta) > beta && abs(delta) < (pi/2 + theta1)
    % Connection region - use equation (8)
    rsafe = r_connection_safe;
else
    % Use minimum safety distance (abs(delta) >= pi/2 + theta1)
    rsafe = r_min_safe;
end

% Add obstacle radius to safety distance
rsafe = rsafe + r_obs;

        
        if static == 1
            dx = pos(1:2) - p_obs(1:2);
            dist_2d = norm(dx);
            if dist_2d < rsafe && pos(3) < p_obs(3)
                in_collision_avoidance = true;
                break;
            end
        else
            dist_to_obs = norm(pos - p_obs);
            if dist_to_obs < agent_radius*2
                in_collision_avoidance = true;
                break;
            end
        end
    end
    
    % Apply attractive force with damping (Equation 21)
    if in_collision_avoidance
        % During collision avoidance - no damping
        Fatt = F_att_p + F_att_v;
    else
        % Normal flight - add damping to prevent oscillation
        kdamp = gains_att.kdamp;
        damping_force = kdamp * vcur;
        Fatt = F_att_p + F_att_v - damping_force;
    end    
%% 1) Calculate Adaptive Safety Distance (Equation 9 from paper)
    % This is the key difference - proper adaptive safety distance calculation
    
    Frep_total = zeros(3,1);
    
    for k = 1:size(all_obs,2)
        p_obs = all_obs(:,k);
        v_obs = all_speeds(:,k);
        r_obs = all_radius(k);
        
        % Relative velocity and position (Equations 10-11)
        Ev = vcur - v_obs;                    % Relative velocity
        dE = p_obs - pos;                     % Relative position (from UAV to obstacle)
        
        % Calculate angle δ between relative velocity and position
        norm_Ev = norm(Ev) + eps;
        norm_dE = norm(dE) + eps;
        cos_delta = dot(Ev, dE) / (norm_Ev * norm_dE);
        cos_delta = max(-1, min(1, cos_delta));  % Clamp to [-1, 1]
        delta = acos(cos_delta);
        
        % Actual distance to obstacle
        if static == 1
            % Cylinder obstacle
            dx = pos(1:2) - p_obs(1:2);
            dist_2d = norm(dx);
            if dist_2d < r_obs && pos(3) < p_obs(3)
                dE_actual = 0.1;  % Very close
            else
                if pos(3) < p_obs(3)
                    dE_actual = dist_2d - r_obs;
                else
                    dE_actual = norm([dx; pos(3) - p_obs(3)]) - r_obs;
                end
            end
        else
            % Sphere obstacle
            dE_actual = norm(dE) - r_obs;
            dE_actual = max(dE_actual, 0.1);  % Avoid negative distances
        end
        
        % Calculate adaptive safety distance rsafe (Equation 9)
        
        
        % Variable part of safety distance (Equation 7)
r_var_safe = r_min_safe + (kv/(ka + wmax)) * norm_Ev * cos(delta);

% Calculate connection part parameters for equation (8)
% Variable safety distance at boundary angle β
r_var_safe_at_beta = r_min_safe + (kv/(ka + wmax)) * norm_Ev * cos(beta);

% Connection part of safety distance (Equation 8)
% Based on geometric construction from Figure 1
OM_distance = r_var_safe_at_beta; % Distance at boundary angle β
angle_OQM = beta + theta1; % Based on the geometric construction shown in Figure 1

% Connection part of safety distance (Equation 8)
if sin(angle_OQM) ~= 0
    r_connection_safe = OM_distance * sin(beta - theta1) / sin(angle_OQM);
else
    r_connection_safe = r_min_safe; % Fallback for degenerate case
end

% Ensure r_connection_safe is within reasonable bounds
r_connection_safe = max(r_connection_safe, r_min_safe);
r_connection_safe = min(r_connection_safe, r_var_safe_at_beta);

% Determine rsafe based on angle δ (Equation 9)
if abs(delta) <= beta
    % Use variable safety distance
    rsafe = r_var_safe;
elseif abs(delta) > beta && abs(delta) < (pi/2 + theta1)
    % Connection region - use equation (8)
    rsafe = r_connection_safe;
else
    % Use minimum safety distance (abs(delta) >= pi/2 + theta1)
    rsafe = r_min_safe;
end

% Add obstacle radius to safety distance
rsafe = rsafe+r_obs ;

        % Check if within safety distance
        if dE_actual >= rsafe
            continue;  % No repulsion needed
        end
        
        % Calculate threat level (Equation 12)
        if cos(delta) > 0
            TH_level = (1/dE_actual - 1/rsafe) * norm_Ev * cos(delta);
        else
            TH_level = 0;
        end
        
        % Position repulsion (Equation 13)
        if dE_actual < rsafe && delta >= 0 && delta <= pi/2
            F_p_rep = gains_rep.krepp * TH_level * (-dE / norm_dE);
        else
            F_p_rep = zeros(3,1);
        end
        
        % Speed repulsion (Equation 14)
        if dE_actual < rsafe && delta > pi/2 && delta < 3*pi/2
            if abs(cos(delta)) > eps
                perp_component = -dE/norm_dE * (1/cos(delta)) + Ev/norm_Ev;
            else
                perp_component = Ev/norm_Ev;
            end
            perp_norm = norm(perp_component) + eps;
            perp_component = perp_component / perp_norm;
            
            F_v_rep = gains_rep.krepv * TH_level * perp_component;
        else
            F_v_rep = zeros(3,1);
        end
        
        % Total repulsion from this obstacle (Equation 15)
        if static ==1
        F_rep_k = (F_p_rep + F_v_rep)*norm(Fatt);
        else 
            F_rep_k = (F_p_rep + F_v_rep);
        end

        
        % For static obstacles, remove vertical component
        if static == 1   
            F_rep_k(3) = 0;
        end
        

        if static == 1 && pos(3)> p_obs(3) + 2
            F_rep_k = zeros(3,1);
        end
        
        Frep_total = Frep_total + F_rep_k;
    end
    
     %% 2) Process other agents as obstacles via helper
   % Filter out stopped agents before calculating agent interactions
    active_agent_pos = [];
    active_speed_agent = [];
    
    if size(agent_pos, 2) > 0  % Check if there are other agents
        for i = 1:size(agent_pos, 2)
            % Calculate distance from each agent to its goal
            agent_goal = all_agent_goals(:, i);  % Get goal for agent i
            agent_position = agent_pos(:, i);    % Get position for agent i
            agent_to_goal_dist = norm(agent_goal - agent_position);
            
            % Only include agents that haven't stopped (distance to goal >= R_STOP)
            if agent_to_goal_dist >= params.R_STOP 
                active_agent_pos = [active_agent_pos, agent_position];
                active_speed_agent = [active_speed_agent, speed_agent(:, i)];
            end
        end
    end
    
    % Calculate agent interaction only with active (non-stopped) agents
    if ~isempty(active_agent_pos)
        F_agent = calculateAgentInteraction( ...
            pos,d_goal, vcur, active_agent_pos, active_speed_agent, params_dist, gains_rep, gains_att);
    else
        F_agent = zeros(3,1);  % No active agents to interact with
    end
    
    % Also prevent current agent from receiving F_agent if it has stopped
    if d_goal < 1.2* params.R_STOP
        F_agent = zeros(3,1);
    end
    F_agent(3) = 0;
    % F_agent = zeros(3,1);

    Frep_total = Frep_total + F_agent;
    % 
    
    
    %% 4) Total Force (Equation 22)
    Ft = Fatt + Frep_total;
    
    
end