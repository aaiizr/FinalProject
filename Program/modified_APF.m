    function [Ft, Fatt, Frep_total] = modified_APF(j, target, pos, static, goal, vcur, all_obs, all_radius, gains_att, gains_rep, params, desired_speed)
% Modified APF with multi-robot avoidance implementation
% Mapping input parameters to new implementation variables
current_pos = pos;
obstacles = all_obs;
radius = all_radius;
Katt = gains_att.kp; % Use position gain as attractive gain
Krep = gains_rep.krepp; % Use repulsive gain
Kform = 0; % Formation gain (default value)

% Initialize variables
goalp = goal;
robot_height = current_pos(3,1);
goal_height = goal(3,1);
flag = 0;

% Initialize force matrix
F = zeros(3, size(obstacles,2) + 10); % Extra space for multi-agent forces

%% Attractive Force
Fatt = potential_attraction(Katt, current_pos, goal);

%% Repulsive Forces from Static Obstacles
for k = 1: length(obstacles(1,:))
    % Measuring the horizontal distance between UAV and centre axis of the building
    rou = sqrt((current_pos(1,1)-obstacles(1,k))^2+(current_pos(2,1)-obstacles(2,k))^2)-radius(k,1);
    rou3d = sqrt((current_pos(1,1)-obstacles(1,k))^2+(current_pos(2,1)-obstacles(2,k))^2+(current_pos(3,1)-obstacles(3,k))^2);
    
    % differentiation of variable rou
    d_rou = [current_pos(1,1)-obstacles(1,k); current_pos(2,1)-obstacles(2,k)]/(rou+radius(k,1));
    
    % Threshold value to judge whether the UAV near to building or not?
    zeta = max(3*radius(k,1),5);
    n = 2;
    
    if rou<=0
        if static == 1
            if robot_height <= obstacles(3,k)
                disp("Crash Detected");
                F(:,k)=0;
            elseif robot_height <= obstacles(3,k)+zeta
                F(:,k) = (n/2)*Krep*(1/(robot_height-obstacles(3,k))-1/zeta)^2*dist_factor(current_pos, goal, n-1, flag)*diff_distance_factor(current_pos, goal, n, flag);
            else
                F(:,k)=0;
            end
        elseif rou3d <= radius(k,1)
            disp("Crash Detected");
            F(:,k)=0;
        else
            F(:,k)=0;
        end
    elseif rou<=zeta
        if robot_height <= obstacles(3,k)
            % Flag - that tells the UAV to move in xy plane
            % no increment in height.
            Frep1 = Krep*abs((1/rou)-(1/zeta))*d_rou;
            F(:,k) = vertcat(Frep1,0);
            F(3,k)=0;
        else
            F(:,k) = 0;
        end
    elseif rou > zeta
        F(:,k) = 0;
    end
end

%% Multi-Agent Avoidance (if other agents exist)
% For now, using empty agenlain and group since not provided in original parameters
agenlain = {}; % This would need to be passed as parameter for multi-agent scenarios
group = []; % This would need to be passed as parameter for formation control

[~,nkk]=size(agenlain);
for kk = 1: nkk
    % Measuring the horizontal distance between UAV and the other agent
    rou = sqrt((current_pos(1,1)-agenlain{kk}(1))^2+(current_pos(2,1)-agenlain{kk}(2))^2 ...
        +(current_pos(3,1)-agenlain{kk}(3))^2);
    
    % differentiation of variable rou
    d_rou = [current_pos(1,1)-agenlain{kk}(1); current_pos(2,1)-agenlain{kk}(2); ...
        current_pos(3,1)-agenlain{kk}(3)]/rou;
    
    % Threshold value to judge whether the UAV near to other agent or not?
    zeta = 1;
    n = 2;
    
    if rou<=zeta
        if robot_height > agenlain{kk}(3)
            goal=goalp;
            % Inter-agent repulsive force
            Frep1 = Kform*Krep*((1/rou)-(1/zeta))*(1/rou^2)*dist_factor(current_pos, goal, n, flag)*d_rou;
            F(:,k+kk) = Frep1;
            F(3,k+kk) = 0;
        else
            F(:,k+kk) = 0;
        end
    elseif (rou > zeta) && (ismember(kk, group))
        dd=[(current_pos(1,1)-agenlain{kk}(1));(current_pos(2,1)-agenlain{kk}(2))];
        goal=agenlain{kk};
        F(:,k+kk)=vertcat(Katt*Kform*((1/rou)-(1/zeta))*dd/norm(dd),0);
        F(3,k+kk) = 0;
    end
end

%% Calculate Total Forces
Frep_total = sum(F,2); % summation of all repulsive forces
Ft = Fatt + Frep_total*norm(Fatt);

end

