function agent_near_goal = check_agent_near_goal(current_pos, assigned_goal, target, threshold)
    agent_near_goal = false;
    for j = 1:length(current_pos)
        if ~isempty(current_pos{j}) && assigned_goal(j) > 0
            goal_pos = target{assigned_goal(j)};
            distance_to_goal = norm(current_pos{j} - goal_pos);
            if distance_to_goal < threshold
                agent_near_goal = true;
                return;
            end
        end
    end
end
