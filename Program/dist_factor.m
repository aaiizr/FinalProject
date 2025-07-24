function factor = dist_factor(current_pos, goal, n, flag)
    % Distance factor for goal-aware repulsion
    d_goal = norm(goal - current_pos);
    if d_goal > eps
        factor = d_goal^n;
    else
        factor = 1;
    end
end

