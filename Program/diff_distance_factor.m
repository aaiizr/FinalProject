function diff_factor = diff_distance_factor(current_pos, goal, n, flag)
    % Derivative of distance factor
    d_goal = norm(goal - current_pos);
    if d_goal > eps
        diff_factor = n * d_goal^(n-1) * (goal - current_pos) / d_goal;
    else
        diff_factor = zeros(size(current_pos));
    end
end
    