%% Helper Functions
function Fatt = potential_attraction(Katt, current_pos, goal)
    % Calculate attractive potential field force
    Fatt = Katt * (goal - current_pos);
end

