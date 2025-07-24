function [Bp, Bpp] = classify_blockers(i, current_pos, neigh, goal_i, params)
        pos_i = current_pos{i};
        dir   = (goal_i - pos_i) / norm(goal_i - pos_i);
        Bp = []; Bpp = [];
        for j = neigh
            v = current_pos{j} - pos_i;
            angle = acos( dot(v,dir) / (norm(v)*norm(dir) + eps) );
            if angle < pi/2
                Bp(end+1) = j;
            else
                Bpp(end+1) = j;
            end
        end
    end