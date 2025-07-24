function VN = voronoi_neighbors(i, current_pos)
        N = numel(current_pos);
        pos_i = current_pos{i}; VN = [];
        for j=1:N
            if j==i, continue; end
            mid = (pos_i+current_pos{j})/2;
            if norm(pos_i-mid)<norm(current_pos{j}-mid)
                VN(end+1)=j;
            end
        end
    end
    