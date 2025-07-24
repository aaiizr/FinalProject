function [found,cycle]=dfs(v,vis,st,path,G)
        vis(v)=true; st(v)=true; path(end+1)=v;
        for w=G{v}
            if ~vis(w)
                [found,cycle]=dfs(w,vis,st,path,G);
                if found, return; end
            elseif st(w)
                idx=find(path==w,1);
                cycle=path(idx:end); found=true; return;
            end
        end
        st(v)=false; found=false; cycle=[];
    end
    
    %% Revised avoidance_multiRobot function based on Du et al. 2019 paper
