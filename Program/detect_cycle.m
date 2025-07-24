function [found,cycle] = detect_cycle(G)
        N=numel(G);
        vis=false(1,N); st=false(1,N);
        for v=1:N
            [found,cycle]=dfs(v,vis,st,[],G);
            if found, return; end
        end
        found=false; cycle=[];
    end