   function [assigned_goal, theta, requests,params] = alokasi_tugasnew(i, current_pos, assigned_goal, theta, neighbors, target, params, requests,done)
        
    
    % Input:
    %   i               : indeks agen
    %   current_pos     : cell array posisi {1..N}
    %   assigned_goal   : 1xN, tujuan tiap agen
    %   theta           : 1xN, prioritas unik
    %   neighbors       : cell array list tetangga per agen
    %   target          : cell array posisi target {1..M}
    %   params          : struct R_STOP, R_SWITCH, R_ATTR, dtheta, delta, comm_range, timeout
    %   requests        : struct berisi:
    %       sent{i}.to   : list permintaan kirim
    %       in{i}        : incoming request FIFO
    %       ack{i}       : incoming ack FIFO
    %       timer{i}{j}  : timestamp request i->j
    if done(i)
        return;
    end
    
    % 1) Hitung neighbor Voronoi lokal via half-space
    neighbors{i} = voronoi_neighbors(i, current_pos);
    pos_i = current_pos{i};
    g_i   = assigned_goal(i);
    cnt = histcounts(assigned_goal, 1:(numel(target)+1));
    
    % 2) Proses timeout request
    now = tic;
    for j = requests.sent(i).to
        if now - requests.timer{i}{j} > params.timeout
            % batal request yg timeout
            requests.sent(i).to(requests.sent(i).to==j) = [];
        end
    end
    
    % 3) Proses incoming requests (FIFO)
    while ~isempty(requests.in{i})
        sender = requests.in{i}(1);
        requests.in{i}(1) = [];
        % Rule 2/3: jika sender block dan tujuan berbeda
        if theta(sender)<theta(i) && assigned_goal(sender)~=g_i
            requests.ack{i}(end+1)=sender;  % positif
        else
            requests.nack{i}(end+1)=sender; % negatif
        end
    end
    
    % 4) Ekseskusi swap jika ada ACK sesuai FIFO
    while ~isempty(requests.ack{i})
        j = requests.ack{i}(1);
        requests.ack{i}(1)=[];
        % swap assigned_goal dan update theta
        tmp = assigned_goal(i);
        assigned_goal(i) = assigned_goal(j);
        assigned_goal(j) = tmp;
        theta(i) = theta(i) + params.dtheta;
    end
    
    % 5) Inisiasi request baru ke blockers (B+ dan B++)
    [Bp, Bpp] = classify_blockers(i, current_pos, neighbors{i}, target{g_i}, params);
    candidates = [Bp, Bpp];
    for j = candidates
        d_ij = norm(pos_i - current_pos{j});
        if d_ij<params.R_SWITCH && d_ij>params.R_STOP
            
            wz = 1;  
            diff_i = pos_i   - target{g_i};
            diff_j = current_pos{j} - target{g_i};
            cost_i = norm([diff_i(1:2); wz*diff_i(3)]);
            cost_j = norm([diff_j(1:2); wz*diff_j(3)]);
            
            % Softmax‐compare
            pi = exp(-params.Kcost * cost_i);
            pj = exp(-params.Kcost * cost_j);
            if pj > pi
                % kirim request i->j jika j lebih "berhak"  
                requests.sent(i).to(end+1) = j;
                requests.timer{i}{j}      = now;
                requests.in{j}(end+1)     = i;
            end
            % ==========================================================
            
        end
    end
    
    
    % ————————————————————————————————
    N    = numel(current_pos);
    M    = numel(target);
    counts = histcounts(assigned_goal, 1:(M+1));  % vektor 1×M
    
    % 6) Rule 1+: stop hanya kalau belum penuh kuota
    if norm(pos_i - target{g_i}) < params.R_STOP && counts(g_i) < params.quota(g_i)
        % hanya hold kalau target g_i belum mencapai quota(g_i)
        return;
    end
    % ————————————————————————————————
    
    
    % 7) Deadlock resolution sesuai paper
    % 7) Deadlock resolution sesuai paper
    G = build_request_graph(requests.sent);
    [found, cycle] = detect_cycle(G);
    if found
        victim = cycle( find(theta(cycle)==max(theta(cycle)), 1) );
        
        % cek ada alternatif goal?
        alt_goals = setdiff(1:numel(target), assigned_goal(victim));
        if isempty(alt_goals)
            return;   % no alternative → skip swap
        end
    
        % kalau ada, hitung cost dan swap
        costs = arrayfun(@(t) norm(current_pos{victim}-target{t}), alt_goals);
        [~, m] = min(costs);      
        assigned_goal(victim) = alt_goals(m);
        theta(victim)     = theta(victim) + params.dtheta;
    end
    
    % setelah semua swap / deadlock resolution selesai
    params.assigngoal(end+1, :) = assigned_goal;
    
    end
    