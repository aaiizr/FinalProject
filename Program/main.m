% Muhammad Faiz RAmadhan
   % Update 7/3/2025
current_time = datetime('now', 'Format', 'yyyy-MM-dd_HH-mm-ss');
log_filename = sprintf('simulation_log_%s.txt', char(current_time));

% Start logging ALL command window output
diary(log_filename);
diary on;

fprintf('🎬 SIMULATION STARTED: %s\n', char(datetime('now')));
fprintf('📝 Command window output being saved to: %s\n', log_filename);
fprintf('════════════════════════════════════════════════════════\n\n');

%% Environment code
 
    clf;
    close all;
    clear all;
    clc;
    
    %% simulation parameter
    iteration = 8000;      %iteration limit
    dt=0.05;                %time step
    Time=dt.*[1:iteration]; %time limit
    tpause=0.0001;          %pause duration after each iteration
    speed=2;                %camera orbit speed
    tic

    %rng generator rng(seed,'type'), change seed to randomly change building
    rng(1460,"v5uniform");
    randomize=1 %random or Fixed
    static = 1  % Set to 0 for dynamic obstacles
    mixed = 0 % Campur Dinamis Statis (Belum Bisa)
    APF_type = 4 % Try different algorithms (1-4)
    datang = 1 % Coba 1 -3
    % datang =1 obstacle dari depan
    % datang = 2 obstacles dari belakang 
    % datang = 3 obstacles dari samping
    centered = 0
    % posisi obstacle ditengah pastikan sama randomize =1
    
    % Display simulation settings
    apf_names = {'Traditional APF', 'Modified APF (MAPF)', 'Velocity APF (VAPF)', 'Dynamic APF (DAPF)'};
    mode_str = '';
    if static == 1
        mode_str = 'Static Obstacles';
    elseif mixed == 1
        mode_str = 'Mixed (Static + Dynamic) Obstacles';
    else
        mode_str = 'Dynamic Obstacles Only';
    end
    
    %% System Parameter
    % Path Planning gain
    Krep = 50;  % Gain Repulsive
    Katt = 1;
     % Gain Attractive
    Kform = 0; %Gain Formasi (Belum Dipake)
    Kcost = 1; %Cost
    alocfreq = 100; % Cek Switching Setiap 10 Iterasi
    targetrad = 5; % Radius Target
    desired_speed = 10; % Agent Speed
    obstacle_speed = 15;  % Obstacle speed
    %%
    
    %% Inisiasi Halangan 
    sped    = obstacle_speed;  
    radii   = 7;         % Radius dy
    % namic obstacle
    nObsVec = 15;  % Jumlah Halangan
    r_dyn = radii;
    M_obs = nObsVec;
    
   %% Parameter DAPF
    params_dist.rminsafe = 30;    % radius aman
    params_dist.kw      = 40; 
    params_dist.ka      = 10;
    params_dist.wmax    = 15;
    params_dist.beta    = pi/6;
    params_dist.theta1  = pi/12;
    params.Kcost = Kcost;
    params.MinDistance= targetrad
    params.STABILITY_DISTANCE = 15;    % Don't reassign if within this distance
    params.COMMITMENT_DISTANCE = 10;   % Fully committed if within this distance  
    params.MIN_REASSIGN_BENEFIT = 5;   % Only reassign if significantly beneficial
    
    % gains untuk attraction/repulsion formasi
    gains_att.kp    = Katt;
    gains_att.kv    = Katt;
    gains_att.kdamp = 0;  % Increased damping to prevent oscillation
    gains_rep.krepp = Krep;
    gains_rep.krepv = Krep;
    %% Agen Dan Target Inisiasi
    %Defining agent & target
    N=5;                    %number of agent 
    M=1;                    %target is 3
     final_target_assignment = zeros(1, N);  % Track final target for each agent
   
    for t = 1:M
        x_rand = 190 + (195 - 190) * rand;
        y_rand = 190 + (195 - 190) * rand;
        z_rand =  60 + ( 65 -  60) * rand;
        target{t} = [ x_rand; y_rand; z_rand ];
    end
    %% Formasi Belum Dipake
    % Define formation patterns relative to each target
    R = 2; % Formation radius
    f = cell(N,1);
    Delta = cell(N,1);  % Formation offsets for each agent
    for i=1:N
        angle = 2*pi*(i-1)/N;
        Delta{i} = [ R*cos(angle);
                     R*sin(angle);
                     0 ];  % Keep formation in same z-plane
        f{i} = Delta{i};  % Store for compatibility
    end
    
    d_des = cell(N,N);
    for i=1:N
      for j=1:N
        d_des{i,j} = f{j} - f{i};
      end
    end
    %% 
    %view angle view(xy plane, z plane) in degrees
    xyp=45;
    zp=90;
    view(xyp,zp)
    %% Resolusi Sphere
    if static ==0
    nRes = 10;
    [XS0, YS0, ZS0] = sphere(nRes);
    end
    %% Buildings Position
    
    % Inisialisasi posisi gedung (Cpos) – 50×3
Cpos = [
    80,80,100;
    120,130,80
    120,100,90
  % 1.573276287912332e+02, 1.378946000869402e+02, 66.363534063880167;
  % 1.220482473888913e+02,  72.922377362135506, 47.618375837277867;
  % 7.361177385694313e+01,  8.115383312930386e+01, 72.406970515365515;
  % 8.819028346568220e+01,  1.499734300865982e+02, 79.469622105133908;
  % 1.597862906134208e+02,  1.105014199862938e+02, 70.025477598063702;
  % 6.159951608429921e+01,  1.569554992912268e+02, 73.069941746938326;
  % 6.783081579899851e+01,  7.844358579822623e+01, 38.044413896617669;
  % 8.072262672982433e+01,  1.189531604409474e+02, 64.408485099343920;
  % 1.463994888052817e+02,  8.777200988196230e+01, 46.493517336543391;
  % 1.536616738978522e+02,  6.672340490407777e+01, 23.207872893391407;
  % 5.322359666034333e+01,  9.765154800942299e+01, 20.254444954636842;
  % 1.410457777462161e+02,  1.538210264511630e+02, 76.625309465556043;
  % 1.132997890371578e+02,  1.033481569585932e+02, 63.125625972731122;
  % 5.065373181088210e+01,  1.624948951289558e+02, 66.784530402276204;
  % 1.657596417009627e+02,  9.294229330596747e+01, 54.230649438973259;
  % 1.222348255003406e+02,  6.734440811858958e+01, 73.614953154623720;
  % 4.992481000345745e+01,  1.434973022997406e+02, 53.248253154374183;
  % 6.045796165598497e+01,  9.317799551847328e+01, 32.971330335181079;
  % 1.642121446856819e+02,  9.876946451729935e+01, 73.561523537512116;
  % 5.228367048679788e+01,  8.016717768565396e+01, 58.863044189240448;
  % 1.681187555292010e+02,  1.659427187567823e+02, 52.986570506108421;
  % 4.110188322046096e+01,  1.231035279979451e+02, 46.837424237331291;
  % 1.019707998110143e+02,  9.537307042764756e+01, 72.902490531615626;
  % 1.323444324299894e+02,  1.446113218422077e+02, 75.386084605828117;
  % 1.161039185093896e+02,  1.289591450145620e+02, 88.605646621606127;
  % 5.453230591769799e+01,  1.283129694141639e+02, 22.934205907411897;
  % 5.629942778193472e+01,  1.567364676475588e+02, 71.131648795729205;
  % 1.356404034240190e+02,  1.409244384201849e+02, 64.919460775576255;
  % 1.353980200550252e+02,  8.733888968818356e+01, 76.113958180576574;
  % 1.562358399405659e+02,  4.957577768955932e+01, 76.447218816318340;
  % 6.908643745102425e+01,  1.137448124471752e+02, 66.794166366824356;
  % 1.099193456072939e+02,  1.656662514721841e+02, 78.129872863194009;
  % 9.784866777427357e+01,  4.854531402547812e+01, 57.713186644019700;
  % 1.467210078948024e+02,  1.550299303103431e+02, 20.143246090024437;
  % 5.036310148912456e+01,  7.363493417739761e+01, 89.485543641140424;
  % 6.809939147950307e+01,  5.058684262757231e+01, 39.224598495382153;
  % 8.461566199495947e+01,  1.666784879895814e+02, 50.665848219792480;
  % 1.672034524426207e+02,  6.539244389263217e+01, 22.213398876297642;
  % 1.342640898899008e+02,  8.154995140003953e+01, 83.969226367125316;
  % 4.497989431257338e+01,  9.483208816317341e+01, 29.595425987966216;
  % 9.112245024343316e+01,  1.030947457885754e+02, 32.083086622248246;
  % 1.694744987549772e+02,  1.510121538816217e+02, 28.085054979298548;
  % 1.676175598775339e+02,  8.518659156217305e+01, 52.784477685246443;
  % 7.827058219085836e+01,  4.164762248412715e+01, 87.022702449975228;
  % 6.122399537328910e+01,  1.312854511385135e+02, 49.468523034523074;
  % 5.933472569401872e+01,  8.622013334436897e+01, 44.832387854396501;
  % 1.214000937591466e+02,  1.596316321668983e+02, 37.162873499323034;
  % 1.697417848176984e+02,  5.453504852254214e+01, 24.656151897055132;
  % 9.796334027015541e+01,  1.144536725674922e+02, 20.783375339746961;
  % 1.199890572642272e+02,  1.635534136404759e+02, 40.344071960353233
];

% ----------------------------
% Inisialisasi radius (50×1)
radius = [
    7;
    6;
    9
  % 6.185872968728452;
  % 8.964777724273965;
  % 5.485747135843992;
  % 9.037518297143492;
  % 6.008904071349907;
  % 9.235594138539918;
  % 8.538654486202947;
  % 6.037835329366217;
  % 8.126538563083551;
  % 8.247119790472057;
  % 4.472953753848000;
 % 10.570936834379474;
 %  9.179010286357329;
 %  7.456135694904914;
 %  5.249681114365560;
 %  8.499316985105903;
 %  6.948839887779886;
 %  7.796712152048944;
 %  6.894786768783632;
 %  5.605224824704907;
 %  7.543407775395499;
 %  9.372789648181408;
 % 10.320786578246002;
 %  8.812422183856746;
 %  7.535837756950957;
 %  5.405198509683015;
 %  8.116378111187737;
 %  6.247466717337204;
 %  9.212976639978034;
 %  5.907951127615620;
 %  7.436736997897347;
 %  8.386368725566832;
 %  6.101062151079636;
 %  5.706359280856537;
 %  7.273562645133646;
 %  5.615773461848578;
 %  7.334084224570097;
 %  6.657780059093303;
 %  9.858501863730547;
 %  9.057918755858047;
 %  6.593862973526205;
 %  5.310714982307363;
 %  8.655301180046145;
 %  7.677012065903208;
 %  8.541646704193894;
 %  6.270589509444984;
 %  7.377924528942020;
 %  8.190405633985000;
 %  8.113725408404168;
 %  5.312221181635445
];

    % Jika Anda juga butuh radius1 (radius "asli" sebelum diskalakan 1.1):
    

    %% radnomize Obstacles
    
    if randomize == 1 % Static
    nb = nObsVec; % number of obstacles
    
    % Initialize arrays to store obstacle properties
    cposx = zeros(1, nb);
    cposy = zeros(1, nb);
    cposz = zeros(1, nb);
    radius = zeros(nb, 1);
    
    % Define bounds based on centered flag
    if centered == 0
        x_min = 40; x_max = 170;
        y_min = 40; y_max = 170;
        z_min = 60; z_max = 90;
    else
        x_min = 70; x_max = 71; % Note: this gives no variation in original code
        y_min = 70; y_max = 71;
        z_min = 50; z_max = 90;
    end
    
    % Minimum distance between obstacle centers (can be adjusted)
    min_separation = -20; % Adjust this value based on your needs
    max_attempts = 1000; % Maximum attempts to place each obstacle
    
    % Place obstacles one by one
    for i = 1:nb
        placed = false;
        attempts = 0;
        
        while ~placed && attempts < max_attempts
            attempts = attempts + 1;
            
            % Generate random position
            temp_x = x_min + (x_max - x_min) * rand();
            temp_y = y_min + (y_max - y_min) * rand();
            temp_z = z_min + (z_max - z_min) * rand();
            
            % Generate random radius
            temp_radius = radii + (5 - 3) * rand();
            
            % Check if this position overlaps with existing obstacles
            valid_position = true;
            
            for j = 1:i-1
                % Calculate distance between centers
                distance = sqrt((temp_x - cposx(j))^2 + (temp_y - cposy(j))^2);
                
                % Check if obstacles would overlap (distance < sum of radii + minimum separation)
                required_distance = radius(j) + temp_radius + min_separation;
                
                if distance < required_distance
                    valid_position = false;
                    break;
                end
            end
            
            % If position is valid, place the obstacle
            if valid_position
                cposx(i) = temp_x;
                cposy(i) = temp_y;
                cposz(i) = temp_z;
                radius(i) = temp_radius;
                
                placed = true;
            end
        end
        
        % If we couldn't place the obstacle after max attempts
        if ~placed
            warning('Could not place obstacle %d without overlap after %d attempts', i, max_attempts);
            % Place it anyway with last attempted position (or handle as needed)
            cposx(i) = temp_x;
            cposy(i) = temp_y;
            cposz(i) = temp_z;
            radius(i) = temp_radius;
            
        end
    end
    
    % Create position matrix
    Cpos = [cposx' cposy' cposz'];
    
    % Optional: Display placement statistics
    fprintf('Successfully placed %d obstacles\n', nb);
    
    % Optional: Verify no overlaps (for debugging)
    overlap_count = 0;
    for i = 1:nb
        for j = i+1:nb
            distance = sqrt((cposx(i) - cposx(j))^2 + (cposy(i) - cposy(j))^2);
            if distance < (radius(i) + radius(j))
                overlap_count = overlap_count + 1;
            end
        end
    end
    
    if overlap_count > 0
        warning('Found %d overlapping obstacle pairs', overlap_count);
    else
        fprintf('No overlaps detected\n');
    end
    end

    radius1 = radius / 1.1;
    SEQUENTIAL_SPAWN_INTERVAL = 300;  % Spawn every 150 iterations
MIN_GOAL_DISTANCE_THRESHOLD = 100; % Don't spawn if agent within 10 units of goal
next_spawn_iteration = SEQUENTIAL_SPAWN_INTERVAL; % First spawn at iteration 150
sequential_spawn_count = 0; % Track how many spawn events occurred
obstacles_per_spawn = nObsVec; % Number of obstacles to spawn each time (same as initial)
original_M_obs = M_obs;
    
    if static == 1
        % Static mode only
        obstacles = Cpos';
        M_obs = size(obstacles,2);   
        radius     = radius;        % Use static radius
        obs_speeds = zeros(3, size(Cpos,1));  % Zero velocity
        obstacle_types = ones(1, M_obs);  % 1 = static
        static_obs_indices = 1:M_obs;
        dynamic_obs_indices = [];
    else
        % Dynamic mode or mixed mode
        if mixed == 1
            %% Belum Jadi
            % Mixed mode: both static and dynamic obstacles
            
            % Static obstacles
            static_obstacles = Cpos';
            static_radius = radius;
            n_static = size(static_obstacles, 2);
            
            % Dynamic obstacles
            M_obs_dynamic = nObsVec;
            
            % Initialize dynamic obstacle states
            obs_states = zeros(12, M_obs_dynamic);
            obs_goals = zeros(3, M_obs_dynamic);
            
            for k = 1:M_obs_dynamic
                % Start near a random target
                t = randi(M);
                start_pos = [180+randn*10; 180+randn*10; 40+randn*10];
            
            % Set goal to another position
            end_pos = [40; 40; 40];
                % Initialize state
                obs_states(1:3, k) = start_pos;
                obs_states(4:12, k) = zeros(9,1);
                obs_goals(:, k) = end_pos;
            end
            
            % Combine all obstacles
            obstacles = [static_obstacles, obs_states(1:3, :)];
            radius = [static_radius; r_dyn * ones(M_obs_dynamic, 1)];
            M_obs = n_static + M_obs_dynamic;
            obs_speeds = zeros(3, M_obs);
            
            % Track obstacle types: 1=static, 2=dynamic
            obstacle_types = [ones(1, n_static), 2*ones(1, M_obs_dynamic)];
            static_obs_indices = 1:n_static;
            dynamic_obs_indices = (n_static+1):M_obs;
            
        else
            %% Pure dynamic mode
            M_obs = nObsVec;
            
            % Initialize dynamic obstacle states
            obs_states = zeros(12, M_obs);
            obs_goals = zeros(3, M_obs);
            
            for k = 1:M_obs
                % Start near a random target
                t = randi(M);
                % start_pos = target{t} + [0;80;0] - [randn*20; randn*20; randn*5];
                %% Penentuan Arah Datang Obstacle Dinamis
                if datang == 1 %% Dari Depan
                start_pos = [180+randn*10; 180+randn*10; 65+randn*3];
            
                % Set goal to another position
                end_pos = [-40+randn*1; -40+randn*1; 60+randn*10];
                elseif datang == 2
                    end_pos = [580+randn*10; 580+randn*10; 61+randn*2];
            
                    % Set goal to another position
                    start_pos = [-30+randn*1; -30+randn*1; 61+randn*2];
                else
                     end_pos = [150+randn*20; 10+randn*20; 64+randn*7];
            
                    % Set goal to another position
                    start_pos = [-10+randn*20; 190+randn*20; 64+randn*4];
                end

            
                % Initialize state
                obs_states(1:3, k) = start_pos;
                obs_states(4:12, k) = zeros(9,1);
                obs_goals(:, k) = end_pos;
            end
            
            % Only dynamic obstacles
            obstacles = obs_states(1:3, :);
            radius = r_dyn * ones(M_obs, 1);
            obs_speeds = zeros(3, M_obs);
            obstacle_types = 2*ones(1, M_obs);  % All dynamic
            static_obs_indices = [];
            dynamic_obs_indices = 1:M_obs;
        end
    end
    
    %% Random initial UAV pos outside any obstacle
    start = cell(N,1);
    for i = 1:N
        valid = false;
        while ~valid
            xr = 15*rand(1);
            yr = 15*rand(1);
            zr = 0*rand(1);
            valid = true;
            % cek setiap obstacle (dinamis atau statis) via 'obstacles' & 'radius'
            for k = 1:size(obstacles,2)
                dx = xr - obstacles(1,k);
                dy = yr - obstacles(2,k);
                dz = zr - obstacles(3,k);
                if static == 1
                % jika (x,y) masuk lingkaran obstacle dan z di bawah tinggi obstacle
                if dx^2 + dy^2 < radius(k)^2 && zr < obstacles(3,k)
                    valid = false;
                    break;
                end
                else
                if dx^2 + dy^2 +dz^2 < radius(k)^2
                    valid = false;
                    break;
                end
                end
            end
        end
        start{i} = [xr; yr; zr];
    end
%% Formasi Nyoba    
    % % Virtual leader state (add after target definition)
    % xr = cell(M,1);
    % vr = cell(M,1);
    % xr_init = cell(M,1);
    % xr_history = cell(M,1);  % To store trajectory
    % 
    % %% Communication topology matrices (add after line ~58)
    % % These define which agents can communicate
    % A_comm = zeros(N,N);  % Adjacency matrix for agent communication
    % L_comm = zeros(N,N);  % Laplacian matrix
    % B_comm = zeros(N,N);  % Leader communication matrix
    % 
    % % Example topology (modify based on your needs)
    % % This creates a connected graph
    % for i = 1:N
    %     for j = 1:N
    %         if i ~= j && norm(start{i} - start{j}) < 100  % Neighbors within 100 units
    %             A_comm(i,j) = 1;
    %         end
    %     end
    % end
    % 
    % % Compute Laplacian
    % for i = 1:N
    %     L_comm(i,i) = sum(A_comm(i,:));
    %     for j = 1:N
    %         if i ~= j
    %             L_comm(i,j) = -A_comm(i,j);
    %         end
    %     end
    % end
    % 
    % % Leader communication (at least one agent can see the leader)
    % B_comm = diag(rand(N,1) > 0.7);  % Random agents can see leader
    % if sum(diag(B_comm)) == 0
    %     B_comm(1,1) = 1;  % Ensure at least agent 1 sees leader
    % end
    
    %% setting up simulation
    fill3([0 200 200 0],[0 0 200 200],[0 0 0 0],[0.3 0.3 0.3]);
    hold on
    grid on
    x = 0:4:200;
    y = 0:4:200;
    xlabel("x");
    ylabel("y");
    zlabel("z");
    xlim([0 200]);
    ylim([0 200]);
    zlim([0 100]);
    fhandle=figure(1);
    hObs = gobjects(0,1);
    
    % Initialize static obstacles
    if static == 1
        hObs = gobjects(M_obs,1);
        for i = 1:M_obs
            % create_cylinder(radius1(i), Cpos(i,:), [0.45, 0.58, 0.76]);
            [h1,~,~] = create_cylinder(radius(i), Cpos(i,:), [0.45, 0.58, 0.76]);
            hObs(i) = h1;
        end
    else
        % Dynamic or mixed mode
        if mixed == 1
            % Draw static obstacles as cylinders
            hObs = gobjects(length(static_obs_indices),1);
            for idx = 1:length(static_obs_indices)
                i = static_obs_indices(idx);
                % create_cylinder(radius1(idx), Cpos(idx,:), [0 0 0]);
                [h1,~,~] = create_cylinder(radius(idx), Cpos(idx,:), [0.45, 0.58, 0.76]);
                hObs(idx) = h1;
            end
        else
            hObs = [];
        end
        
        % Initialize dynamic obstacle graphics as quadcopters
        if ~isempty(dynamic_obs_indices)
            hObsQuad = cell(length(dynamic_obs_indices), 1);
            hObsCircle = gobjects(length(dynamic_obs_indices), 1);
            hObsCirclex = gobjects(length(dynamic_obs_indices), 1);
            
            for idx = 1:length(dynamic_obs_indices)
                k = dynamic_obs_indices(idx);
                if mixed == 1
                    % In mixed mode, use local index for obs_states
                    obs_idx = idx;
                else
                    % In pure dynamic mode, use k directly
                    obs_idx = k;
                end
                
                pos_k = obs_states(1:3, obs_idx);
                % Create quadcopter graphic
                hObsQuad{idx} = create_quadcopter_graphics(pos_k, obs_states(7:9, obs_idx), [1 0 0], 2);
                radiussafe = 0.25
                
                % Create safety circle around obstacle
                theta_circle = linspace(0, 2*pi, 50);
                circle_x = pos_k(1) + r_dyn * cos(theta_circle);
                circle_y = pos_k(2) + r_dyn * sin(theta_circle);
                circle_z = pos_k(3) * ones(size(theta_circle));
                hObsCircle(idx) = plot3(circle_x, circle_y, circle_z, 'r--', 'LineWidth', 1.5);

                circle_xx = pos_k(1) + radiussafe * cos(theta_circle);
                circle_yy = pos_k(2) + radiussafe * sin(theta_circle);
                circle_zz = pos_k(3) *  ones(size(theta_circle));
                hObsCirclex(idx) = plot3(circle_xx, circle_yy, circle_zz, 'b', 'LineWidth', 3);
            end
        else
            hObsQuad = [];
            hObsCircle = [];
        end
    end
    
    Nfeas=100000;
    feas_point=zeros(Nfeas,3);
    for j = 1:Nfeas
        flag = 0;
        while ~flag
            flag = 1;
            feas_point(j,:) = [200*rand 200*rand 100*rand];
            % Loop melalui setiap obstacle (dinamis atau statis, sesuai toggle)
            for k = 1 : size(obstacles,2)
                dx = feas_point(j,1) - obstacles(1,k);
                dy = feas_point(j,2) - obstacles(2,k);
                horiz_dist2 = dx^2 + dy^2;
                % obstacles(3,k) = tinggi obstacle; radius(k) = jari-jari obstacle
                if horiz_dist2 < radius(k)^2 && feas_point(j,3) < obstacles(3,k)
                    flag = 0;
                    break;
                end
            end
        end
    end
    
    %Defining agent & target
    colors = lines(M);  % M warna berbeda
    cmap = lines(max(M,N));  % Define cmap early to avoid errors with dynamic obstacles
    final_stopping_pos = cell(N, 1);
    
    for t = 1:M
      
      plot3(target{t}(1), target{t}(2), target{t}(3), 'o', ...
          'MarkerSize', 15, 'MarkerFaceColor', colors(t,:), ...
          'MarkerEdgeColor', 'black', 'LineWidth', 2);
    
    % Target label
    text(target{t}(1)+5, target{t}(2)+5, target{t}(3)+5, ...
         sprintf('Target %d', t), 'FontSize', 12, 'FontWeight', 'bold', ...
         'Color', colors(t,:), 'BackgroundColor', 'white', ...
         'EdgeColor', 'black');
    end
    legend(arrayfun(@(t) sprintf('Target %d',t), 1:M, 'UniformOutput',false));
    
    fprintf('\n=== Simulation Settings ===\n');
    fprintf('APF Algorithm: %s\n', apf_names{APF_type});
    fprintf('Obstacle Mode: %s\n', mode_str);
    fprintf('Number of Agents: %d\n', N);
    fprintf('Number of Targets: %d\n', M);
    if static == 0
        fprintf('Dynamic Obstacles: %d\n', length(dynamic_obs_indices));
        if mixed == 1
            fprintf('Static Obstacles: %d\n', length(static_obs_indices));
        end
    end
    fprintf('===========================\n\n');
    
    titlehandle=title(sprintf("Simulation Loading - %s with %s", apf_names{APF_type}, mode_str));
    disp('Simulation Loading');
    subtitlehandle=subtitle("Iteration 0");
    % h=plot3(target{1}(1),target{1}(2),target{1}(3),"Marker","*","Color","black","LineStyle",":");

    
    % Defining intial position of the UAV.
    state0=zeros(9,1);
    
    for j=1:N
        state{j}=[start{j}; state0];
        all_state{j}=[start{j}; state0];
        current_pos{j}=start{j};
        previous_pos{j}=current_pos{j};
        data_points{j}=zeros(iteration,3);
        all_Ft{j}=zeros(3,1);
        Ftp{j} = zeros(3,1); 
    end
    
    figure(1)
    hold on
    grid on
    axis([0 200 0 200 0 100]);   % set limit x,y,z
    axis manual;                 % kunci agar tidak autoscale
    
    %Path Planning
    F = zeros(3,length(obstacles));
    
    %% --- Task Allocation: nearest-first with equal quotas + theta ---
    
    % 0) Inisiasi theta untuk deadlock‐free swap (makalah Hu et al.)
    theta = (1:N) + rand(1,N)*0.1;
    fprintf('Initial theta: '); fprintf('%.2f ', theta); fprintf('\n');
    
    % 1) Hitung jarak tiap agen ke tiap target & tampilkan
    D = zeros(N, M);
    for j = 1:N
        for t = 1:M
            D(j,t) = norm(current_pos{j} - target{t});
        end
        fprintf('Agent %d distances:', j);
        for t = 1:M
            fprintf('  T%d=%.2f', t, D(j,t));
        end
        fprintf('\n');
    end
    
 
    % 1) hitung rata2 jarak ke tiap target
    avgD = mean(D,1);              % D adalah matriks N×M jarak agen→target
    
    % 2) urutkan target berdasarkan jarak naik
    [~, idx] = sort(avgD, 'ascend');
    
    % 3) buat kuota
    base  = floor(N/M);
    rem   = mod(N, M);
    quota = base * ones(1, M);
    
    % 4) berikan +1 ke rem target paling kecil avg jaraknya
    quota(idx(1:rem)) = quota(idx(1:rem)) + 1;
    
    fprintf('Quotas per target: '); disp(quota);
    
    % 3) Inisiasi assigned_goal & counter
    assigned_goal = zeros(1, N);
    counts = zeros(1, M);
    
    % 4) Buat daftar pasangan (agent, target) & urutkan by jarak naik
    [pj, pt] = ndgrid(1:N, 1:M);
    pairs = [pj(:), pt(:)];
    [~, order] = sort(D(:));
    sorted_pairs = pairs(order, :);
    
    % 5) Greedy nearest-first assign sesuai kuota
    for k = 1:size(sorted_pairs,1)
        j = sorted_pairs(k,1);
        t = sorted_pairs(k,2);
        if assigned_goal(j)==0 && counts(t) < quota(t)
            assigned_goal(j) = t;
            counts(t) = counts(t) + 1;
        end
    end
    fprintf('Initial assignment: '); disp(assigned_goal);
    
    % Initialize agent quadcopter graphics
    hAgentQuad = cell(N,1);  % Use cell array instead of gobjects
    for i=1:N
        if assigned_goal(i) > 0 && assigned_goal(i) <= size(cmap,1)
            col = cmap(assigned_goal(i),:);
        else
            col = [0 0 1]; % Default blue for agents
        end
        hAgentQuad{i} = create_quadcopter_graphics(start{i}, zeros(3,1), col, 1.5); % Scale 1.5 for agents
        htext{i}=text(start{i}(1)-1,start{i}(2)-1,start{i}(3)-1,(sprintf('%d', i))); %Callout name
    end
    
    % Initial quota distribution
    initcounts = histcounts(assigned_goal,1:M+1);
    
    % Redistribution for under-quota targets
    under = find(initcounts < quota);
    over = find(initcounts > quota);
    
    for t = under
        need = quota(t) - initcounts(t);
        while need>0 && ~isempty(over)
            s = over(1);
            donors = find(assigned_goal==s);
            [~, idxs] = sort( arrayfun(@(j) norm(current_pos{j}-target{t}), donors ), 'descend');
            move = donors(idxs(1));
            assigned_goal(move) = t;
            need = need - 1;
            initcounts = histcounts(assigned_goal,1:M+1);
            if initcounts(s) <= quota(s)
                over(1) = [];
            end
        end
    end
    
    % message queues untuk handshake swap
    for j=1:N
      requests.in{j}   = [];
      requests.ack{j}  = [];
      requests.nack{j} = [];
      requests.sent(j).to = [];
    end

    SAVE_3D_REPLAY = true;  % Set to true to enable 3D replay data saving

if SAVE_3D_REPLAY
    fprintf('Initializing 3D replay recording...\n');
    
    % Initialize replay data structure
    replay_data = struct();
    replay_data.settings = struct();
    replay_data.settings.APF_type = APF_type;
    replay_data.settings.apf_name = apf_names{APF_type};
    replay_data.settings.mode_str = mode_str;
    replay_data.settings.static = static;
    replay_data.settings.mixed = mixed;
    replay_data.settings.datang = datang;
    replay_data.settings.N = N;
    replay_data.settings.M = M;
    replay_data.settings.M_obs = M_obs;
    replay_data.settings.iteration = iteration;
    replay_data.settings.dt = dt;
    replay_data.settings.xyp = xyp;
    replay_data.settings.zp = zp;
    
    % Save static elements
    replay_data.static_elements = struct();
    replay_data.static_elements.targets = target;
    replay_data.static_elements.start_positions = start;
    replay_data.static_elements.colors = cmap;
    replay_data.static_elements.target_colors = colors;
    replay_data.static_elements.params = params;
    
    % Save obstacles
    replay_data.static_elements.obstacles = struct();
    replay_data.static_elements.obstacles.positions = obstacles;
    replay_data.static_elements.obstacles.radii = radius;
    if static == 1
        replay_data.static_elements.obstacles.Cpos = Cpos;
        replay_data.static_elements.obstacles.type = 'static';
    else
        replay_data.static_elements.obstacles.initial_states = obs_states;
        replay_data.static_elements.obstacles.goals = obs_goals;
        replay_data.static_elements.obstacles.type = 'dynamic';
        if mixed == 1
            replay_data.static_elements.obstacles.static_indices = static_obs_indices;
            replay_data.static_elements.obstacles.dynamic_indices = dynamic_obs_indices;
        end
    end
    
    % Pre-allocate arrays for dynamic data
    max_iter = iteration;
    replay_data.dynamic = struct();
    replay_data.dynamic.agent_positions = zeros(N, 3, max_iter);
    replay_data.dynamic.agent_orientations = zeros(N, 3, max_iter);
    replay_data.dynamic.agent_assignments = zeros(N, max_iter);
    replay_data.dynamic.agent_status = false(N, max_iter);  % done/not done
    replay_data.dynamic.agent_crashed = false(N, max_iter);
    replay_data.dynamic.target_reached_time = zeros(N, 1);
    
    if static == 0
    % Calculate maximum possible obstacles including sequential spawns
    max_spawns = ceil(iteration / SEQUENTIAL_SPAWN_INTERVAL);
    max_possible_obstacles = M_obs + (max_spawns * obstacles_per_spawn);
    
    replay_data.dynamic.obstacle_positions = zeros(max_possible_obstacles, 3, max_iter);
    replay_data.dynamic.obstacle_orientations = zeros(max_possible_obstacles, 3, max_iter);
    replay_data.dynamic.obstacle_states = zeros(12, max_possible_obstacles, max_iter);
    
    % Track actual number of obstacles per frame
    replay_data.dynamic.num_obstacles_per_frame = zeros(max_iter, 1);
end
    
    % Save trajectory data structure
    replay_data.dynamic.trajectories = cell(N, 1);
    for j = 1:N
        replay_data.dynamic.trajectories{j} = [];
    end
    
    % Save initial assignment
    replay_data.dynamic.initial_assignment = assigned_goal;
    
    fprintf('3D Replay recording initialized\n');
    replay_frame_count = 0;
end
    
    %% parameters DAS
    params.R_STOP    =1;
    params.R_SWITCH  = 15;
    params.delta     = 5;    % minimal distance δ
    params.dtheta    = 0.1;
    params.comm_range= 50;
    params.timeout    = 2;  % atau detik maksimal, misal 5
    params.quota = quota;    % vektor 1×M
    done = false(1,N);
    params.assigngoal = [];   % matriks kosong: tiap baris satu snapshot assigned_goal
    target_reached_time = zeros(N, 1);  % Time when each agent reached target
    crashed_agents = false(1, N);       % Track crashed agents
    target_filled = false(1, M);        % Track which targets are completely filled
    crash_recovery_points = cell(N, 1); % Store recovery points for crashed agents
    
    % neighbour list (update statis pertama kali)
    for j=1:N
      neighbors{j} = find_neighbors(j, current_pos, params.comm_range);
    end
    
    flag_alo=1;
    target_dilacak = 1:M
    idx_dkt=zeros(M,1);
    idx_hnt=zeros(M,1);
    goal=start;
    
    titlehandle.String=sprintf("Simulation Running - %s", apf_names{APF_type});
    disp('Running');
    disp('Click the figure and press any key to stop iteration')
    
    %% Initialize logging variables
    done_all = false;
    num_steps = ceil(iteration/alocfreq);
    assigned_history = zeros(N, num_steps);
    theta_history    = zeros(N, num_steps);
    max_possible_obstacles = M_obs + (iteration / 100) * nObsVec + 50; % Add buffer
    distance_obs_all = zeros(N, max_possible_obstacles, iteration);

    time_steps       = (1:num_steps) * alocfreq * dt;
    params.assigned_goal = assigned_goal; 
    
    % Initialize history
    distance_obs = zeros(N, iteration);
    crash = false(N, iteration);
    posA_log   = zeros(N, 3, iteration);
    posO_log   = zeros(max_possible_obstacles, 3, iteration);
  % All obstacles (static + dynamic)
    Fatt_log   = zeros(N, 3, iteration);
    Frep_log   = zeros(N, 3, iteration);
    iter_log   = (1:iteration)';
    
    % Initialize speed tracking
    agent_speeds = zeros(N, iteration);
    if static == 0
    obstacle_speeds = zeros(max_possible_obstacles, iteration);
    else
    obstacle_speeds = [];
    end
    
    if static==1
        for i=1:iteration
            % Fix: Only assign to the rows corresponding to actual obstacles
            posO_log(1:M_obs,:,i) = obstacles';
            % Fill remaining rows with NaN if needed
            if M_obs < size(posO_log,1)
                posO_log(M_obs+1:end,:,i) = NaN;
            end
        end
    end
    
    % Initialize handles for visualization
    % Initialize trajectory handles for both static and dynamic modes
    hTraj = cell(N,1);
    for jj = 1:N
        hTraj{jj} = [];  % Will store multiple line handles for color segments
    end
    
    % Track assignment changes
    prev_assigned_goal = assigned_goal;
    segment_start_idx = ones(N,1);  % Starting index of current color segment
    
    % Initialize distance matrix
    distance_agents = zeros(N, N, iteration);
    
   
    detected_agents = cell(N, iteration);     % Which other agents each agent detects
    
    %% MAIN ITERATION
    for i=1:iteration
        % Update dynamic obstacles FIRST 
        if static == 0
            for k = 1:M_obs
                % Get current obstacle state
                obs_pos = obs_states(1:3, k);
                obs_goal = obs_goals(:, k);
                
                % Simple proportional navigation toward goal
                direction = obs_goal - obs_pos;
                dist_to_goal = norm(direction);
                
                if dist_to_goal > 5  % If not at goal
                    % Desired acceleration toward goal (using configurable speed)
                    desired_velocity = obstacle_speed * direction / dist_to_goal;
                    current_velocity = obs_states(4:6, k);
                    desired_acc = (desired_velocity - current_velocity);
                    
                    % Update obstacle state using quadcopter dynamics
                    obs_states(:, k) = obs_state_update(obs_states(:, k), desired_acc, dt);
                
                else
                    % Reached goal, set new goal
                    obs_goals(1:3, k) =  [-100;-100;50];
                end
                
                % Track obstacle speed
                obstacle_speeds(k, i) = norm(obs_states(4:6, k));
            end
            
            % UPDATE 
            obstacles = obs_states(1:3, :);
            obs_speeds = obs_states(4:6, :);
            
            % Update visualization ( if dynamic obstacles)
            if exist('hObsQuad', 'var') && iscell(hObsQuad) && ~isempty(hObsQuad)
    for k = 1:min(M_obs, length(hObsQuad))
        if k <= size(obs_states, 2) && ~isempty(hObsQuad{k}) && isvalid(hObsQuad{k})
            pos_k = obs_states(1:3, k);
            angles_k = obs_states(7:9, k);
            
            % Update quadcopter graphic
            update_quadcopter_graphics(hObsQuad{k}, pos_k, angles_k);
            
            % Update safety circles if they exist
            if exist('hObsCircle', 'var') && length(hObsCircle) >= k && isvalid(hObsCircle(k))
                theta_circle = linspace(0, 2*pi, 50);
                circle_x = pos_k(1) + r_dyn * cos(theta_circle);
                circle_y = pos_k(2) + r_dyn * sin(theta_circle);
                circle_z = pos_k(3) * ones(size(theta_circle));
                set(hObsCircle(k), 'XData', circle_x, 'YData', circle_y, 'ZData', circle_z);
            end
            
            if exist('hObsCirclex', 'var') && length(hObsCirclex) >= k && isvalid(hObsCirclex(k))
                radiussafe = 0.25;
                circle_xx = pos_k(1) + radiussafe * cos(theta_circle);
                circle_yy = pos_k(2) + radiussafe * sin(theta_circle);
                circle_zz = pos_k(3) * ones(size(theta_circle));
                set(hObsCirclex(k), 'XData', circle_xx, 'YData', circle_yy, 'ZData', circle_zz);
            end
        end
    end
end
            
            % Log obstacle positions
            if size(obstacles, 2) > 0
    posO_log(1:size(obstacles,2), :, i) = obstacles';
    % Fill remaining slots with NaN
    if size(obstacles,2) < size(posO_log,1)
        posO_log(size(obstacles,2)+1:end, :, i) = NaN;
    end
end
        elseif static == 1
            
        end
        %% Sequential Obstacle Batch Spawning Logic
if static == 0 && i == next_spawn_iteration
    % Check if any agent is close to goal
    agent_near_goal = check_agent_near_goal(current_pos, assigned_goal, target, MIN_GOAL_DISTANCE_THRESHOLD);
    
    if ~agent_near_goal
        % Safe to spawn new batch of obstacles
        [obs_states, obs_goals, obstacles, radius, M_obs, obstacle_types] = ...
            spawn_obstacle_batch(obs_states, obs_goals, obstacles, radius, M_obs, obstacle_types, datang, obstacles_per_spawn, r_dyn);
        
        sequential_spawn_count = sequential_spawn_count + 1;
        
        % Create graphics for all new obstacles in the batch
        if exist('hObsQuad', 'var') && iscell(hObsQuad)
            % Calculate range of new obstacles
            start_idx = M_obs - obstacles_per_spawn + 1;
            end_idx = M_obs;
            
            for new_k = start_idx:end_idx
                if new_k <= size(obs_states, 2)
                    new_pos = obs_states(1:3, new_k);
                    new_angles = obs_states(7:9, new_k);
                    
                    % Expand graphics arrays if needed
                    if length(hObsQuad) < new_k
                        hObsQuad{new_k} = [];
                    end
                    hObsQuad{new_k} = create_quadcopter_graphics(new_pos, new_angles, [1 0 0], 2);
                    
                    % Create safety circles
                    if exist('hObsCircle', 'var')
                        if length(hObsCircle) < new_k
                            hObsCircle(new_k) = gobjects(1);
                        end
                        
                        theta_circle = linspace(0, 2*pi, 50);
                        circle_x = new_pos(1) + r_dyn * cos(theta_circle);
                        circle_y = new_pos(2) + r_dyn * sin(theta_circle);
                        circle_z = new_pos(3) * ones(size(theta_circle));
                        hObsCircle(new_k) = plot3(circle_x, circle_y, circle_z, 'r--', 'LineWidth', 1.5);
                    end
                    
                    if exist('hObsCirclex', 'var')
                        if length(hObsCirclex) < new_k
                            hObsCirclex(new_k) = gobjects(1);
                        end
                        
                        radiussafe = 0.25;
                        circle_xx = new_pos(1) + radiussafe * cos(theta_circle);
                        circle_yy = new_pos(2) + radiussafe * sin(theta_circle);
                        circle_zz = new_pos(3) * ones(size(theta_circle));
                        hObsCirclex(new_k) = plot3(circle_xx, circle_yy, circle_zz, 'b', 'LineWidth', 3);
                    end
                end
            end
        end
        
        fprintf('Batch spawn #%d completed at iteration %d (Total obstacles: %d)\n', sequential_spawn_count, i, M_obs);
    else
        fprintf('Obstacle batch spawn blocked - agent near goal at iteration %d\n', i);
    end
    
    % Schedule next spawn
    next_spawn_iteration = i + SEQUENTIAL_SPAWN_INTERVAL;
end
               
        % Task allocation
        if mod(i, alocfreq)==1
    
    %% 1. CRASH DETECTION AND HANDLING
    for j = 1:N
        if ~crashed_agents(j) % Only check non-crashed agents
            agent_crashed = false;
            
            % Check crash with obstacles
            for b = 1:size(obstacles,2)
                dx = current_pos{j}(1) - obstacles(1,b);
                dy = current_pos{j}(2) - obstacles(2,b);
                dz = current_pos{j}(3) - obstacles(3,b);
                
                if static == 1 || (exist('obstacle_types', 'var') && obstacle_types(b) == 1)
                    % Cylinder (static)
                    if dx^2 + dy^2 < radius(b)^2 && current_pos{j}(3) < obstacles(3,b)
                        agent_crashed = true;
                        break;
                    end
                else
                    % Sphere (dynamic)
                    dist_3d = sqrt(dx^2 + dy^2 + dz^2);
                    radiussafe = 0.25;
                    if dist_3d < radiussafe
                        agent_crashed = true;
                        break;
                    end
                end
            end
            
            % Handle newly crashed agent
            if agent_crashed
                crashed_agents(j) = true;
                old_target = assigned_goal(j);
                
                % Find nearest feasible point for crashed agent
                distances_to_feas = arrayfun(@(idx) norm(current_pos{j} - feas_point(idx,:)'), 1:size(feas_point,1));
                [~, nearest_idx] = min(distances_to_feas);
                crash_recovery_points{j} = feas_point(nearest_idx,:)';
                
                fprintf('CRASH: Agent %d crashed, was targeting Target %d, now going to recovery point\n', j, old_target);
                
                % Trigger reallocation flag
                need_reallocation = true;
            end
        end
    end
    
    %% 2. TARGET COMPLETION TRACKING (FIXED)
    %% 2. TARGET COMPLETION TRACKING (FIXED)
for j = 1:N
    if ~crashed_agents(j) && ~done(j)
        % Check if agent reached target
        target_pos = target{assigned_goal(j)};
        if norm(current_pos{j} - target_pos) < params.R_STOP
            if target_reached_time(j) == 0 % First time reaching - ONLY set if zero
                target_reached_time(j) = i * dt; % Use iteration time, not reset
                fprintf('SUCCESS: Agent %d reached Target %d at time %.2f seconds (iteration %d)\n', ...
                    j, assigned_goal(j), target_reached_time(j), i);
                
                % ✅ CRITICAL: FREEZE the target assignment for this agent
                final_target_assignment(j) = assigned_goal(j);  % Save final assignment
            end
            done(j) = true;
        end
    end
end
        
    
    %% 3. CHECK TARGET FILLING STATUS
    active_agents = ~crashed_agents;
    target_counts = zeros(1, M);
    targets_reached_count = zeros(1, M);
targets_assigned_count = zeros(1, M);

for t = 1:M
    % Count agents that actually REACHED this target
    agents_on_target = find(assigned_goal == t & ~crashed_agents);
    targets_reached_count(t) = sum(done(agents_on_target));
    
    % Count agents currently assigned (including those still moving)
    targets_assigned_count(t) = sum(assigned_goal(~crashed_agents) == t);
end

% Find targets that have NO agents reached yet (even if assigned)
targets_not_reached = find(targets_reached_count == 0);
% Find targets that have at least one agent reached but not filled
targets_partially_reached = find(targets_reached_count > 0 & targets_reached_count < params.quota);

% fprintf('Debug: Targets not reached: %s\n', mat2str(targets_not_reached));
% fprintf('Debug: Targets partially reached: %s\n', mat2str(targets_partially_reached));

for j = 1:N
    if ~crashed_agents(j) && ~done(j)
        current_target = assigned_goal(j);
        dist_to_current_target = norm(current_pos{j} - target{current_target});
        
        % ADD THESE STABILITY CHECKS:
        % Don't reassign if agent is committed to current target
        if dist_to_current_target < params.COMMITMENT_DISTANCE
            continue; % Skip all reassignment logic
        end
        
        % Don't reassign if agent is close and target still needs them
        if dist_to_current_target < params.STABILITY_DISTANCE
            agents_close_to_current = sum(arrayfun(@(k) ~crashed_agents(k) && assigned_goal(k) == current_target && ...
                norm(current_pos{k} - target{current_target}) < params.COMMITMENT_DISTANCE, 1:N));
            agents_reached_current = sum(done(find(assigned_goal == current_target)) & ~crashed_agents(find(assigned_goal == current_target)));
            
            if (agents_reached_current + agents_close_to_current) <= params.quota(current_target)
                continue; % Don't reassign stable agents
            end
        end
        
        % PRIORITY 1: If there are targets with NO agents reached yet
        if ~isempty(targets_not_reached)
            % Check if current target already has someone who reached it
            if targets_reached_count(current_target) > 0
                % Current target already has someone, can move this agent
                % Find closest unreached target
                distances_to_unreached = arrayfun(@(t) norm(current_pos{j} - target{t}), targets_not_reached);
                [min_dist, closest_idx] = min(distances_to_unreached);
                new_target = targets_not_reached(closest_idx);
                
                % Only move if this won't leave current target with no assigned agents
                if targets_assigned_count(current_target) > 1
                    old_target = assigned_goal(j);
                    assigned_goal(j) = new_target;
                    theta(j) = theta(j) + params.dtheta;
                    
                    % Update counts
                    targets_assigned_count(old_target) = targets_assigned_count(old_target) - 1;
                    targets_assigned_count(new_target) = targets_assigned_count(new_target) + 1;
                    
                    fprintf('PRIORITY 1: Agent %d moved from reached Target %d to unreached Target %d (dist: %.2f)\n', ...
                        j, old_target, new_target, min_dist);
                    continue; % Skip normal allocation
                end
            end
        end
        
        % PRIORITY 2: If all targets have at least one reached, fill quotas for partially reached
        if isempty(targets_not_reached) && ~isempty(targets_partially_reached)
            % Current target is over-quota and there are under-quota targets
            if targets_reached_count(current_target) >= params.quota(current_target)
                % Find closest partially reached target
                distances_to_partial = arrayfun(@(t) norm(current_pos{j} - target{t}), targets_partially_reached);
                [min_dist, closest_idx] = min(distances_to_partial);
                new_target = targets_partially_reached(closest_idx);
                
                old_target = assigned_goal(j);
                benefit = dist_to_current_target - distances_to_partial(closest_idx);
                if benefit < params.MIN_REASSIGN_BENEFIT
                    continue; % Not worth reassigning
                end
                assigned_goal(j) = new_target;
                theta(j) = theta(j) + params.dtheta;
                
                fprintf('PRIORITY 2: Agent %d moved from full Target %d to partial Target %d (dist: %.2f)\n', ...
                    j, old_target, new_target, min_dist);
                continue; % Skip normal allocation
            end
        end
        
        % PRIORITY 3: Emergency - ensure no target has zero assigned agents
        for t = 1:M
            if targets_assigned_count(t) == 0
                fprintf('EMERGENCY: Target %d has no assigned agents!\n', t);
                old_target = assigned_goal(j);
                assigned_goal(j) = t;
                theta(j) = theta(j) + params.dtheta;
                
                fprintf('EMERGENCY ASSIGN: Agent %d moved from Target %d to empty Target %d\n', ...
                    j, old_target, t);
                break;
            end
        end
        
        % Only call normal allocation if agent is not close to target
        if dist_to_current_target > params.STABILITY_DISTANCE
                [assigned_goal, theta, requests] = alokasi_tugasnew(j, current_pos, assigned_goal, theta, neighbors, target, params, requests, done);
        end
    end
end
    
    % %% 5. REALLOCATION AFTER CRASHES
    % if exist('need_reallocation', 'var') && need_reallocation
    %     fprintf('=== REALLOCATING AFTER CRASH ===\n');
    % 
    %     % Get active agents and recalculate optimal assignment
    %     active_agent_indices = find(~crashed_agents);
    %     N_active = length(active_agent_indices);
    % 
    %     if N_active > 0
    %         % Recalculate quotas for active agents
    %         base_quota = floor(N_active / M);
    %         remainder = mod(N_active, M);
    %         new_quotas = base_quota * ones(1, M);
    % 
    %         % Distribute remainder to unfilled targets first
    %         unfilled_indices = find(~target_filled);
    %         if length(unfilled_indices) >= remainder
    %             new_quotas(unfilled_indices(1:remainder)) = new_quotas(unfilled_indices(1:remainder)) + 1;
    %         else
    %             new_quotas(unfilled_indices) = new_quotas(unfilled_indices) + 1;
    %             remaining = remainder - length(unfilled_indices);
    %             if remaining > 0
    %                 other_targets = setdiff(1:M, unfilled_indices);
    %                 new_quotas(other_targets(1:remaining)) = new_quotas(other_targets(1:remaining)) + 1;
    %             end
    %         end
    % 
    %         % Create distance matrix for active agents
    %         distances = [];
    %         for idx = 1:N_active
    %             j = active_agent_indices(idx);
    %             if ~done(j) % Only reassign agents that haven't reached target
    %                 for t = 1:M
    %                     distances = [distances; j, t, norm(current_pos{j} - target{t})];
    %                 end
    %             end
    %         end
    % 
    %         % Sort by distance and reassign
    %         if ~isempty(distances)
    %             [~, sort_idx] = sort(distances(:, 3));
    %             sorted_distances = distances(sort_idx, :);
    % 
    %             % Reset assignment counts
    %             new_assignment_counts = zeros(1, M);
    % 
    %             % Greedy reassignment
    %             for k = 1:size(sorted_distances, 1)
    %                 j = sorted_distances(k, 1);  % agent index
    %                 t = sorted_distances(k, 2);  % target index
    % 
    %                 if new_assignment_counts(t) < new_quotas(t)
    %                     old_target = assigned_goal(j);
    %                     assigned_goal(j) = t;
    %                     new_assignment_counts(t) = new_assignment_counts(t) + 1;
    % 
    %                     if old_target ~= t
    %                         fprintf('REALLOC: Agent %d moved from Target %d to Target %d\n', j, old_target, t);
    %                     end
    %                 end
    %             end
    %         end
    %     end
    % 
    %     clear need_reallocation; % Reset flag
    %     fprintf('=== REALLOCATION COMPLETE ===\n');
    % end
    % 
    %% 6. ENSURE ALL TARGETS COVERED (Safety check)
    for t = 1:M
        if ~any(assigned_goal(~crashed_agents) == t) && ~target_filled(t)
            % Find nearest active agent to assign to this target
            active_agents_list = find(~crashed_agents);
            if ~isempty(active_agents_list)
                distances = arrayfun(@(j) norm(current_pos{j} - target{t}), active_agents_list);
                [~, min_idx] = min(distances);
                reassign_agent = active_agents_list(min_idx);
                
                old_target = assigned_goal(reassign_agent);
                assigned_goal(reassign_agent) = t;
                theta(reassign_agent) = theta(reassign_agent) + params.dtheta;
                fprintf('SAFETY: Agent %d assigned to uncovered Target %d\n', reassign_agent, t);
            end
        end
    end
    
    % Update neighbors for active agents only
    for j=1:N
        if ~crashed_agents(j) && ~done(j)
            neighbors{j} = find_neighbors(j, current_pos, params.comm_range);
        end
    end
        end
     all_agent_goals = zeros(3, N);
    for agent_idx = 1:N
        target_id = assigned_goal(agent_idx);
        if target_id >= 1 && target_id <= length(target)
            all_agent_goals(:, agent_idx) = target{target_id};
        else
            % Fallback to first target if invalid assignment
            all_agent_goals(:, agent_idx) = target{1};
        end
    end
        
    step = floor(i/alocfreq)+1;
    assigned_history(:,step) = assigned_goal(:);
    theta_history(:,step)    = theta(:);



        
        alloc_step = floor(i/alocfreq)+1;
        assigned_history(:,alloc_step) = assigned_goal';
        theta_history(:,alloc_step)    = theta';
        
        
        % Agent control loop
        % Replace the agent control loop section (around line 800+) with this modified version:

% Agent control loop
% Agent control loop - REPLACE YOUR EXISTING LOOP WITH THIS
for j=1:N
    
    % ===== CALCULATE DISTANCE TO TARGET FOR ALL AGENTS =====
    target_pos = target{assigned_goal(j)};
    distance_to_target = norm(current_pos{j} - target_pos);
    
    % ✅ CRITICAL: Skip all processing for done agents except visualization and logging
    if done(j)
        % Agent is completely done - maintain stopped state
        state{j}(4:6) = zeros(3,1);     % Zero velocity
        state{j}(10:12) = zeros(3,1);   % Zero angular velocity
        state{j}(1:3) = current_pos{j}; % Keep actual position
        Ft{j} = zeros(3,1);
        
        % ✅ FREEZE the target assignment
        if exist('final_target_assignment', 'var') && final_target_assignment(j) == 0
            final_target_assignment(j) = assigned_goal(j);
        end
        
        % ✅ LOG DISTANCE for stopped agents too
        fprintf('Stopped Agent %d: distance to target = %.3f at iteration %d\n', j, distance_to_target, i);
        
        % Update visualization and logging
        posA_log(j,:,i) = current_pos{j}';
        data_points{j}(i,:) = transpose(current_pos{j});
        
        % Update visualization
        if exist('hAgentQuad', 'var') && iscell(hAgentQuad) && length(hAgentQuad) >= j && isvalid(hAgentQuad{j})
            update_quadcopter_graphics(hAgentQuad{j}, current_pos{j}, state{j}(7:9));
        end
        set(htext{j}, 'Position', current_pos{j} - [1;1;1]);
        
        continue; % ✅ CRITICAL: Skip all other processing for done agents
    end
    
    % Handle crashed agents differently
    if crashed_agents(j)
        % Crashed agents go to recovery points, not targets
        if ~isempty(crash_recovery_points{j})
            goal{j} = crash_recovery_points{j};
        else
            % Fallback to original behavior
            if jarak3(start{j},current_pos{j})<8
                goal{j}=feas_point(randi(Nfeas),:)';
            end
        end
    else
        % Normal agents go to assigned targets
        if jarak3(start{j},current_pos{j})<8
            goal{j}=feas_point(randi(Nfeas),:)';
        end
        goal{j} = target{ assigned_goal(j) };
    end
    
    % ===== AUTO-STOP FUNCTIONALITY (IMPROVED) =====
    if ~done(j)
        % ✅ LOG DISTANCE for moving agents
        % fprintf('Moving Agent %d: distance to target = %.3f at iteration %d\n', j, distance_to_target, i);
        
        if distance_to_target <= params.R_STOP
            % Mark as done and record the ACTUAL stopping position
            if target_reached_time(j) == 0
                target_reached_time(j) = i * dt;
                fprintf('SUCCESS: Agent %d reached Target %d at time %.2f seconds (iteration %d)\n', ...
                    j, assigned_goal(j), target_reached_time(j), i);
                
                % SAVE THE ACTUAL STOPPING POSITION (not target position)
                final_stopping_pos{j} = current_pos{j};  % Save where it actually stopped
                final_target_assignment(j) = assigned_goal(j);  % Freeze assignment
                
                fprintf('  Actual stopping position: [%.2f, %.2f, %.2f]\n', ...
                    current_pos{j}(1), current_pos{j}(2), current_pos{j}(3));
                fprintf('  Target position: [%.2f, %.2f, %.2f]\n', ...
                    target_pos(1), target_pos(2), target_pos(3));
                fprintf('  Final distance to target: %.3f units\n', distance_to_target);
            end
            done(j) = true;
            
            % STOP MOVEMENT - but keep actual position
            state{j}(4:6) = zeros(3,1);     % Zero velocity
            state{j}(10:12) = zeros(3,1);   % Zero angular velocity
            
            % Keep agent at LAST REAL POSITION (not target position)
            state{j}(1:3) = current_pos{j}; % Maintain actual stopping position
            
            % Skip force calculation for stopped agents
            Ft{j} = zeros(3,1);
            
            % Update visualization at ACTUAL position
            if assigned_goal(j) > 0 && assigned_goal(j) <= size(cmap,1)
                col = cmap(assigned_goal(j),:);
            else
                col = [0 0 1];
            end
            if exist('hAgentQuad', 'var') && iscell(hAgentQuad) && length(hAgentQuad) >= j && isvalid(hAgentQuad{j})
                update_quadcopter_graphics(hAgentQuad{j}, current_pos{j}, state{j}(7:9));
            end
            
            % Update text position at actual stopping location
            set(htext{j}, 'Position', current_pos{j} - [1;1;1]);
            
            % Continue to next agent (skip force calculation)
            continue;
        end
    end
    
    % ===== ORIGINAL FORCE CALCULATION (for moving agents only) =====
    grouppp = find(assigned_goal == assigned_goal(j));
    N_agents = numel(current_pos);
    idx_other = setdiff(1:N_agents, j);
    agenlain = current_pos(idx_other);
    pos = current_pos{:,j};
    vcur = state{j}(4:6);
    min_obs_dist = inf;
    
    
    
    for k = 1:size(obstacles,2)
        if static == 1
            dx = pos(1:2) - obstacles(1:2,k);
            dist = norm(dx);
            if pos(3) < obstacles(3,k)
                min_obs_dist = min(min_obs_dist, dist - radius(k));
            end
        else
            radiussafe = 0.25;
            dist = norm(pos - obstacles(:,k)) - radiussafe;
            min_obs_dist = min(min_obs_dist, dist);
        end
    end
    
    % NEW: Add other agents as detected obstacles (within communication range)
    detected_agent_indices = [];
    detected_agent_pos = [];
    detected_agent_speeds = [];
    params.comm_range = 100;
    radiussafe = 0.25; % Obstacles Safe Radius
    for k = 1:N
        if k ~= j && ~done(k)
            agent_dist = norm(pos - current_pos{k});
            if agent_dist <=  params.comm_range  % Only detect agents within range
                detected_agent_indices = [detected_agent_indices, k];
                detected_agent_pos = [detected_agent_pos, current_pos{k}];
                detected_agent_speeds = [detected_agent_speeds, state{k}(4:6)];
            end
        end
    end

    % Create fake obstacles for other targets
    other_tg = setdiff(1:M, assigned_goal(j));
    tg_obs = cell2mat(target(other_tg))';
    tg_rad = params.MinDistance * ones(numel(other_tg),1);
    tg_speeds = zeros(3, numel(other_tg));

    % Combine all obstacles
    all_obs = [obstacles];
    all_radius = [radius];
    all_speeds = [obs_speeds];
    speed_agent = [detected_agent_speeds];
    agent_radius = [radiussafe * ones(length(detected_agent_indices), 1)];
    agent_pos = [detected_agent_pos];

    
    % Get additional forces based on selected APF algorithm
    switch APF_type
        case 1
            % Traditional APF
            [Ft_existing, Fatt_j, Frep_j] = traditional_APF( ...
                j, target, current_pos{j}, static, ...
                goal{j}, ...
                all_obs, all_radius, ...
                gains_att, gains_rep, ...
                params);
                
        case 2
            % Modified APF (MAPF)
            [Ft_existing, Fatt_j, Frep_j] = modified_APF( ...
                j, target, current_pos{j}, static, ...
                goal{j}, ...
                vcur, ...
                all_obs, all_radius, ...
                gains_att, gains_rep, ...
                params, ...
                desired_speed);
                
        case 3
            % Velocity APF (VAPF)
            [Ft_existing, Fatt_j, Frep_j] = velocity_APF( ...
                j, target, current_pos{j}, static, ...
                goal{j}, ...
                vcur, ...
                all_obs, all_radius, ...
                all_speeds, ...
                gains_att, gains_rep, ...
                params);
                
        case 4
            % Dynamic APF (DAPF) - Current implementation
            if size(all_speeds, 2) < size(all_obs, 2)
    all_speeds = [all_speeds, zeros(3, size(all_obs, 2) - size(all_speeds, 2))];
end
            [Ft_existing, Fatt_j, Frep_j] = avoidance_multiRobot_pure_dapf( ...
                j, target, current_pos{j}, static, ...
                goal{j}, ...
                vcur, ...
                Kform, ...
                all_obs, all_radius, ...
                F, ...
                agenlain, grouppp, ...
                params, ...
                params_dist, ...
                gains_att, gains_rep, ...
                desired_speed, ...
                all_speeds,speed_agent, agent_radius, agent_pos, all_agent_goals);
                
        otherwise
            error('Invalid APF_type. Use 1=Traditional, 2=MAPF, 3=VAPF, 4=DAPF');
    end
    
    % Combine forces
    Ft{j} = Ft_existing;
    
    % Store forces for logging
    posA_log(j,:,i) = current_pos{j}';
    Fatt_log(j,:,i) = Fatt_j' ;
    Frep_log(j,:,i) = Frep_j';
    
    % Update state
    previous_pos{j} = current_pos{j};

% Initialize Ftp (previous force) if it doesn't exist
if ~exist('Ftp', 'var') || length(Ftp) < j || isempty(Ftp{j})
    Ftp{j} = zeros(3,1);  % Initialize previous force
end

% Update state using new dynamics
state{j} = UAV_input_Ft(state{j}, Ft{j}, Ftp{j},dt);
current_pos{j} = state{j}(1:3);

% Update previous force for next iteration
Ftp{j} = Ft{j};
    if j == 3 && mod(i, 50) == 0  % Every 50 iterations for Agent 3
        target_pos = target{assigned_goal(j)};
        dist_to_target_debug = norm(current_pos{j} - target_pos);
        fprintf('Iter %d: Agent 3 at [%.2f, %.2f, %.2f], dist to target: %.2f, velocity: [%.2f, %.2f, %.2f]\n', ...
            i, current_pos{j}(1), current_pos{j}(2), current_pos{j}(3), ...
            dist_to_target_debug, state{j}(4), state{j}(5), state{j}(6));
    end
    
    % Track agent speed
    agent_speeds(j, i) = norm(state{j}(4:6));
    
    % CRASH DETECTION - Fixed to use current obstacle positions
    for b = 1:size(obstacles,2)
        dx = current_pos{j}(1) - obstacles(1,b);
        dy = current_pos{j}(2) - obstacles(2,b);
        dz = current_pos{j}(3) - obstacles(3,b);
        
        % Check obstacle type
        if static == 1 || (static == 0 && mixed == 1 && obstacle_types(b) == 1)
            % Cylinder (static)
            if dx^2 + dy^2 < radius(b)^2 && current_pos{j}(3) < obstacles(3,b)
                fprintf('Crash detected (static): UAV %d with obstacle %d at iteration %d\n', j, b, i);
                crash(j,i) = true;
                break;
            end
        else
            % Sphere (dynamic) - proper 3D distance check
            dist_3d = sqrt(dx^2 + dy^2 + dz^2);
            radiussafe = 0.25;
            if dist_3d < radiussafe;
                fprintf('Crash detected (dynamic): UAV %d with obstacle %d at iteration %d\n', j, b, i);
                fprintf('  UAV pos: [%.1f, %.1f, %.1f], Obs pos: [%.1f, %.1f, %.1f], Dist: %.2f, Radius: %.2f\n', ...
                    current_pos{j}(1), current_pos{j}(2), current_pos{j}(3), ...
                    obstacles(1,b), obstacles(2,b), obstacles(3,b), dist_3d, radius(b));
                crash(j,i) = true;
                break;
            end
        end
    end
    
    % Store data
    data_points{j}(i,:) = transpose(current_pos{j});
    all_state{j}(:,i+1) = state{j}';
    all_Ft{j}(:,i+1) = Ft{j};
    
    
    % Update agent quadcopter visualization
    if assigned_goal(j) > 0 && assigned_goal(j) <= size(cmap,1)
        col = cmap(assigned_goal(j),:);
    else
        col = [0 0 1];
    end
    if exist('hAgentQuad', 'var') && iscell(hAgentQuad) && length(hAgentQuad) >= j && isvalid(hAgentQuad{j})
        update_quadcopter_graphics(hAgentQuad{j}, current_pos{j}, state{j}(7:9));
    end
    
    % Update trajectory with color changes based on assignment (works for both static and dynamic)
    if exist('hTraj', 'var') && ~isempty(hTraj)
        % Check if assignment changed
        if prev_assigned_goal(j) ~= assigned_goal(j)
            % Assignment changed - finish current segment and start new one
            if ~isempty(hTraj{j}) && segment_start_idx(j) < i
                % Update the last segment to current position
                set(hTraj{j}(end), ...
                    'XData', data_points{j}(segment_start_idx(j):i,1), ...
                    'YData', data_points{j}(segment_start_idx(j):i,2), ...
                    'ZData', data_points{j}(segment_start_idx(j):i,3));
            end
            
            % Start new segment with new color
            segment_start_idx(j) = i;
            prev_assigned_goal(j) = assigned_goal(j);
            
            % Create new line segment with target color
            if assigned_goal(j) > 0 && assigned_goal(j) <= size(cmap,1)
                col = cmap(assigned_goal(j),:);
            else
                col = [0.5 0.5 0.5];
            end
            
            new_line = plot3(NaN, NaN, NaN, 'LineWidth', 1.5, 'Color', col);
            if isempty(hTraj{j})
                hTraj{j} = new_line;
            else
                hTraj{j}(end+1) = new_line;
            end
        else
            % Same assignment - update current segment
            if isempty(hTraj{j})
                % Create first segment
                if assigned_goal(j) > 0 && assigned_goal(j) <= size(cmap,1)
                    col = cmap(assigned_goal(j),:);
                else
                    col = [0.5 0.5 0.5];
                end
                hTraj{j} = plot3(NaN, NaN, NaN, 'LineWidth', 1.5, 'Color', col);
            end
            
            % Update the current segment
            set(hTraj{j}(end), ...
                'XData', data_points{j}(segment_start_idx(j):i,1), ...
                'YData', data_points{j}(segment_start_idx(j):i,2), ...
                'ZData', data_points{j}(segment_start_idx(j):i,3));
        end
    end
    
    % Update text position
    set(htext{j}, 'Position', current_pos{j} - [1;1;1]);
    
    % Distance logging
    % ✅ FIXED: Distance logging with proper freezing for stopped agents
    if ~done(j) || i == 1
        % Calculate distance to ALL obstacles for this agent
        min_dist_to_any_obstacle = inf;
        
        for k = 1:M_obs
            if k <= size(obstacles, 2)
                obs_pos = obstacles(:, k);
                obs_radius = radius(k);
                agent_pos = current_pos{j};
                
                if static == 1
                    % 🔧 FIXED: Static cylinder distance calculation
                    % Distance from agent to cylinder surface
                    
                    % Horizontal distance to cylinder center
                    dx_2d = agent_pos(1) - obs_pos(1);
                    dy_2d = agent_pos(2) - obs_pos(2);
                    horizontal_dist = sqrt(dx_2d^2 + dy_2d^2);
                    
                    if agent_pos(3) <= obs_pos(3)
                        % Agent is below or at obstacle height
                        if horizontal_dist <= obs_radius
                            % Agent is INSIDE the cylinder - calculate distance to edge
                            dist_to_cylinder_edge = obs_radius - horizontal_dist;
                             % Very close but not zero
                        else
                            % Agent is OUTSIDE the cylinder - distance to surface
                            actual_distance = horizontal_dist - obs_radius;
                        end
                    else
                        % Agent is ABOVE the obstacle
                        if horizontal_dist <= obs_radius
                            % Agent is directly above cylinder
                            actual_distance = agent_pos(3) - obs_pos(3);
                        else
                            % Agent is above and to the side - 3D distance to corner
                            edge_point = [obs_pos(1) + obs_radius * (dx_2d/horizontal_dist);
                                         obs_pos(2) + obs_radius * (dy_2d/horizontal_dist);
                                         obs_pos(3)];
                            actual_distance = norm(agent_pos - edge_point);
                        end
                    end
                else
                    % 🔧 FIXED: Dynamic sphere distance calculation
                    center_to_center = norm(agent_pos - obs_pos);
                    actual_distance = center_to_center - 0.25;
                end
                
                % 🔧 Ensure minimum distance but don't artificially inflate
                
                
                % Track minimum distance to any obstacle
                min_dist_to_any_obstacle = min(min_dist_to_any_obstacle, actual_distance);
                
                % Store individual obstacle distance
                if ~exist('distance_obs_all', 'var')
                    distance_obs_all = zeros(N, M_obs, iteration);
                end
                distance_obs_all(j, k, i) = actual_distance;
                
                % 🔧 DEBUG: Print suspicious distances
                if actual_distance < 0.1 && mod(i, 100) == 0
                    fprintf('⚠️  Agent %d very close to Obstacle %d: dist=%.3f at iter %d\n', ...
                        j, k, actual_distance, i);
                end
            end
        end
        
        % Store the minimum distance to any obstacle
        distance_obs(j, i) = min_dist_to_any_obstacle;
        
    else
        % 🔧 AGENT HAS STOPPED: Freeze distances properly
        if target_reached_time(j) > 0
            stop_iteration = round(target_reached_time(j) / dt);
            stop_iteration = max(1, min(stop_iteration, i-2));
            
            if stop_iteration > 0 && stop_iteration <= size(distance_obs, 2)
                % Copy the ACTUAL calculated distance from when it stopped
                frozen_distance = distance_obs(j, stop_iteration);
                
                % 🔧 Only use frozen distance if it's reasonable
                if frozen_distance > 0 && isfinite(frozen_distance)
                    distance_obs(j, i) = frozen_distance;
                    
                    % Also freeze individual obstacle distances
                    if exist('distance_obs_all', 'var')
                        for k = 1:M_obs
                            if stop_iteration <= size(distance_obs_all, 3)
                                frozen_k = distance_obs_all(j, k, stop_iteration);
                                if frozen_k > 0 && isfinite(frozen_k)
                                    distance_obs_all(j, k, i) = frozen_k;
                                end
                            end
                        end
                    end
                else
                    fprintf('⚠️  Invalid frozen distance for Agent %d: %.6f\n', j, frozen_distance);
                    distance_obs(j, i) = 1.0; % Reasonable fallback
                end
            end
        end
    end
    
end
        % Record inter-agent distances
        for a = 1:N
            for b = a+1:N
                d = norm(current_pos{a} - current_pos{b});
                distance_agents(a,b,i) = d;
                distance_agents(b,a,i) = d;
            end
        end
        
        % Update display
        subtitlehandle.String = sprintf('Iteration %d', i);
        %% Save Replay Data
    %% Save Replay Data (FIXED VERSION - Replace your existing section)
if SAVE_3D_REPLAY
    replay_frame_count = replay_frame_count + 1;
    
    % Save agent data
    for j = 1:N
        replay_data.dynamic.agent_positions(j, :, replay_frame_count) = current_pos{j}';
        replay_data.dynamic.agent_orientations(j, :, replay_frame_count) = state{j}(7:9)';
        replay_data.dynamic.agent_assignments(j, replay_frame_count) = assigned_goal(j);
        replay_data.dynamic.agent_status(j, replay_frame_count) = done(j);
        replay_data.dynamic.agent_crashed(j, replay_frame_count) = crashed_agents(j);
        
        % Save trajectory points
        if replay_frame_count == 1
            replay_data.dynamic.trajectories{j} = current_pos{j}';
        else
            replay_data.dynamic.trajectories{j}(end+1, :) = current_pos{j}';
        end
    end
    
    % Save target reached times
    replay_data.dynamic.target_reached_time = target_reached_time;
    
    % 🔧 FIXED: Save dynamic obstacle data with sequential spawning support
    if static == 0
        % ✅ CRITICAL: Save the current number of obstacles
        replay_data.dynamic.num_obstacles_per_frame(replay_frame_count) = M_obs;
        
        % ✅ FIXED: Save obstacle data for ALL possible obstacles (including future spawns)
        max_obs_slots = size(replay_data.dynamic.obstacle_positions, 1);
        
        for k = 1:max_obs_slots
            if k <= M_obs && k <= size(obs_states, 2)
                % This obstacle exists - save its state
                replay_data.dynamic.obstacle_positions(k, :, replay_frame_count) = obs_states(1:3, k)';
                replay_data.dynamic.obstacle_orientations(k, :, replay_frame_count) = obs_states(7:9, k)';
                replay_data.dynamic.obstacle_states(:, k, replay_frame_count) = obs_states(:, k);
                
                % Debug output for sequential spawns
                if k > original_M_obs && replay_frame_count > 1
                    prev_pos = replay_data.dynamic.obstacle_positions(k, :, replay_frame_count-1);
                    if all(prev_pos == 0) && any(obs_states(1:3, k) ~= 0)
                        fprintf('🎯 RECORDING: Obstacle %d spawned at frame %d, pos [%.1f, %.1f, %.1f]\n', ...
                            k, replay_frame_count, obs_states(1,k), obs_states(2,k), obs_states(3,k));
                    end
                end
            else
                % This obstacle doesn't exist yet - fill with zeros
                replay_data.dynamic.obstacle_positions(k, :, replay_frame_count) = [0, 0, 0];
                replay_data.dynamic.obstacle_orientations(k, :, replay_frame_count) = [0, 0, 0];
                if size(replay_data.dynamic.obstacle_states, 2) >= k
                    replay_data.dynamic.obstacle_states(:, k, replay_frame_count) = zeros(12, 1);
                end
            end
        end
        
        % 🔧 Debug: Print obstacle count changes
        if replay_frame_count > 1
            prev_count = replay_data.dynamic.num_obstacles_per_frame(replay_frame_count-1);
            if M_obs > prev_count
                fprintf('📈 SEQUENTIAL SPAWN: Obstacle count increased from %d to %d at frame %d\n', ...
                    prev_count, M_obs, replay_frame_count);
            end
        end
    end
end

        % drawnow;
        
        agents_reached_target = 0;
total_active_agents = 0;

% Debug output every 100 iterations
if mod(i, 100) == 0
    fprintf('\n--- Completion Check at Iteration %d ---\n', i);
end

for j = 1:N
    if ~crashed_agents(j)  % Only count non-crashed agents
        total_active_agents = total_active_agents + 1;
        
        % Check current distance to assigned target
        target_pos = target{assigned_goal(j)};
        current_dist = norm(current_pos{j} - target_pos);
        
        % Agent is considered "reached" if marked as done
        if done(j)
            agents_reached_target = agents_reached_target + 1;
            
            % Debug output every 100 iterations for reached agents
            if mod(i, 100) == 0
                fprintf('  Agent %d: REACHED Target %d (dist=%.2f, time=%.2f)\n', ...
                    j, assigned_goal(j), current_dist, target_reached_time(j));
            end
        else
            % Debug output for agents still moving (only if close)
            if current_dist < 10 && mod(i, 100) == 0
                fprintf('  Agent %d: MOVING to Target %d (dist=%.2f)\n', ...
                    j, assigned_goal(j), current_dist);
            end
        end
    end
end

% Primary completion condition: All active agents have reached their targets
mission_complete = (agents_reached_target == total_active_agents) && (total_active_agents > 0);


% Debug summary every 100 iterations
if mod(i, 100) == 0
    fprintf('  Summary: %d/%d agents reached targets\n', agents_reached_target, total_active_agents);
    % fprintf('  All targets covered: %s\n', char(all_targets_covered + "false"));
    fprintf('  Mission complete: %s\n', char(mission_complete + "false"));
end
%% 

% ===== STOP SIMULATION IF MISSION COMPLETE =====
if mission_complete
        % Calculate final timing
        final_simulation_time = toc;
        final_iteration_time = i * dt;
        
        fprintf('\n=== MISSION COMPLETED ===\n');
        
        % ADD THIS HERE:
        % Save replay data
        if SAVE_3D_REPLAY
            replay_data.actual_iterations = replay_frame_count;
            replay_data.final_simulation_time = final_simulation_time;
            replay_data.final_iteration_time = final_iteration_time;
            save_replay_data(replay_data, 'completed');
        end
        
        % Your existing completion code continues...
        if mission_complete
            fprintf('✓ All active agents (%d/%d) reached their targets\n', agents_reached_target, total_active_agents);
        end
    
    
    
    fprintf('Simulation stopped at iteration %d\n', i);
    fprintf('Total simulation time: %.2f seconds (real time: %.2f seconds)\n', final_iteration_time, final_simulation_time);
    
    % ===== DETAILED TARGET COMPLETION ANALYSIS =====
    fprintf('\n=== TARGET COMPLETION DETAILS ===\n');
    for t = 1:M
        agents_on_target = find(assigned_goal == t & ~crashed_agents);
        agents_reached_target_t = agents_on_target(done(agents_on_target));
        
        quota_t = 1; % Default quota
        if exist('params', 'var') && isfield(params, 'quota') && t <= length(params.quota)
            quota_t = params.quota(t);
        end
        
        fprintf('Target %d (Quota: %d):\n', t, quota_t);
        
        if ~isempty(agents_reached_target_t)
            % Get completion times for this target
            completion_times = target_reached_time(agents_reached_target_t);
            valid_times = completion_times(completion_times > 0);
            valid_agents = agents_reached_target_t(completion_times > 0);
            
            if ~isempty(valid_times)
                [sorted_times, sort_idx] = sort(valid_times);
                sorted_agents = valid_agents(sort_idx);
                
                for idx = 1:length(sorted_agents)
                    ag = sorted_agents(idx);
                    final_dist = norm(current_pos{ag} - target{t});
                    fprintf('  → Agent %d reached at %.2f seconds (final dist: %.3f)\n', ...
                        ag, sorted_times(idx), final_dist);
                end
                
                % Target completion statistics
                if length(sorted_times) > 1
                    fprintf('  Time span: %.2f seconds (%.2f to %.2f)\n', ...
                        max(sorted_times) - min(sorted_times), min(sorted_times), max(sorted_times));
                else
                    fprintf('  Single agent completion at %.2f seconds\n', sorted_times(1));
                end
            else
                fprintf('  → Agents reached but timing data missing\n');
            end
        else
            fprintf('  → No agents reached this target\n');
        end
        fprintf('\n');
    end
    
    % ===== OVERALL MISSION STATISTICS =====
    successful_times = [];
    for j = 1:N
        if j <= length(crashed_agents) && j <= length(target_reached_time)
            if ~crashed_agents(j) && target_reached_time(j) > 0
                successful_times(end+1) = target_reached_time(j);
            end
        end
    end
    
    if ~isempty(successful_times)
        fprintf('=== OVERALL MISSION STATISTICS ===\n');
        [min_time, min_idx] = min(successful_times);
        [max_time, max_idx] = max(successful_times);
        
        % Find agent indices for min/max times
        successful_agents = [];
        for j = 1:N
            if j <= length(crashed_agents) && j <= length(target_reached_time)
                if ~crashed_agents(j) && target_reached_time(j) > 0
                    successful_agents(end+1) = j;
                end
            end
        end
        
        if ~isempty(successful_agents)
            first_agent = successful_agents(min_idx);
            last_agent = successful_agents(max_idx);
            
            fprintf('First completion: %.2f seconds (Agent %d → Target %d)\n', ...
                min_time, first_agent, assigned_goal(first_agent));
            fprintf('Last completion: %.2f seconds (Agent %d → Target %d)\n', ...
                max_time, last_agent, assigned_goal(last_agent));
            fprintf('Average completion: %.2f seconds\n', mean(successful_times));
            fprintf('Mission duration: %.2f seconds\n', max_time - min_time);
            fprintf('Success rate: %.1f%% (%d/%d agents)\n', ...
                (length(successful_times)/N)*100, length(successful_times), N);
        end
    end
    
    fprintf('=====================================\n\n');
    
    % Set iterend before breaking
    iterend = i;
    break; % Exit the main simulation loop
end

% ===== CHECK FOR USER INTERRUPTION =====
if exist('fhandle', 'var') && isvalid(fhandle) && ~isempty(fhandle.CurrentCharacter)
        fprintf('\nSimulation stopped by user at iteration %d\n', i);
        
        % ADD THIS HERE:
        % Save replay data even if interrupted
        if SAVE_3D_REPLAY
            replay_data.actual_iterations = replay_frame_count;
            save_replay_data(replay_data, 'interrupted');
        end
        
        iterend = i;
        break;
    end

% This should be the END of your main iteration loop
end % End of main "for i=1:iteration" loop
    

    if ~exist('iterend', 'var')
    iterend = iteration;  % If loop completed normally
end

disp('Simulation Finished');
if exist('titlehandle', 'var') && isvalid(titlehandle)
    titlehandle.String = sprintf('Simulation Finished - %s', apf_names{APF_type});
end
elapsedSim = toc;

% ===== FINAL MISSION RESULTS (Safe Array Access) =====
fprintf('\n=== FINAL MISSION RESULTS ===\n');

% Ensure all arrays are the correct size
required_length = N;
if length(crashed_agents) ~= required_length
    fprintf('Warning: Resizing crashed_agents array from %d to %d\n', length(crashed_agents), required_length);
    if length(crashed_agents) < required_length
        crashed_agents(end+1:required_length) = false;
    else
        crashed_agents = crashed_agents(1:required_length);
    end
end

if length(target_reached_time) ~= required_length
    fprintf('Warning: Resizing target_reached_time array from %d to %d\n', length(target_reached_time), required_length);
    if length(target_reached_time) < required_length
        target_reached_time(end+1:required_length) = 0;
    else
        target_reached_time = target_reached_time(1:required_length);
    end
end

if length(done) ~= required_length
    fprintf('Warning: Resizing done array from %d to %d\n', length(done), required_length);
    if length(done) < required_length
        done(end+1:required_length) = false;
    else
        done = done(1:required_length);
    end
end

if length(assigned_goal) ~= required_length
    fprintf('Warning: Resizing assigned_goal array from %d to %d\n', length(assigned_goal), required_length);
    if length(assigned_goal) < required_length
        assigned_goal(end+1:required_length) = 1;  % Default to target 1
    else
        assigned_goal = assigned_goal(1:required_length);
    end
end

% Display final status for each agent
fprintf('Final Agent Status:\n');
for j = 1:N
    if crashed_agents(j)
        fprintf('  Agent %d: CRASHED\n', j);
    elseif done(j) && target_reached_time(j) > 0
        final_dist = norm(current_pos{j} - target{assigned_goal(j)});
        fprintf('  Agent %d: REACHED Target %d at %.2f seconds (final dist: %.3f)\n', ...
            j, assigned_goal(j), target_reached_time(j), final_dist);
    else
        fprintf('  Agent %d: DID NOT REACH target\n', j);
    end
end

% Final statistics
total_crashed = sum(crashed_agents);
total_successful = sum(~crashed_agents & done & target_reached_time > 0);

fprintf('\nFinal Mission Summary:\n');
fprintf('  Total agents: %d\n', N);
fprintf('  Successful: %d\n', total_successful);
fprintf('  Crashed: %d\n', total_crashed);
fprintf('  Success rate: %.1f%%\n', (total_successful/N)*100);
fprintf('  Total simulation time: %.2f seconds\n', elapsedSim);
fprintf('================================\n');

% Save crash data
if exist('crash', 'var')
    save('crash_data.mat', 'crash');
end



    %% ANALYSIS OF STOPPING ACCURACY
fprintf('\n=== STOPPING ACCURACY ANALYSIS ===\n');
fprintf('Target stopping threshold (R_STOP): %.3f units\n', params.R_STOP);
fprintf('%-8s %-12s %-15s %-15s %-15s\n', 'Agent', 'Stop Time', 'Actual Dist', 'Target Pos', 'Stop Pos');
fprintf('%s\n', repmat('-', 1, 80));

for j = 1:N
    if done(j) && ~isempty(final_stopping_pos{j})
        target_pos = target{assigned_goal(j)};
        actual_dist = norm(final_stopping_pos{j} - target_pos);
        
        fprintf('%-8d %-12.2f %-15.3f [%.1f,%.1f,%.1f] [%.1f,%.1f,%.1f]\n', ...
            j, target_reached_time(j), actual_dist, ...
            target_pos(1), target_pos(2), target_pos(3), ...
            final_stopping_pos{j}(1), final_stopping_pos{j}(2), final_stopping_pos{j}(3));
    end
end
%%

% Statistics
stopped_agents = find(done);
if ~isempty(stopped_agents)
    actual_distances = zeros(length(stopped_agents), 1);
    for idx = 1:length(stopped_agents)
        j = stopped_agents(idx);
        if ~isempty(final_stopping_pos{j})
            actual_distances(idx) = norm(final_stopping_pos{j} - target{assigned_goal(j)});
        end
    end
    
    fprintf('\n=== STOPPING STATISTICS ===\n');
    fprintf('Number of agents that stopped: %d\n', length(stopped_agents));
    fprintf('Mean stopping distance: %.3f units\n', mean(actual_distances));
    fprintf('Std stopping distance: %.3f units\n', std(actual_distances));
    fprintf('Min stopping distance: %.3f units\n', min(actual_distances));
    fprintf('Max stopping distance: %.3f units\n', max(actual_distances));
    fprintf('Agents within R_STOP threshold: %d/%d (%.1f%%)\n', ...
        sum(actual_distances <= params.R_STOP), length(stopped_agents), ...
        100 * sum(actual_distances <= params.R_STOP) / length(stopped_agents));
end

    
    %% replay -> run section
    % view(xyp,zp);
    % titlehandle.String='Simulation Replay';
    % for i=1:iterend
    %     for j=1:N
    %         plottt(:,j)=[data_points{j}(i,1); data_points{j}(i,2); data_points{j}(i,3)]';
    %     end
    %     figure(1)
    %     set(h, 'XData', plottt(1,:), 'YData', plottt(2,:),'ZData', plottt(3,:))
    %     subtitlehandle.String=(sprintf('Iteration %d: black = grp 1', i));
    %     %pause(tpause);
    %     drawnow;
    % end
    % 
    % view(90,0);
    
    %% Fixed Table Creation Section
    numRows = N * iterend;
    
    Iterasi = repmat((1:iterend)', N, 1);
    Agen    = repelem((1:N)', iterend, 1);
    
    Ax = reshape(posA_log(:,1,1:iterend), [], 1);
    Ay = reshape(posA_log(:,2,1:iterend), [], 1);
    Az = reshape(posA_log(:,3,1:iterend), [], 1);
    
    if static == 0
        Ox = repmat(reshape(posO_log(1,1,1:iterend), [], 1), N, 1);
        Oy = repmat(reshape(posO_log(1,2,1:iterend), [], 1), N, 1);
        Oz = repmat(reshape(posO_log(1,3,1:iterend), [], 1), N, 1);
    else
        tmp = obstacles';
        Ox = repmat(tmp(1,1), numRows, 1);
        Oy = repmat(tmp(1,2), numRows, 1);
        Oz = repmat(tmp(1,3), numRows, 1);
    end
    
    FattX = reshape(Fatt_log(:,1,1:iterend), [], 1);
    FattY = reshape(Fatt_log(:,2,1:iterend), [], 1);
    FattZ = reshape(Fatt_log(:,3,1:iterend), [], 1);
    
    FrepX = reshape(Frep_log(:,1,1:iterend), [], 1);
    FrepY = reshape(Frep_log(:,2,1:iterend), [], 1);
    FrepZ = reshape(Frep_log(:,3,1:iterend), [], 1);
    
    T = table(Iterasi, Agen, ...
              Ax, Ay, Az, ...
              Ox, Oy, Oz, ...
              FattX, FattY, FattZ, ...
              FrepX, FrepY, FrepZ);
    
    writetable(T, 'simulation_log.csv');
    save('simulation_log.mat','T');
    
    % Legend handling for trajectory plots
    if exist('hTraj', 'var') && ~isempty(hTraj)
        % Collect all trajectory line handles for legend
        all_traj_handles = [];
        traj_labels = {};
        
        % Get one representative line per target
        for t = 1:M
            % Find an agent with this target
            agents_with_target = find(assigned_goal == t);
            if ~isempty(agents_with_target)
                ag = agents_with_target(1);
                if ~isempty(hTraj{ag})
                    % Find a line segment with this target's color
                    for seg = 1:length(hTraj{ag})
                        if isvalid(hTraj{ag}(seg))
                            line_color = get(hTraj{ag}(seg), 'Color');
                            if all(abs(line_color - cmap(t,:)) < 0.01)
                                all_traj_handles(end+1) = hTraj{ag}(seg);
                                traj_labels{end+1} = sprintf('Target %d traj', t);
                                break;
                            end
                        end
                    end
                end
            end
        end
        
        % Add legend based on obstacle type
        if static == 0 && exist('hObsQuad', 'var') && iscell(hObsQuad) && ~isempty(hObsQuad)
            % Dynamic obstacles - get first obstacle handle
            if ~isempty(all_traj_handles) && length(hObsQuad) >= 1 && isvalid(hObsQuad{1})
                legend([all_traj_handles, hObsQuad{1}], [traj_labels, {'Dynamic Obstacle'}], 'Location', 'northeast');
            elseif length(hObsQuad) >= 1 && isvalid(hObsQuad{1})
                legend(hObsQuad{1}, {'Dynamic Obstacle'}, 'Location', 'northeast');
            elseif ~isempty(all_traj_handles)
                legend(all_traj_handles, traj_labels, 'Location', 'northeast');
            end
        elseif ~isempty(all_traj_handles)
            % Static obstacles or no obstacles visible
            legend(all_traj_handles, traj_labels, 'Location', 'northeast');
        end
    end
    
    for t=1:M
      grp = find(assigned_goal==t);
      fprintf('Agen di Target %d: %s\n', t, mat2str(grp));
    end
    
   
    %% Plotting Section
    %% → Hitung panjang lintasan tiap agen (HANYA sampai agen berhenti)
path_lengths = zeros(N,1);

for j = 1:N
    % Tentukan sampai iterasi mana agen ini bergerak
    if done(j) && target_reached_time(j) > 0
        % Agen sudah berhenti - hitung sampai waktu berhenti saja
        stop_iteration = round(target_reached_time(j) / dt);
        end_iter = min([stop_iteration, iterend, size(data_points{j}, 1)]);
        
        fprintf('Agent %d stopped at iteration %d (time %.2fs)\n', j, stop_iteration, target_reached_time(j));
    else
        % Agen belum berhenti - hitung sampai akhir simulasi
        end_iter = min([iterend, size(data_points{j}, 1)]); 
        
        fprintf('Agent %d did not stop - calculating until simulation end (iter %d)\n', j, end_iter);
    end
    
    if end_iter > 1
        % Ambil posisi dari awal sampai agen berhenti
        pts = data_points{j}(1:end_iter, :);  % ukuran end_iter×3
        
        % Hapus posisi yang invalid (zeros atau NaN)
        valid_rows = ~any(isnan(pts), 2) & ~all(pts == 0, 2);
        pts = pts(valid_rows, :);
        
        if size(pts, 1) > 1
            deltas = diff(pts, 1, 1);              % (end_iter-1)×3 perbedaan posisi
            dists = sqrt(sum(deltas.^2, 2));       % jarak Euclid per langkah
            path_lengths(j) = sum(dists);          % total jarak tempuh
        else
            path_lengths(j) = 0;  % Tidak ada gerakan
        end
    else
        path_lengths(j) = 0;  % Tidak ada data
    end
end

% Tampilkan hasil dengan informasi tambahan
fprintf('\n=== Panjang Lintasan per Agen (Sampai Berhenti) ===\n');
fprintf('%-8s %-15s %-12s %-15s\n', 'Agent', 'Path Length', 'Stop Time', 'Status');
fprintf('%s\n', repmat('-', 1, 60));

for j = 1:N
    if done(j) && target_reached_time(j) > 0
        status = sprintf('STOPPED (%.1fs)', target_reached_time(j));
        stop_time_str = sprintf('%.2f s', target_reached_time(j));
    else
        status = 'NOT STOPPED';
        stop_time_str = 'N/A';
    end
    
    fprintf('%-8d %-15.2f %-12s %-15s\n', j, path_lengths(j), stop_time_str, status);
end

    %%
    % Plotting Figure All
   %% Fixed Inter-Agent Distance Plot with Simulation Stop Indicators
figure;
nrow = ceil(sqrt(N));
ncol = ceil(N/nrow);

% Calculate simulation stop time
simulation_stop_time = iterend * dt;

for i = 1:N
    subplot(nrow, ncol, i);
    hold on; grid on;
    
    % ✅ FIX: Properly initialize line_handles as cell array
    line_handles = {};
    line_colors = {};
    other_agents = setdiff(1:N, i);
    
    for idx = 1:length(other_agents)
        k = other_agents(idx);
        dik = squeeze(distance_agents(i, k, 1:iterend));
        
        % ✅ FIX: Store handle and color properly
        h = plot(Time(1:iterend), dik, 'LineWidth', 3);
        line_handles{idx} = h;
        line_colors{idx} = h.Color;  % Store the color
    end
    
    % Add simulation stop indicator
    ymax = 0;
    for idx = 1:length(other_agents)
        k = other_agents(idx);
        current_max = max(squeeze(distance_agents(i, k, 1:iterend)));
        ymax = max(ymax, current_max);
    end
    
    if ymax == 0
        ymax = 100; % Default max if no data
    end
    
    % Vertical line at simulation stop
    plot([simulation_stop_time, simulation_stop_time], [0, ymax], 'r--', ...
        'LineWidth', 2);
    
    % Add text annotation for simulation stop
    text(simulation_stop_time + 0.5, ymax * 0.9, ...
        sprintf('SIM STOP\n%.1fs', simulation_stop_time), ...
        'FontSize', 9, 'Color', 'red', 'FontWeight', 'bold', ...
        'HorizontalAlignment', 'left');
    
    % Mark individual agent stopping times
    if done(i) && target_reached_time(i) > 0
        agent_stop_time = target_reached_time(i);
        
        % Mark agent stop time with different color
        plot([agent_stop_time, agent_stop_time], [0, ymax], 'g:', ...
            'LineWidth', 4);
        
        % Add agent stop annotation
        text(agent_stop_time, ymax * 0.8, ...
            sprintf('A%d STOP\n%.1fs', i, agent_stop_time), ...
            'FontSize', 9, 'Color', 'green', 'FontWeight', 'bold', ...
            'HorizontalAlignment', 'center');
        
        % Mark final distances to other agents at stopping time
        stop_iteration = round(agent_stop_time / dt);
        if stop_iteration <= iterend
            for idx = 1:length(other_agents)
                k = other_agents(idx);
                final_distance = distance_agents(i, k, stop_iteration);
                
                % ✅ FIX: Use stored color instead of dot indexing
                if idx <= length(line_colors)
                    marker_color = line_colors{idx};
                else
                    marker_color = [0 0 1]; % Default blue
                end
                
                % Mark final distance point
                plot(agent_stop_time, final_distance, 'o', ...
                    'Color', marker_color, 'MarkerSize', 9, ...
                    'MarkerFaceColor', marker_color, ...
                    'MarkerEdgeColor', 'white');
                
                % Add distance value annotation (only for very close agents)
                if final_distance < 10
                    text(agent_stop_time + 1, final_distance + 1, ...
                        sprintf('%.1f', final_distance), ...
                        'FontSize', 9, 'Color', marker_color);
                end
            end
        end
    end
    
    % Set plot properties
    title(sprintf('Agen %d', i));
    % ✅ ADD LABELS TO ALL SUBPLOTS
    xlabel('Waktu (s)', 'FontSize', 12);
    ylabel('Jarak (meter)', 'FontSize', 12);  % ✅ Changed from "unit" to "meter"
    
   
    
    % ✅ FIX: Enhanced legend with proper handle access
    legend_labels = {};
    legend_handles_array = [];
    
    for idx = 1:length(other_agents)
        k = other_agents(idx);
        final_dist = distance_agents(i, k, iterend);
        
        if done(i) && target_reached_time(i) > 0
            stop_iter = round(target_reached_time(i) / dt);
            if stop_iter <= iterend && stop_iter > 0
                stop_dist = distance_agents(i, k, stop_iter);
                legend_labels{idx} = sprintf('%d↔%d (final: %.1f)', i, k, stop_dist);
            else
                legend_labels{idx} = sprintf('%d↔%d (final: %.1f)', i, k, final_dist);
            end
        else
            legend_labels{idx} = sprintf('%d↔%d (final: %.1f)', i, k, final_dist);
        end
        
        % Store handle for legend
        if idx <= length(line_handles)
            legend_handles_array(idx) = line_handles{idx};
        end
    end
    
    % Add legend with smaller font
    if ~isempty(legend_labels) && ~isempty(legend_handles_array)
        legend(legend_handles_array, legend_labels, 'Location', 'best', 'FontSize', 9);
    end
    
    % Set axis limits
    xlim([0, max(Time(iterend), simulation_stop_time) * 1.1]);
    ylim([0, ymax * 1.05]);
end

% Overall title with simulation info
sgtitle(sprintf('Jarak Inter-Agen vs Waktu | Simulasi Berhenti: %.1fs | Algoritma: %s', ...
    simulation_stop_time, apf_names{APF_type}));

%% Additional summary information (FIXED)
figure;
hold on; grid on;

% Plot showing when each agent stopped
if exist('cmap', 'var') && size(cmap, 1) >= N
    agent_colors = cmap(1:N, :);
else
    agent_colors = lines(N);
end

for j = 1:N
    if done(j) && target_reached_time(j) > 0
        % Bar showing agent stop time
        bar(j, target_reached_time(j), 'FaceColor', agent_colors(j,:), ...
            'EdgeColor', 'black', 'LineWidth', 1);
        
        % Add text annotation
        text(j, target_reached_time(j) + 1, ...
            sprintf('%.1fs', target_reached_time(j)), ...
            'HorizontalAlignment', 'center', 'FontWeight', 'bold');
    else
        % Agent didn't stop - show simulation end time
        bar(j, simulation_stop_time, 'FaceColor', [0.7 0.7 0.7], ...
            'EdgeColor', 'black', 'LineWidth', 1);
        
        text(j, simulation_stop_time + 1, 'N/A', ...
            'HorizontalAlignment', 'center', 'FontWeight', 'bold');
    end
end

% Add simulation stop line
plot([0.5, N+0.5], [simulation_stop_time, simulation_stop_time], 'r--', ...
    'LineWidth', 2);

text(N/2, simulation_stop_time + 2, ...
    sprintf('Simulation Stop: %.1fs', simulation_stop_time), ...
    'HorizontalAlignment', 'center', 'Color', 'red', 'FontWeight', 'bold');

xlabel('Agent');
ylabel('Waktu Berhenti (s)');
title('Waktu Berhenti per Agen');
xlim([0.5, N+0.5]);
ylim([0, simulation_stop_time * 1.2]);

% Add agent labels
xticks(1:N);
xticklabels(arrayfun(@(j) sprintf('A%d', j), 1:N, 'UniformOutput', false));

%% Final distance matrix at simulation end (FIXED)
figure;

% Create distance matrix at simulation end
final_distances = zeros(N, N);
for i = 1:N
    for j = 1:N
        if i ~= j
            final_distances(i, j) = distance_agents(i, j, iterend);
        end
    end
end

% Plot as heatmap
imagesc(final_distances);
colorbar;
colormap('hot');

% Add text annotations
for i = 1:N
    for j = 1:N
        if i ~= j
            text(j, i, sprintf('%.1f', final_distances(i, j)), ...
                'HorizontalAlignment', 'center', 'FontWeight', 'bold', ...
                'Color', 'white');
        end
    end
end

title(sprintf('Jarak Akhir Antar-Agen (t=%.1fs)', simulation_stop_time));
xlabel('Agent');
ylabel('Agent');

% Set axis labels
xticks(1:N);
yticks(1:N);
xticklabels(arrayfun(@(j) sprintf('A%d', j), 1:N, 'UniformOutput', false));
yticklabels(arrayfun(@(j) sprintf('A%d', j), 1:N, 'UniformOutput', false));

%% Final distance matrix at simulation end
figure;

% Create distance matrix at simulation end
final_distances = zeros(N, N);
for i = 1:N
    for j = 1:N
        if i ~= j
            final_distances(i, j) = distance_agents(i, j, iterend);
        end
    end
end

% Plot as heatmap
imagesc(final_distances);
colorbar;
colormap('hot');

% Add text annotations
for i = 1:N
    for j = 1:N
        if i ~= j
            text(j, i, sprintf('%.1f', final_distances(i, j)), ...
                'HorizontalAlignment', 'center', 'FontWeight', 'bold', ...
                'Color', 'white');
        end
    end
end

title(sprintf('Jarak Akhir Antar-Agen (t=%.1fs)', simulation_stop_time));
xlabel('Agent');
ylabel('Agent');

% Set axis labels
xticks(1:N);
yticks(1:N);
xticklabels(arrayfun(@(j) sprintf('A%d', j), 1:N, 'UniformOutput', false));
yticklabels(arrayfun(@(j) sprintf('A%d', j), 1:N, 'UniformOutput', false));
    
    
    %%
    
    % Plot tiap agen berdasarkan assigned_goal mereka:
    if ~exist('cmap', 'var')
        cmap = lines(M);
    end
    
    % Only plot simple trajectories if we don't have colored segments
    % (This is a fallback in case trajectory recording was disabled)
    if isempty(hTraj{1})
        for t = 1:M
          agents_on_t = find(assigned_goal == t);
          for ag = agents_on_t
            plot3( data_points{ag}(1:iterend-1,1), ...
                   data_points{ag}(1:iterend-1,2), ...
                   data_points{ag}(1:iterend-1,3), ...
                   'Color', cmap(t,:), 'LineWidth', 1.5 );
          end
        end
        legend(arrayfun(@(t) sprintf('Target %d',t), 1:M,'Uni',false));
    end
    
    num_steps_valid = floor(iterend / alocfreq) + 1;
    
    % Limit num_steps_valid to actual array sizes
    max_available_steps = size(assigned_history, 2);
    max_time_steps = length(time_steps);
    num_steps_valid = min(num_steps_valid, max_available_steps);
    num_steps_valid = min(num_steps_valid, max_time_steps);
    
    % Now safely trim arrays
    if num_steps_valid > 0 && num_steps_valid <= size(assigned_history, 2)
        assigned_history = assigned_history(:, 1:num_steps_valid);
    end
    
    if num_steps_valid > 0 && num_steps_valid <= size(theta_history, 2)
        theta_history = theta_history(:, 1:num_steps_valid);
    end
    
    if num_steps_valid > 0 && num_steps_valid <= length(time_steps)
        time_steps = time_steps(1:num_steps_valid);
    end
    
   


%% FIX: Maintain Constant Distance to Obstacles After Agent Stops
% Replace your obstacle distance calculation section with this:
%% Calculate distance from each agent to ALL obstacles separately
%% → Kode Pengecekan Jarak Semua Obstacle ke Agen vs Waktu (FIXED)
% Calculate distance from each agent to ALL obstacles separately
% Initialize distance matrix: [N agents × M_obs obstacles × iterations]

%% ✅ FIXED: Distance Calculation for All Obstacles
% Calculate distance from each agent to ALL obstacles separately
% Initialize distance matrix: [N agents × M_obs obstacles × iterations]
if ~exist('distance_obs_all', 'var')
    distance_obs_all = zeros(N, M_obs, iterend);
end

% Calculate distances for all agents and all obstacles
for j = 1:N
    for k = 1:M_obs
        % Only calculate new distances for moving agents
        if ~done(j) || (j == 1 && k == 1 && i == 1) % Always calculate for first iteration
            if k <= size(obstacles, 2)
                obs_pos = obstacles(:, k);
                obs_radius = radius(k);
                
                if static == 1
                    % Static cylinder
                    dx = current_pos{j}(1:2) - obs_pos(1:2);
                    dist_2d = norm(dx);
                    
                    if current_pos{j}(3) < obs_pos(3)
                        % Below obstacle height
                        dist_to_obs = max(0.1, dist_2d - obs_radius);
                    else
                        % Above obstacle
                        dist_to_obs = norm([dx; current_pos{j}(3) - obs_pos(3)]) - obs_radius;
                    end
                else
                    % Dynamic sphere
                    if exist('obstacle_types', 'var') && mixed == 1 && obstacle_types(k) == 1
                        % Static cylinder in mixed mode
                        dx = current_pos{j}(1:2) - obs_pos(1:2);
                        dist_2d = norm(dx);
                        if current_pos{j}(3) < obs_pos(3)
                            dist_to_obs = dist_2d - obs_radius;
                        else
                            dist_to_obs = norm([dx; current_pos{j}(3) - obs_pos(3)]) - obs_radius;
                        end
                    else
                        % Dynamic sphere
                        dist_to_obs = norm(current_pos{j} - obs_pos) - 0.25;
                    end
                end
                
                distance_obs_all(j, k, i) = max(0.1, dist_to_obs);
            end
        else
            % ✅ Agent has stopped - freeze distance at stopping value
            % ✅ FIXED: Agent has stopped - freeze distance properly
if target_reached_time(j) > 0
    actual_stop_iter = round(target_reached_time(j) / dt);
    actual_stop_iter = max(1, min(actual_stop_iter, iterend));
    
    % Check if we have valid data at stopping iteration
    if actual_stop_iter > 0 && actual_stop_iter <= size(distance_obs_all, 3)
        frozen_dist = distance_obs_all(j, k, actual_stop_iter);
        
        % Only use frozen distance if it's valid (> 0.01)
        if frozen_dist > 0.01 && isfinite(frozen_dist)
            distance_obs_all(j, k, i) = frozen_dist;
        else
            % Recalculate if frozen distance is invalid
            obs_pos = obstacles(:, k);
            obs_radius = radius(k);
            
            if static == 1
                dx = current_pos{j}(1:2) - obs_pos(1:2);
                dist_2d = norm(dx);
                if current_pos{j}(3) < obs_pos(3)
                    new_dist = max(0.1, dist_2d - obs_radius);
                else
                    new_dist = norm([dx; current_pos{j}(3) - obs_pos(3)]) - obs_radius;
                end
            else
                new_dist = norm(current_pos{j} - obs_pos) - obs_radius;
            end
            
            distance_obs_all(j, k, i) = max(0.1, new_dist);
            
            % Debug output
            fprintf('Recalculated distance for stopped Agent %d, Obstacle %d: %.3f\n', j, k, new_dist);
        end
    else
        % Fallback: recalculate if can't access stopping iteration
        obs_pos = obstacles(:, k);
        obs_radius = radius(k);
        
        if static == 1
            dx = current_pos{j}(1:2) - obs_pos(1:2);
            dist_2d = norm(dx);
            if current_pos{j}(3) < obs_pos(3)
                new_dist = max(0.1, dist_2d - obs_radius);
            else
                new_dist = norm([dx; current_pos{j}(3) - obs_pos(3)]) - obs_radius;
            end
        else
            new_dist = norm(current_pos{j} - obs_pos) - obs_radius;
        end
        
        distance_obs_all(j, k, i) = max(0.1, new_dist);
    end
end
        end
    end
end

       
%% Plot untuk setiap agen: jarak ke semua obstacle vs waktu
%% ✅ FIXED: Plot untuk setiap agen dengan legend yang benar
obstacle_colors = lines(M_obs);  % Different color for each obstacle

for j = 1:N
    figure;
    hold on; grid on;
    
    % ✅ Store line handles properly for correct legend
    h_lines = [];
    obstacle_labels = {};
    
    for k = 1:M_obs
        distances_to_k = squeeze(distance_obs_all(j, k, 1:iterend));
        
        % Plot line for this obstacle and STORE the handle
        h_k = plot(Time(1:iterend), distances_to_k, '-', ...
            'Color', obstacle_colors(k, :), 'LineWidth', 2.5);
        h_lines(end+1) = h_k;  % ✅ Store actual handle
        
        % Create label for this obstacle
        if static == 0
            obstacle_labels{end+1} = sprintf('Obs Dinamis %d', k);
        else
            obstacle_labels{end+1} = sprintf('Obs Statis %d', k);
        end
    end
    
    % Mark stopping point if agent stopped
    if done(j) && target_reached_time(j) > 0
        stop_time = target_reached_time(j);
        stop_iter = round(stop_time / dt);
        
        if stop_iter > 0 && stop_iter <= iterend
            % Calculate ymax from all obstacles
            ymax = 0;
            for k = 1:M_obs
                ymax = max(ymax, max(squeeze(distance_obs_all(j, k, 1:iterend))));
            end
            
            % Add vertical line at stopping time
            plot([stop_time, stop_time], [0, ymax], 'r--', 'LineWidth', 2);
            
            % Mark stopping points for each obstacle
            for k = 1:M_obs
                stop_distance_k = distance_obs_all(j, k, stop_iter);
                
                % Mark stopping point for this obstacle
                plot(stop_time, stop_distance_k, 'o', ...
                    'Color', obstacle_colors(k, :), 'MarkerSize', 8, ...
                    'MarkerFaceColor', obstacle_colors(k, :), ...
                    'MarkerEdgeColor', 'white', 'LineWidth', 1.5);
                
                % ✅ Highlight the flat section after stopping for each obstacle
                if stop_iter < iterend
                    flat_section_time = Time(stop_iter:iterend);
                    flat_section_dist = squeeze(distance_obs_all(j, k, stop_iter:iterend));
                    
                    % Plot flat section with thicker line
                    plot(flat_section_time, flat_section_dist, '-', ...
                        'Color', obstacle_colors(k, :), 'LineWidth', 4);
                end
                
                % Add distance value annotation for close obstacles
                if stop_distance_k < 50  % Only annotate if relatively close
                    text(stop_time + 1, stop_distance_k + 2, ...
                        sprintf('%.1f', stop_distance_k), ...
                        'FontSize', 8, 'Color', obstacle_colors(k, :), ...
                        'FontWeight', 'bold');
                end
            end
            
           
        end
    end
    
    % Enhanced plot formatting
    xlabel('Waktu (s)', 'FontSize', 12, 'FontWeight', 'bold');
    ylabel('Jarak Agen → Obstakel (unit)', 'FontSize', 12, 'FontWeight', 'bold');
    title(sprintf('Jarak Agen %d ke Setiap Obstakel vs Waktu', j), ...
        'FontSize', 14, 'FontWeight', 'bold');
    
    % Add grid and better formatting
    grid on;
    set(gca, 'GridAlpha', 0.3, 'FontSize', 11);
    
    % Enhanced legend with obstacle information
    if ~isempty(h_lines)
        legend_entries = obstacle_labels;
        
        % Add final distances to legend
        for k = 1:M_obs
            final_dist = distance_obs_all(j, k, iterend);
            legend_entries{k} = sprintf('%s ', obstacle_labels{k});
        end
        
        legend(h_lines, legend_entries, 'Location', 'best', 'FontSize', 10);
    end
    F
    % Set y-axis to start from 0 for better visualization
    all_distances = squeeze(distance_obs_all(j, :, 1:iterend));
    ymax = max(all_distances(:));
    ylim([0, ymax * 1.1]);
    xlim([0, Time(iterend) * 1.05]);
end
%% Summary Plot: All Agents vs Closest Obstacle
figure;
hold on; grid on;

agent_colors = lines(N);
for j = 1:N
    % Find minimum distance to any obstacle at each time step
    min_distances = zeros(1, iterend);
    for i = 1:iterend
        min_distances(i) = min(squeeze(distance_obs_all(j, :, i)));
    end
    
    % Plot minimum distance line
    h_agent = plot(Time(1:iterend), min_distances, '-', ...
        'Color', agent_colors(j, :), 'LineWidth', 2);
    
    % Mark stopping point
    if done(j) && target_reached_time(j) > 0
        stop_time = target_reached_time(j);
        stop_iter = round(stop_time / dt);
        
        if stop_iter > 0 && stop_iter <= iterend
            stop_min_distance = min_distances(stop_iter);
            
            plot(stop_time, stop_min_distance, 'o', ...
                'Color', agent_colors(j, :), 'MarkerSize', 8, ...
                'MarkerFaceColor', agent_colors(j, :), ...
                'MarkerEdgeColor', 'white', 'LineWidth', 1.5);
        end
    end
end

xlabel('Waktu (s)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Jarak Minimum ke Obstakel (unit)', 'FontSize', 12, 'FontWeight', 'bold');
title('Jarak Minimum Setiap Agen ke Obstakel Terdekat', 'FontSize', 14, 'FontWeight', 'bold');
grid on;

% ✅ FIXED: Create proper legend with handles and status
summary_handles = [];
summary_labels = {};

% Collect handles and create labels with stop status
for j = 1:N
    if done(j) && target_reached_time(j) > 0
        stop_time = target_reached_time(j);
        stop_iter = round(stop_time / dt);
        
        if stop_iter > 0 && stop_iter <= iterend
            stop_min_distance = min_distances(stop_iter);
            summary_labels{j} = sprintf('Agen %d (stop: %.1f)', j, stop_min_distance);
        else
            summary_labels{j} = sprintf('Agen %d (berhenti)', j);
        end
    else
        final_min_distance = min_distances(iterend);
        summary_labels{j} = sprintf('Agen %d (final: %.1f)', j, final_min_distance);
    end
end

% Note: You need to store handles when creating the plot lines above
% Add this after each plot() command: summary_handles(end+1) = h_agent;
legend(summary_handles, summary_labels, 'Location', 'best', 'FontSize', 10);

%% Verification: Check that distances stay constant after stopping
fprintf('\n=== VERIFICATION: All Distances Constant After Stopping ===\n');
for j = 1:N
    if done(j) && target_reached_time(j) > 0
        stop_iter = round(target_reached_time(j) / dt);
        
        if stop_iter > 0 && stop_iter < iterend
            fprintf('Agent %d stopped at iteration %d:\n', j, stop_iter);
            
            for k = 1:M_obs
                % Check if distance to obstacle k remains constant
                check_end = min(iterend, stop_iter + 10);
                distances_after = squeeze(distance_obs_all(j, k, stop_iter:check_end));
                
                is_constant = all(abs(distances_after - distances_after(1)) < 0.001);
                
                fprintf('  Obstacle %d: Distance = %.3f, Constant = %s\n', ...
                    k, distances_after(1), char(is_constant + "false"));
                
                if ~is_constant
                    fprintf('    ⚠️  Range: %.3f to %.3f\n', min(distances_after), max(distances_after));
                end
            end
            fprintf('\n');
        end
    end
end


    %% Agent Speed Plot
    figure;
    subplot(2,1,1);
    hold on; grid on;
    for jj = 1:N
        plot(Time(1:iterend), agent_speeds(jj,1:iterend), 'LineWidth',1.5);
    end
    xlabel('Waktu (s)');
    ylabel('Kecepatan (m/s)');
    title('Kecepatan Agen vs Waktu');
    legend(arrayfun(@(j) sprintf('Agen %d',j), 1:N, 'UniformOutput',false), ...
           'Location','northeastoutside');
    ylim([0, max(max(agent_speeds(:,1:iterend)))*1.1]);
    
    % Obstacle Speed Plot (if dynamic)
    if static == 0 && ~isempty(dynamic_obs_indices)
        subplot(2,1,2);
        hold on; grid on;
        for k = 1:length(dynamic_obs_indices)
            plot(Time(1:iterend), obstacle_speeds(k,1:iterend), 'r-', 'LineWidth',2);
        end
        xlabel('Waktu (s)');
        ylabel('Kecepatan (m/s)');
        title(sprintf('Kecepatan Obstacle Dinamis (Target: %.1f m/s)', obstacle_speed));
        if mixed == 1
            legend(arrayfun(@(k) sprintf('Dynamic Obs %d',k), 1:length(dynamic_obs_indices), 'UniformOutput',false), ...
                   'Location','northeastoutside');
        else
            legend(arrayfun(@(k) sprintf('Obstacle %d',k), 1:length(dynamic_obs_indices), 'UniformOutput',false), ...
                   'Location','northeastoutside');
        end
        ylim([0, obstacle_speed*1.5]);
        
        % Add reference line for target speed
        plot([0, Time(iterend)], [obstacle_speed, obstacle_speed], 'k--', 'LineWidth',1);
    end
    %%
    % Target assignment history
    figure;
    hold on; grid on;
    for j = 1:N
        stairs(time_steps, assigned_history(j,:), 'LineWidth', 2);
    end
    xlabel('Waktu (s)');
    ylabel('Target yang Dituju');
    title('Riwayat Penugasan Target per Agen');
    legend(arrayfun(@(j) sprintf('Agen %d', j), 1:N, 'UniformOutput', false));
    yticks(1:M);
    ylim([0.9, M + 0.1]);
    
   
  
%% FINAL FIX: Distance to Target Calculation
actual_iterations = min(iterend, iteration);
dist_to_target = zeros(N, actual_iterations);

% Create agent target history
agent_target_history = zeros(N, actual_iterations);

% Fill the target history based on allocation frequency
for i = 1:actual_iterations
    if exist('assigned_history', 'var') && size(assigned_history, 2) > 0
        alloc_step = min(floor((i-1)/alocfreq) + 1, size(assigned_history, 2));
        alloc_step = max(1, alloc_step);
        
        for j = 1:N
            agent_target_history(j, i) = assigned_history(j, alloc_step);
        end
    else
        for j = 1:N
            agent_target_history(j, i) = assigned_goal(j);
        end
    end
end

% Calculate distances using the correct position data
for j = 1:N
    for i = 1:actual_iterations
        % ✅ FIX: Use the correct position source
        if done(j) && target_reached_time(j) > 0
            stop_iteration = round(target_reached_time(j) / dt);
            
            if i >= stop_iteration
                % ✅ For stopped agents AFTER stopping: use final_stopping_pos
                if exist('final_stopping_pos', 'var') && ~isempty(final_stopping_pos{j})
                    pos = final_stopping_pos{j}';  % Convert to row vector [1x3]
                else
                    % Fallback to data_points if final_stopping_pos not available
                    if i <= size(data_points{j}, 1) && ~isempty(data_points{j}(i,:)) && any(data_points{j}(i,:) ~= 0)
                        pos = data_points{j}(i, :);
                    else
                        % Use last valid position
                        if i > 1
                            pos = final_stopping_pos{j}';
                        else
                            pos = start{j}';
                        end
                    end
                end
            else
                % ✅ For agents BEFORE stopping: use data_points
                if i <= size(data_points{j}, 1) && ~isempty(data_points{j}(i,:)) && any(data_points{j}(i,:) ~= 0)
                    pos = data_points{j}(i, :);
                else
                    % Use last valid position or interpolate
                    if i > 1 && i-1 <= size(dist_to_target, 2)
                        % Keep previous distance
                        dist_to_target(j, i) = dist_to_target(j, i-1);
                        continue;
                    else
                        pos = start{j}';
                    end
                end
            end
        else
            % ✅ For agents that haven't stopped: use data_points
            if i <= size(data_points{j}, 1) && ~isempty(data_points{j}(i,:)) && any(data_points{j}(i,:) ~= 0)
                pos = data_points{j}(i, :);
            else
                % Use last valid distance or start position
                if i > 1 && i-1 <= size(dist_to_target, 2)
                    dist_to_target(j, i) = dist_to_target(j, i-1);
                    continue;
                else
                    pos = start{j}';
                end
            end
        end
        
        % Get target assignment for this iteration
        current_target_id = agent_target_history(j, i);
        
        % Validate target ID
        if current_target_id < 1 || current_target_id > length(target)
            current_target_id = 1;
        end
        
        % ✅ Calculate distance with correct dimensions
        tgt = target{current_target_id}';  % Convert to row vector [1x3]
        calculated_distance = norm(pos - tgt);  % [1x3] - [1x3] = correct!
        
        dist_to_target(j, i) = calculated_distance;
    end
end

%% FIXED DISTANCE TO TARGET PLOT WITH PROPER LEGEND
figure;
hold on; grid on;

% Store line handles for proper legend
line_handles = [];
legend_labels = {};

for j = 1:N
    line_handle = plot(Time(1:actual_iterations), dist_to_target(j, :), 'LineWidth', 4);
    
    % ✅ STORE THE ACTUAL LINE HANDLE
    line_handles(end+1) = line_handle;
    
    % Mark the actual stopping point
    if done(j) && target_reached_time(j) > 0
        time_reached = target_reached_time(j);
        if time_reached <= Time(actual_iterations)
            time_reached_iter = round(time_reached / dt);
            if time_reached_iter <= actual_iterations && time_reached_iter > 0
                actual_stopping_distance = dist_to_target(j, time_reached_iter);
                
                % Mark stopping point
                plot(time_reached, actual_stopping_distance, 'o', ...
                    'Color', line_handle.Color, 'MarkerSize', 10, ...
                    'MarkerFaceColor', line_handle.Color, 'MarkerEdgeColor', 'white');
                
                % Add text annotation
                text(time_reached + (Time(actual_iterations) * 0.02), actual_stopping_distance + 5, ...
                    sprintf('STOP\n%.2f', actual_stopping_distance), ...
                    'FontSize', 8, 'Color', line_handle.Color, 'FontWeight', 'bold');
                
                % Add vertical line to show stopping time
                ymax = max(dist_to_target(j, :));
                plot([time_reached, time_reached], [0, ymax], ':', ...
                    'Color', line_handle.Color, 'LineWidth', 3);
            end
        end
    end
end

% Add reference line for R_STOP threshold
threshold_handle = plot([0, Time(actual_iterations)], [params.R_STOP, params.R_STOP], 'r--', 'LineWidth', 2);

% ✅ CREATE PROPER LEGEND WITH LINE HANDLES AND CLEAN LABELS
for j = 1:N
    if done(j) && target_reached_time(j) > 0
        stop_iter = round(target_reached_time(j) / dt);
        if stop_iter <= actual_iterations && stop_iter > 0
            actual_distance = dist_to_target(j, stop_iter);
            stop_target = agent_target_history(j, stop_iter);
            % ✅ CLEAN LEGEND FORMAT
            legend_labels{j} = sprintf('Agen %d → T%d (STOP: %.2f)', j, stop_target, actual_distance);
        else
            legend_labels{j} = sprintf('Agen %d → T%d (ERROR)', j, assigned_goal(j));
        end
    else
        legend_labels{j} = sprintf('Agen %d → T%d (MOVING)', j, assigned_goal(j));
    end
end

% Add threshold to legend
line_handles(end+1) = threshold_handle;
legend_labels{end+1} = sprintf('R_{STOP} threshold = %.1f meter', params.R_STOP);

% ✅ CREATE LEGEND WITH ACTUAL LINE HANDLES
legend(line_handles, legend_labels, 'Location', 'northeast', 'FontSize', 10);

xlabel('Waktu (s)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Jarak ke Target (meter)', 'FontSize', 12, 'FontWeight', 'bold');
title('Jarak Agen ke Target vs Waktu', 'FontSize', 14, 'FontWeight', 'bold');
grid on;

%%
%% PLOT DISTANCE FROM EACH AGENT TO ALL TARGETS
% This code works with your actual variable structure

% Your variables:
% - data_points{j}(i,:) = position history of agent j at iteration i
% - target{t} = position of target t
% - assigned_goal(j) = which target is assigned to agent j
% - actual_iterations = number of iterations that actually ran

figure;
hold on; grid on;

% Store line handles for legend
line_handles = [];
legend_labels = {};

% Define colors and line styles
target_colors = ['r', 'g', 'b', 'c', 'm', 'y', 'k'];
line_styles = {'-', '--', ':', '-.'};

% For each agent
for j = 1:N
    % For each target
    for target_idx = 1:M
        % Calculate distance from agent j to target target_idx at each iteration
        distances_to_target = zeros(1, actual_iterations);
        
        for iter = 1:actual_iterations
            % Get agent position at this iteration
            agent_pos = data_points{j}(iter, :);  % This is your position history
            
            % Get target position
            target_pos = target{target_idx};  % This is your target array
            
            % Calculate distance
            distances_to_target(iter) = norm(agent_pos - target_pos');
        end
        
        % Choose color and line style
        color_idx = mod(target_idx - 1, length(target_colors)) + 1;
        style_idx = mod(j - 1, length(line_styles)) + 1;
        
        % Plot the line
        if target_idx == assigned_goal(j)
            % This is the assigned target - use thick solid line
            line_handle = plot(Time(1:actual_iterations), distances_to_target, ...
                'Color', target_colors(color_idx), ...
                'LineStyle', '-', ...
                'LineWidth', 4);
            legend_text = sprintf('Agent %d → T%d (ASSIGNED)', j, target_idx);
        else
            % This is not the assigned target - use thin dashed line
            line_handle = plot(Time(1:actual_iterations), distances_to_target, ...
                'Color', target_colors(color_idx), ...
                'LineStyle', line_styles{style_idx}, ...
                'LineWidth', 2);
            legend_text = sprintf('Agent %d → T%d', j, target_idx);
        end
        
        % Store handle for legend
        line_handles(end+1) = line_handle;
        legend_labels{end+1} = legend_text;
        
        % Mark stopping point if this is the assigned target
        if target_idx == assigned_goal(j) && done(j) && target_reached_time(j) > 0
            time_reached = target_reached_time(j);
            if time_reached <= Time(actual_iterations)
                time_reached_iter = round(time_reached / dt);
                if time_reached_iter <= actual_iterations && time_reached_iter > 0
                    actual_stopping_distance = distances_to_target(time_reached_iter);
                    
                    % Mark stopping point
                    plot(time_reached, actual_stopping_distance, 'o', ...
                        'Color', target_colors(color_idx), 'MarkerSize', 10, ...
                        'MarkerFaceColor', target_colors(color_idx), 'MarkerEdgeColor', 'white');
                    
                    % Add text annotation
                    text(time_reached + (Time(actual_iterations) * 0.02), actual_stopping_distance + 5, ...
                        sprintf('STOP\n%.2f', actual_stopping_distance), ...
                        'FontSize', 8, 'Color', target_colors(color_idx), 'FontWeight', 'bold');
                end
            end
        end
    end
end

% Add reference line for R_STOP threshold
threshold_handle = plot([0, Time(actual_iterations)], [params.R_STOP, params.R_STOP], 'k--', 'LineWidth', 2);
line_handles(end+1) = threshold_handle;
legend_labels{end+1} = sprintf('R_{STOP} threshold = %.1f meter', params.R_STOP);

% Create legend
legend(line_handles, legend_labels, 'Location', 'northeast', 'FontSize', 8);
xlabel('Waktu (s)', 'FontSize', 12, 'FontWeight', 'bold');
ylabel('Jarak ke Target (meter)', 'FontSize', 12, 'FontWeight', 'bold');
title('Jarak Semua Agen ke Semua Target vs Waktu', 'FontSize', 14, 'FontWeight', 'bold');
grid on;
%%
%% SEPARATE SUBPLOT FOR EACH AGENT
figure;

for j = 1:N
    subplot(ceil(sqrt(N)), ceil(sqrt(N)), j);
    hold on; grid on;
    
    line_handles = [];
    legend_labels = {};
    
    % Plot distance to each target
    for target_idx = 1:M
        % Calculate distance from agent j to target target_idx
        distances_to_target = zeros(1, actual_iterations);
        
        for iter = 1:actual_iterations
            % Get agent position at this iteration
            agent_pos = data_points{j}(iter, :);  % Your position history
            
            % Get target position
            target_pos = target{target_idx};  % Your target array
            
            % Calculate distance
            distances_to_target(iter) = norm(agent_pos - target_pos');
        end
        
        % Different style for assigned vs non-assigned targets
        if target_idx == assigned_goal(j)
            line_handle = plot(Time(1:actual_iterations), distances_to_target, ...
                'LineWidth', 4, 'Color', 'r');
            legend_labels{end+1} = sprintf('T%d (ASSIGNED)', target_idx);
        else
            line_handle = plot(Time(1:actual_iterations), distances_to_target, ...
                'LineWidth', 2, 'Color', [0.5, 0.5, 0.5]);
            legend_labels{end+1} = sprintf('T%d', target_idx);
        end
        
        line_handles(end+1) = line_handle;
        
        % Mark stopping point for assigned target
        if target_idx == assigned_goal(j) && done(j) && target_reached_time(j) > 0
            time_reached = target_reached_time(j);
            if time_reached <= Time(actual_iterations)
                time_reached_iter = round(time_reached / dt);
                if time_reached_iter <= actual_iterations && time_reached_iter > 0
                    actual_stopping_distance = distances_to_target(time_reached_iter);
                    plot(time_reached, actual_stopping_distance, 'ro', ...
                        'MarkerSize', 8, 'MarkerFaceColor', 'r', 'MarkerEdgeColor', 'white');
                end
            end
        end
    end
    
    % Add threshold line
    threshold_handle = plot([0, Time(actual_iterations)], [params.R_STOP, params.R_STOP], 'k--', 'LineWidth', 1);
    line_handles(end+1) = threshold_handle;
    legend_labels{end+1} = sprintf('R_{STOP} = %.1f', params.R_STOP);
    
    % Formatting
    title(sprintf('Agent %d', j), 'FontSize', 12, 'FontWeight', 'bold');
    xlabel('Waktu (s)', 'FontSize', 10);
    ylabel('Jarak (m)', 'FontSize', 10);
    legend(line_handles, legend_labels, 'Location', 'best', 'FontSize', 8);
    grid on;
end

sgtitle('Jarak Setiap Agen ke Semua Target', 'FontSize', 14, 'FontWeight', 'bold');
%% Verification
fprintf('\n=== DISTANCE CALCULATION VERIFICATION ===\n');
for j = 1:N
    if done(j) && target_reached_time(j) > 0
        stop_iter = round(target_reached_time(j) / dt);
        if stop_iter <= actual_iterations && stop_iter > 0
            calculated_distance = dist_to_target(j, stop_iter);
            target_at_stop = agent_target_history(j, stop_iter);
            
            % Manual verification
            if exist('final_stopping_pos', 'var') && ~isempty(final_stopping_pos{j})
                manual_distance = norm(final_stopping_pos{j} - target{target_at_stop});
                
                fprintf('Agent %d:\n', j);
                fprintf('  Calculated distance: %.3f units\n', calculated_distance);
                fprintf('  Manual verification: %.3f units\n', manual_distance);
                fprintf('  Match: %s\n', char((abs(calculated_distance - manual_distance) < 0.01) + "false"));
                fprintf('  Status: %s\n', char((calculated_distance <= params.R_STOP) + "OVER" + "✓ PASS"));
                fprintf('\n');
            end
        end
    end
end

%% LIntasan Lebih Tebal
    %% Alternative Timeline Visualization (using thick lines instead of rectangles)
figure;
hold on; grid on;
for j = 1:N
    assignment_changes = find(diff([0, assigned_history(j,:)]) ~= 0);
    
    y_pos = j;
    for k = 1:length(assignment_changes)
        start_time = time_steps(assignment_changes(k));
        if k < length(assignment_changes)
            end_time = time_steps(assignment_changes(k+1));
        else
            end_time = Time(iterend);
        end
        
        target_id = assigned_history(j, assignment_changes(k));
        if target_id > 0 && target_id <= size(cmap,1)
            col = cmap(target_id,:);
        else
            col = [0.5 0.5 0.5];
        end
        
        % Use thick line instead of rectangle
        if start_time < end_time
            plot([start_time, end_time], [y_pos, y_pos], 'Color', col, 'LineWidth', 8);
        end
    end
end

% Add target labels and formatting
for t = 1:M
    text(Time(iterend)*1.02, 0.5-t*0.3, sprintf('Target %d', t), ...
         'Color', cmap(t,:), 'FontWeight', 'bold', 'FontSize', 10);
end

xlabel('Time (s)');
ylabel('Agent');
title('Agent Target Assignment Timeline (Color = Assigned Target)');
yticks(1:N);
yticklabels(arrayfun(@(j) sprintf('Agent %d', j), 1:N, 'UniformOutput', false));
ylim([0.5, N+0.5]);
xlim([0, Time(iterend)*1.05]);
    
  
    %% COMPLETE TRAJECTORY PLOT - REVISED VERSION (PLACE AROUND LINE 1800-1900)
figure;
clf; % Clear figure completely to avoid artifacts
hold on; grid on;

% Plot obstacles based on mode
if static == 1
    % Static mode - all obstacles are cylinders
    for i = 1:M_obs
        [h1,~,~] = create_cylinder(radius(i), Cpos(i,:), [0.45, 0.58, 0.76]);
    end
else
    % Dynamic or mixed mode
    if mixed == 1
        % Plot static obstacles as cylinders
        for idx = 1:length(static_obs_indices)
            i = static_obs_indices(idx);
            [h1,~,~] = create_cylinder(radius(i), Cpos(idx,:), [0.45, 0.58, 0.76]);
        end
    end
end

% Plot targets as POINTS ONLY (no cylinders)
% FIXED TARGET PLOTTING - Replace your existing target section
for t = 1:M
    % Large circular marker for target
    plot3(target{t}(1), target{t}(2), target{t}(3), 'o', ...
          'MarkerSize', 20, 'MarkerFaceColor', colors(t,:), ...
          'MarkerEdgeColor', 'black', 'LineWidth', 3);
    
    % X marker for emphasis
    plot3(target{t}(1), target{t}(2), target{t}(3), 'kx', ...
          'MarkerSize', 15, 'LineWidth', 3);
    
    % Target label
    text(target{t}(1)+5, target{t}(2)+5, target{t}(3)+5, ...
         sprintf('T%d', t), 'FontSize', 12, 'FontWeight', 'bold', ...
         'Color', colors(t,:), 'BackgroundColor', 'white', ...
         'EdgeColor', 'black');
end
% Remove the target_handles array completely - it's not needed

% Plot agent trajectories with proper color coding
trajectory_handles = [];
trajectory_labels = {};
used_targets = [];

for j = 1:N
    % Get trajectory data up to actual end iteration
    actual_end = min(iterend, size(data_points{j}, 1));
    traj_data = data_points{j}(1:actual_end, :);
    
    % Remove any NaN or invalid data points
    valid_idx = ~any(isnan(traj_data), 2) & ~any(isinf(traj_data), 2);
    traj_data = traj_data(valid_idx, :);
    
    if size(traj_data, 1) < 2
        continue; % Skip if not enough valid points
    end
    
    % Get agent's final target assignment color
    final_target = assigned_goal(j);
    if final_target > 0 && final_target <= size(cmap,1)
        col = cmap(final_target,:);
    else
        col = [0.5 0.5 0.5]; % Gray for unassigned
    end
    
    % Plot trajectory as single colored line
    h_traj = plot3(traj_data(:,1), traj_data(:,2), traj_data(:,3), ...
                   'Color', col, 'LineWidth', 5);
    
    % Mark starting position (white square)
    plot3(start{j}(1), start{j}(2), start{j}(3), ...
          's', 'MarkerSize', 10, 'MarkerFaceColor', 'white', ...
          'MarkerEdgeColor', 'black', 'LineWidth', 2);
    
    % Mark final position (black circle)
    if length(current_pos) >= j && ~isempty(current_pos{j})
        plot3(current_pos{j}(1), current_pos{j}(2), current_pos{j}(3), ...
              'o', 'MarkerSize', 10, 'MarkerFaceColor', 'black', ...
              'MarkerEdgeColor', 'white', 'LineWidth', 2);
        
        % Agent number label
        text(current_pos{j}(1)+2, current_pos{j}(2)+2, current_pos{j}(3)+2, ...
             sprintf('A%d', j), 'FontSize', 10, 'FontWeight', 'bold', ...
             'Color', 'black', 'BackgroundColor', 'white');
    end
    
    % Store handles for legend (one per target)
    if ~ismember(final_target, used_targets) && final_target > 0
        trajectory_handles(end+1) = h_traj;
        trajectory_labels{end+1} = sprintf('Target %d Trajectory', final_target);
        used_targets(end+1) = final_target;
    end
end

% Set up the plot appearance
xlabel('X (m)'); 
ylabel('Y (m)'); 
zlabel('Z (m)');

% Dynamic title based on APF type and obstacle mode
apf_str = sprintf('(%s)', apf_names{APF_type});
if static == 1
    title_str = sprintf('Complete Agent Trajectories with Target Assignment Colors %s', apf_str);
elseif mixed == 1
    title_str = sprintf('Complete Agent Trajectories %s - Mixed Obstacles', apf_str);
else
    title_str = sprintf('Complete Agent Trajectories %s - Dynamic Obstacles', apf_str);
end
title(title_str);

% Set view and limits
view(45, 30);
axis equal;
xlim([0 200]); 
ylim([0 200]); 
zlim([0 100]);

% Clean legend (only if we have trajectory handles)
if ~isempty(trajectory_handles) && ~isempty(trajectory_labels)
    legend(trajectory_handles, trajectory_labels, 'Location', 'northeast', ...
           'FontSize', 10);
end

% Add grid for better visualization
grid on;
set(gca, 'GridAlpha', 0.3);

%%
% Get all figure handles
figs = findall(0, 'type', 'figure');

% Save each figure
for i = 1:length(figs)
    figure(figs(i)); % Make the figure active
    savefig(figs(i), sprintf('figure_%d.fig', figs(i).Number));
end
 