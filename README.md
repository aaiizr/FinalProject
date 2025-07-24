 Algoritma Pembagian Tugas Terdistribusi Multi Kamikaze Uav Dengan Penghindaran Halangan Dinamis Menggunakan Dynamic Artificial Potential Field
Muhammad Faiz Ramadhan
5022211013

 
Visualisasi Sistem Multi Agen menghadapi Interceptor Quadcopter yang menjadi halangan dalam penyelesaian misi. 

Gambaran Umum
Simulasi ini mengimplementasikan sistem multi-agent UAV untuk perencanaan jalur menggunakan berbagai algoritma Artificial Potential Field (APF). Sistem mendukung penghindaran obstacle, alokasi tugas, dan visualisasi 3D real-time.

Skema Keseluruhan Sistem
 
Fitur Program:
-	4 Algoritma APF: Traditional, Modified (MAPF), Velocity (VAPF), Dynamic (DAPF)
-	Multi-type Obstacles: Silinder statis, Quadcopter Dinamis
-	Alokasi Tugas: Assignment dengan sistem kuota dan distributed assignment switch
-	Visualisasi Real-time: Grafik 3D dengan model quadcopter
-	Sistem Replay: Simpan dan putar ulang simulasi lengkap
-	Analisis Komprehensif: Tracking jarak, deteksi collision, metrik performa

Alur Program :
Main Script → Setup Environment → Inisialisasi Agent, Halangan dan Target → Alokasi Tugas → Main Loop [Kalkulasi APF → Update State → Visualisasi → Deteksi Collision] → Hasil

Step Menjalankan Program
Run main.m
Search % Plotting Figure All untuk bagian plotting pada program.
Modifikasi Parameter berikut untuk mengetahui efeknya pada Program
Dokumentasi Parameter
Parameter Inti Simulasi
-	iteration 
iteration = 8000;  % Default
 		Rentang: 1000 - 20000
Deskripsi: Jumlah iterasi maksimum simulasi

-	dt 
dt = 0.05;  % Default: timestep 50ms
Rentang: 0.01 - 0.2
Deskripsi: Time step simulasi dalam detik
Dampak:
dt kecil = lebih akurat tapi lebih lambat
dt besar = lebih cepat tapi kurang stabil
Mempengaruhi akurasi integrasi dinamika

Parameter Agent Dan Target

-	N (Integer) - Jumlah Agent
N = 5;  
Rentang: 1 - 10
Deskripsi: Jumlah UAV agent dalam simulasi

-	M (Integer) - Jumlah Target
M = 5;  
Rentang: 1 - 12
Deskripsi: Jumlah lokasi target
o	Program Generasi Target:
for t = 1:M
x_rand = 190 + (195 - 190) * rand;  % X: 190-195
y_rand = 190 + (195 - 190) * rand;  % Y: 190-195  
z_rand = 60 + (65 - 60) * rand;     % Z: 60-65
target{t} = [x_rand; y_rand; z_rand];
end

Parameter Konfigurasi Obstacle
-	static 
static = 0/1; 
Nilai:
0: Obstacle dinamis 
1: Obstacle cylindrical statis seperti bangunan

-	randomize 
randomize = 1;  
Nilai:
0: Menggunakan posisi obstacle dari array Cpos
1: Generate posisi random dengan penghindaran collision
o	Program Generasi Random:
x_min = 40; x_max = 170;  % Area spawn obstacle
y_min = 40; y_max = 170;
z_min = 40; z_max = 90;

-	datang - Arah Datang Obstacle Dinamis
datang = 1/2/3; 
Nilai:
1: Obstacle datang dari depan
2: Obstacle datang dari belakang
3: Obstacle datang dari samping 
o	Program Spawning Obstacle Dinamis:
datang = 1 (Datang dari depan)
start_pos = [180+randn*10; 180+randn*10; 65+randn*3];
end_pos = [-40+randn*1; -40+randn*1; 60+randn*10];

datang = 2 (Datang dari belakang)  
start_pos = [-30+randn*1; -30+randn*1; 61+randn*2];
end_pos = [580+randn*10; 580+randn*10; 61+randn*2];

datang = 3 (Datang dari samping)
start_pos = [-10+randn*20; 190+randn*20; 57+randn*4];
end_pos = [150+randn*20; 10+randn*20; 55+randn*7];

-	centered 
centered = 1;  Posisi obstacle statis di tengah
Nilai:
0: Obstacle tersebar di area luas
1: Obstacle terpusat di area tengah
Penggunaan: Untuk testing skenario tertentu

-	nObsVec - Jumlah Obstacle 
nObsVec = 10;  
Rentang: 1 - 50
Deskripsi: Jumlah obstacle yang akan di-spawn
Untuk Obstacle Dinamis terjadi Sequential Spawning, Obstacle tambahan spawn setiap 300 iterasi

-	obstacle_speed 
obstacle_speed = 15  m/s
Rentang: 5 - 50 m/s
Deskripsi: Kecepatan target untuk obstacle dinamis

Pemilihan Algoritma APF
-	APF_type (Integer)
APF_type = 4;  % Rekomendasi: DAPF
Nilai:
  		1: Traditional APF
    		* Gaya attractive/repulsive sederhana

  		2: Modified APF (MAPF)
    		* Enhanced attractive force dengan komponen velocity

  		3: Velocity APF (VAPF)
    		* Mempertimbangkan velocity relatif
  
  		4: Dynamic APF (DAPF) 
    		* Adaptive safety distance (Du et al. 2019)
    		* Kalkulasi threat level

Parameter Gain Gaya Path Planning

-	Krep (Float) - Gain Gaya Repulsif
Krep = 10;  % Default
- Rentang: 1 - 100
- Deskripsi: Faktor scaling untuk gaya repulsi obstacle
- Dampak:
  		- Nilai rendah (1-5): Penghindaran lemah, kemungkinan collision
  		- Nilai sedang (10-20): Perilaku seimbang
  		- Nilai tinggi (30+): Penghindaran kuat, kemungkinan osilasi

-	 Katt (Float) - Gain Gaya Atraktif
Katt = 2;  % Default
- Rentang: 0.5 - 20
- Deskripsi: Faktor scaling untuk gaya tarik target
- Dampak:
  		- Nilai rendah: Konvergensi lambat ke target
  		- Nilai tinggi: Pendekatan cepat tapi potensi overshoot

-	gains_att.kdamp (Float) - Gain Damping 
gains_att.kdamp = 0.5;  % Default dalam kode
- Rentang: 0 - 5.0
- Deskripsi: Koefisien damping untuk mencegah osilasi pada gaya atraktif
- Dampak:
  		- 0: Tidak ada damping, mungkin terjadi osilasi
 		- 0.1-1.0: Damping ringan, mengurangi overshoot
  		- 1.0-3.0: Damping sedang, gerakan lebih halus
  		- >3.0: Damping kuat, gerakan lambat tapi stabil
- Rumus Matematika:
  		damping_force = kdamp * vcur
  		Fatt = F_att_p + F_att_v - damping_force
- Kapan Diterapkan:
  		- Normal flight: Damping diterapkan
  		- Collision avoidance: Damping dinonaktifkan untuk respons cepat

-	gains_att.kp (Float) - Gain Posisi Atraktif
gains_att.kp = Katt;  % Default sama dengan Katt
- Rumus: F_att_p = gains_att.kp * (goal - pos)

-	gains_att.kv (Float) - Gain Velocity Atraktif  
gains_att.kv = Katt;  % Default sama dengan Katt
- Rumus: F_att_v = gains_att.kv * (desired_velocity - vcur)

-	gains_rep.krepp (Float) - Gain Posisi Repulsif
gains_rep.krepp = Krep;  % Default sama dengan Krep
- Deskripsi: Gain untuk komponen posisi gaya repulsif

-	 gains_rep.krepv (Float) - Gain Velocity Repulsif
gains_rep.krepv = Krep;  % Default sama dengan Krep
- Deskripsi: Gain untuk komponen velocity gaya repulsif

-	desired_speed (Float)
desired_speed = 10;  % m/s
- Rentang: 1 - 30 m/s
- Deskripsi: Kecepatan jelajah target untuk agent
- Dampak:
  - Mempengaruhi waktu konvergensi
  - Kecepatan tinggi meningkatkan risiko collision
  - Harus kompatibel dengan constraint dinamika
- Nilai Real-world:
  - Quadcopter: 5-15 m/s
  - Fixed-wing: 15-30 m/s
  - Indoor: 1-5 m/s

Parameter Stopping Dan Alokasi
-	targetrad (Float) - Radius Target
targetrad = 1;  % meter
- Rentang: 1 - 5 meter
- Deskripsi: Radius Plotting di sekitar target

-	params.R_STOP (Float) - Threshold Berhenti
params.R_STOP = 1;  % meter
- Rentang: 0.5 - 10 meter
- Deskripsi: Threshold jarak untuk agent berhenti di target
- Kritis: Menentukan kriteria sukses misi
- Tuning:
  - Misi presisi: 0.5-2 meter
  - Misi survei: 2-5 meter
  - Misi transport: 5-10 meter

-	params.R_SWITCH (Float) - Jarak Switching
params.R_SWITCH = 15;  % meter
- Rentang: 5 - 50 meter
- Deskripsi: Threshold jarak untuk mempertimbangkan task switch
- Dampak: Mempengaruhi frekuensi realokasi tugas dinamis
- Hubungan: Sebaiknya R_SWITCH > R_STOP

-	params.STABILITY_DISTANCE (Float) - Jarak Stabilitas
params.STABILITY_DISTANCE = 15;  % meter
- Rentang: 5 - 30 meter
- Deskripsi: Agent dalam jarak ini tidak akan direassign
- Tujuan: Mencegah thrashing dalam assignment

-	params.COMMITMENT_DISTANCE (Float) - Jarak Komitmen
params.COMMITMENT_DISTANCE = 10;  % meter
- Rentang: 3 - 20 meter
- Deskripsi: Agent dalam jarak ini sepenuhnya berkomitmen ke target
- Tujuan: Memastikan completion yang stabil

-	params.MIN_REASSIGN_BENEFIT (Float) - Benefit Minimum Reassign
params.MIN_REASSIGN_BENEFIT = 5;  % meter
- Rentang: 1 - 20 meter
- Deskripsi: Reassignment hanya dilakukan jika benefit > threshold ini
- Tujuan: Mencegah reassignment yang tidak signifikan

Parameter DAPF Lanjutan
-	params_dist.rminsafe (Float) - Jarak Aman Minimum
params_dist.rminsafe = 10;  % meter
- Rentang: 1 - 30 meter
- Deskripsi: Jarak aman base untuk algoritma DAPF
- Dampak: Clearance minimum dari obstacle tanpa mempertimbangkan velocity

-	params_dist.kw (Float) - Bobot Velocity
params_dist.kw = 40;
- Rentang: 10 - 100
- Deskripsi: Bobot untuk komponen velocity dalam adaptive safety distance
- Dampak: Nilai tinggi meningkatkan safety distance pada kecepatan tinggi

-	params_dist.ka (Float) - Bobot Akselerasi
params_dist.ka = 10;
- Rentang: 1 - 50
- Deskripsi: Bobot untuk komponen akselerasi
- Dampak: Mempengaruhi respons terhadap perubahan akselerasi

-	params_dist.wmax (Float) - Pertimbangan Velocity Maksimum
params_dist.wmax = 15;  % m/s
- Rentang: 5 - 50 m/s
- Deskripsi: Velocity maksimum untuk kalkulasi safety distance
- Dampak: Membatasi pertumbuhan safety distance yang bergantung velocity

-	params_dist.beta (Float) - Sudut Batas
params_dist.beta = pi/6;  % 30 derajat
- Rentang: π/12 - π/3 (15° - 60°)
- Deskripsi: Sudut batas untuk region safety adaptif
- Dampak: Mendefinisikan transisi antara variable dan fixed safety distance

-	params_dist.theta1 (Float) - Sudut Koneksi
params_dist.theta1 = pi/12;  % 15 derajat
- Rentang: π/36 - π/6 (5° - 30°)
- Deskripsi: Sudut koneksi untuk kalkulasi safety distance
- Dampak: Menghaluskan transisi di connection region

Parameter Environment Dan Visualisasi
-	xyp, zp (Float) - Sudut View Kamera
xyp = 45;   % Sudut XY plane (derajat)
zp = 90;    % Sudut Z plane (derajat)
- Rentang: 0 - 360 derajat
- Deskripsi: Sudut viewing kamera default
- View Umum:
  - Top view: xyp=0, zp=90
  - Side view: xyp=90, zp=0
  - Isometric: xyp=45, zp=30

-	nRes (Integer) - Resolusi Sphere
nRes = 10;  % Default untuk obstacle dinamis
- Rentang: 5 - 50
- Deskripsi: Jumlah face untuk rendering sphere
- Dampak Performa: O(nRes²) untuk rendering grafik
Parameter Koleksi Data

-	SAVE_3D_REPLAY (Boolean)
SAVE_3D_REPLAY = true;  % Enable recording replay
- Dampak: 
  - true: Record complete simulation state (~50-200MB file)
  - false: Tidak ada capability replay tapi hemat memori

-	alocfreq (Integer) - Frekuensi Alokasi
alocfreq = 100;  % Cek switching setiap 100 iterasi
- Rentang: 10 - 500
- Deskripsi: Seberapa sering cek untuk realokasi tugas
- Dampak:
  - Nilai rendah: Lebih responsif tapi komputasi tinggi
  - Nilai tinggi: Kurang responsif tapi lebih efisien

-	SEQUENTIAL_SPAWN_INTERVAL (Integer)
SEQUENTIAL_SPAWN_INTERVAL = 300;  % Spawn setiap 300 iterasi
- Rentang: 100 - 1000
- Deskripsi: Interval spawning obstacle baru dalam mode dinamis
- Dampak: Mengontrol tingkat kesulitan yang meningkat

-	MIN_GOAL_DISTANCE_THRESHOLD (Float)
MIN_GOAL_DISTANCE_THRESHOLD = 100;  % meter
- Rentang: 50 - 200 meter
- Deskripsi: Tidak spawn obstacle jika agent dalam jarak ini ke goal
- Tujuan: Mencegah interferensi saat agent mendekati target
Parameter Fisik UAV
-	m (Float) - Massa UAV
mm = 1;  % kg
- Rentang: 0.5 - 5 kg
- Deskripsi: Massa UAV untuk kalkulasi dinamika
- Dampak: Mempengaruhi respons terhadap gaya

-	g (Float) - Gravitasi
g = 10;  % m/s²
- Nilai: Biasanya 9.81 atau 10 untuk simplifikasi
- Deskripsi: Konstanta gravitasi

-	Ix, Iy, Iz (Float) - Momen Inersia
Ix = 0.03; Iy = 0.03; Iz = 0.05;  % kg⋅m²
- Rentang: 0.01 - 0.1 kg⋅m²
- Deskripsi: Momen inersia untuk rotasi UAV
- Dampak: Mempengaruhi respons angular

-	dd (Float) - Koefisien Drag
dd = 0.7;
- Rentang: 0.1 - 2.0
- Deskripsi: Koefisien gaya gesek udara
- Dampak: Mempengaruhi peredaman gerakan

-	L (Float) - Panjang Lengan
L = 0.2;  % meter
- Rentang: 0.1 - 0.5 meter
- Deskripsi: Panjang lengan quadcopter
- Dampak: Mempengaruhi kontrol torque

-	d (Float) - Koefisien Yaw
d = 3.13e-3;
- Rentang: 1e-3 - 1e-2
- Deskripsi: Koefisien untuk gerakan yaw
- Dampak: Mempengaruhi respons yaw
Parameter Kontrol UAV
-	K1, L1 (Float) - Gain Kontrol Altitude
K1 = 20; L1 = 9;
- Deskripsi: Gain proporsional dan derivatif untuk kontrol ketinggian
- Dampak: Mempengaruhi stabilitas vertikal

-	K2, L2 (Float) - Gain Kontrol Roll
K2 = 100; L2 = 21;
- Deskripsi: Gain untuk kontrol roll
- Dampak: Mempengaruhi gerakan samping

-	K3, L3 (Float) - Gain Kontrol Pitch  
K3 = K2; L3 = L2;
- Deskripsi: Gain untuk kontrol pitch
- Dampak: Mempengaruhi gerakan maju-mundur

-	K4, L4 (Float) - Gain Kontrol Yaw
K4 = 0.09; L4 = 0.61;
- Deskripsi: Gain untuk kontrol yaw
- Dampak: Mempengaruhi rotasi horizontal
Dokumentasi Fungsi
Main Simulation Loop
for i = 1:iteration
    % Update obstacle dinamis
    % Logic alokasi tugas  
    % Loop kontrol agent
    % Deteksi collision
    % Update visualisasi
    % Logging data
end

Alur:
1. Update State Obstacle (jika dinamis)
2. Alokasi Tugas (setiap alocfreq iterasi)
3. Loop Kontrol Agent (untuk setiap agent)
4. Update Visualisasi
5. Koleksi Data

UAV_input_Ft(x, Ft, Ftp, dt)
Tujuan: Update state UAV dengan input gaya
Parameter:
- x: State saat ini [pos(3), vel(3), angles(3), ang_vel(3)]
- Ft: Vector gaya total saat ini
- Ftp: Vector gaya sebelumnya
- dt: Time step

Output: Vector state yang sudah diupdate
Model Matematis: Dinamika quadcopter 6DOF dengan konversi gaya-ke-akselerasi

pers_state_uavnkontroller(x, r)
Tujuan: Dinamika UAV dengan kontroller terintegrasi
Parameter:
- x: Vector state [12×1]
- r: Trajectory referensi [9×1]

Struktur Vector State:
x = [x, y, z,           % Posisi (1:3)
     u, v, w,           % Velocity linear (4:6)  
     roll, pitch, yaw,  % Orientasi (7:9)
     p, q, r];          % Velocity angular (10:12)

Parameter Fisik:
mm = 1;      % Massa (kg)
g = 10;      % Gravitasi (m/s²)
Ix = 0.03;   % Momen inersia X (kg⋅m²)
Iy = 0.03;   % Momen inersia Y (kg⋅m²)
Iz = 0.05;   % Momen inersia Z (kg⋅m²)
dd = 0.7;    % Koefisien drag
L = 0.2;     % Panjang lengan (m)

obs_state_update(x, a_des, dt)
Tujuan: Update state obstacle dinamis
Parameter:
- x: State obstacle [12×1]
- a_des: Akselerasi yang diinginkan [3×1]
- dt: Time step

Integrasi: Menggunakan dinamika yang sama dengan UAV tapi dengan kontroller sederhana

avoidance_multiRobot_pure_dapf(...)
Tujuan: Kalkulasi gaya DAPF utama (implementasi Du et al. 2019)

Komponen Kunci:

1. Kalkulasi Gaya Atraktif:
% Gaya atraktif posisi (Equation 18)
F_att_p = gains_att.kp * (goal - pos);

% Gaya atraktif kecepatan (Equation 19)  
desired_velocity = Vo * (goal - pos) / d_goal;
F_att_v = gains_att.kv * (desired_velocity - vcur);

% Penerapan damping
if in_collision_avoidance
    Fatt = F_att_p + F_att_v;  % Tidak ada damping
else
    damping_force = kdamp * vcur;
    Fatt = F_att_p + F_att_v - damping_force;  % Dengan damping
end

2. Adaptive Safety Distance (Equation 9):
% Variable safety distance (Equation 7)
r_var_safe = r_min_safe + (kv/(ka + wmax)) * norm_Ev * cos(delta);

% Tentukan rsafe berdasarkan sudut δ
if abs(delta) <= beta
    rsafe = r_var_safe;
elseif abs(delta) > beta && abs(delta) < (pi/2 + theta1)
    rsafe = r_connection_safe;  % Connection region
else
    rsafe = r_min_safe;  % Minimum safety
end

3. Kalkulasi Threat Level (Equation 12):
if cos(delta) > 0
    TH_level = (1/dE_actual - 1/rsafe) * norm_Ev * cos(delta);
else
    TH_level = 0;
end

4. Gaya Repulsif:
% Repulsi posisi (Equation 13)
if dE_actual < rsafe && delta >= 0 && delta <= pi/2
    F_p_rep = gains_rep.krepp * TH_level * (-dE / norm_dE);
end

% Repulsi kecepatan (Equation 14)
if dE_actual < rsafe && delta > pi/2 && delta < 3*pi/2
    F_v_rep = gains_rep.krepv * TH_level * perp_component;
end

 traditional_APF(...)
Tujuan: Implementasi potential field klasik
Model Matematis:
% Gaya atraktif
Fatt = gains_att.kp * (goal - pos) / norm(goal - pos);

% Gaya repulsif  
Frep = gains_rep.krepp * (1/d - 1/d0) * (1/d)^2 * unit_vector;

modified_APF(...)
Tujuan: APF yang ditingkatkan dengan komponen velocity
Peningkatan:
- Gaya atraktif berbasis velocity
- Repulsi yang mempertimbangkan goal
- Properti konvergensi lebih baik


 velocity_APF(...)
Tujuan: Prediksi collision berbasis velocity
Fitur Kunci:
- Pertimbangan velocity relatif
- Prediksi waktu collision
- Penghindaran berbasis trajectory

 alokasi_tugasnew(...)
Tujuan: Alokasi tugas terdistribusi dengan konsensus
Algoritma: Berdasarkan pendekatan market-based dengan protokol handshake

Komponen Kunci:

1. Penemuan Neighbor:
neighbors{i} = voronoi_neighbors(i, current_pos);

2. Pemrosesan Request:
% Pemrosesan FIFO request yang masuk
while ~isempty(requests.in{i})
    sender = requests.in{i}(1);
    if theta(sender) < theta(i) && assigned_goal(sender) ~= g_i
        requests.ack{i}(end+1) = sender;  % Terima
    else
        requests.nack{i}(end+1) = sender; % Tolak
    end
end

3. Eksekusi Swap:
% Eksekusi swap jika ACK diterima
while ~isempty(requests.ack{i})
    j = requests.ack{i}(1);
    % Swap assignment
    tmp = assigned_goal(i);
    assigned_goal(i) = assigned_goal(j);
    assigned_goal(j) = tmp;
    theta(i) = theta(i) + params.dtheta;
end

4. Resolusi Deadlock:
% Deteksi cycle dalam graph request
[found, cycle] = detect_cycle(G);
if found
    victim = cycle(find(theta(cycle) == max(theta(cycle)), 1));
    % Reassign victim ke target alternatif
end

 voronoi_neighbors(i, current_pos)
Tujuan: Cari neighbor Voronoi untuk topologi komunikasi
Algoritma: Test half-space untuk adjacency Voronoi
for j = 1:N
    if j == i, continue; end
    mid = (pos_i + current_pos{j}) / 2;
    if norm(pos_i - mid) < norm(current_pos{j} - mid)
        VN(end+1) = j;  % j adalah neighbor Voronoi dari i
    end
end

calculateAgentInteraction(...)
Tujuan: Kalkulasi gaya antar-agent
Fitur:
- Repulsi agent-ke-agent
- Maintenance formasi (opsional)
- Gaya cohesion untuk perilaku group

check_agent_near_goal(...)
Tujuan: Cek apakah ada agent dekat goal (digunakan untuk logic spawning)
Penggunaan: Mencegah spawning obstacle ketika agent dekat target

Fungsi Grafik Dan Visualisasi
create_quadcopter_graphics(pos, angles, color, scale)
Tujuan: Buat representasi visual quadcopter 3D
Komponen:
- Body central (sphere)
- Empat lengan (lines)
- Propeller (circles)
- Indikator arah


update_quadcopter_graphics(hgroup, pos, angles)
Tujuan: Update posisi dan orientasi quadcopter

 create_cylinder(Radius, pos, color)
Tujuan: Buat grafik obstacle cylindrical
Komponen:
- Permukaan curved (surf)
- Top cap (fill3)
- Bottom cap (fill3)

Fungsi Replay Dan Analisis

save_replay_data(replay_data, end_type)
Tujuan: Simpan complete simulation state untuk replay
 replay_uav_simulation(filename)
Tujuan: Load dan replay simulasi yang disimpan
Fitur:
- Reproduksi tepat visualisasi asli
- Kontrol interaktif (play/pause/speed)
- Navigasi frame-by-frame
- Grafik sama dengan simulasi asli

saturate(x, lower, upper)
Tujuan: Batasi nilai ke rentang tertentu
Penggunaan: Saturasi input kontrol, batas safety

 wrapToPi(angle)
Tujuan: Wrap sudut ke rentang [-π, π]
Penggunaan: Kalkulasi sudut dalam dinamika

 jarak3(a, b)
Tujuan: Kalkulasi jarak Euclidean 3D
Penggunaan: Kalkulasi jarak di seluruh simulasi

outer_cascade_tracking(x, r)
Tujuan: Kontrol outer loop untuk tracking trajectory
Algoritma: Cascade control untuk tracking posisi XY

