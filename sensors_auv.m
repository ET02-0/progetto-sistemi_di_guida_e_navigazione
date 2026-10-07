%% Posizione delle boe acustiche (LBL transponders per Bearing Only)
% Le posizioniamo agli angoli dell'area operativa in coordinate NED
P_base1_ned = [-50;   -50;   30]; 
P_base2_ned = [250;   -50;   30];
P_base3_ned = [250;   200;   30];
P_base4_ned = [-50;   200;   30];

P_bases_ned = [P_base1_ned'; P_base2_ned'; P_base3_ned'; P_base4_ned'];
N_bases = size(P_bases_ned, 1);

%% Deviazioni standard e frequenze sensori

% Accelerometro (IMU)
std_dev_acc = 0.01;  % [m/s^2] 
freq_acc = 100;      % [Hz] (1/ST)

% Giroscopio (IMU)
% Specifica 'aG': bias costante solo sui giroscopi[cite: 1]
bias_gyro = [0.015, -0.01, 0.005];  % [rad/s]
std_dev_gyro = 3*1e-4;              % [rad/s]
freq_gyro = 100;     % [Hz]

% Magnetometro (per Specifica ALG)
% Ci serve per calcolare l'Eulero fuori dal filtro assieme all'accelerometro
std_dev_mag = 0.03;  % [adimensionale, sulle componenti del campo normalizzato]
freq_mag = 100;       % [Hz]

% Sensore Acustico (Bearing Only - Specifica BO)
std_dev_BO = deg2rad(1.0); % [rad] Errore di 1 grado nella rilevazione dell'angolo
freq_BO = 100;         % [Hz] I ping acustici sono lenti

%% Matrici di Covarianza EKF (Stato a 12 Componenti)
% x = [N, E, D, VN, VE, VD, Roll, Pitch, Yaw, bgx, bgy, bgz]

% Covarianze di Processo (Q)
q_pos   = 1e-6;     % Posizione
q_vel   = 1e-4;     % Velocità
q_euler = 1e-6;     % Angoli di Eulero (Specifica E)[cite: 1]
q_bias  = 1e-6;     % Gyro Bias (Specifica aG)[cite: 1, 4]

% Matrice Q (12x12)
Q = diag([repmat(q_pos,1,3), repmat(q_vel,1,3), repmat(q_euler,1,3), repmat(q_bias,1,3)]);

% Covarianze di Misura (R)
% Per il BO avremo N misure angolari pari a N_bases[cite: 1]
R_BO = eye(N_bases) * (std_dev_BO^2);

% L'equazione ALG fornisce un "finto" Eulero misurato[cite: 1]
% La sua covarianza dipende dall'errore di magnetometro e accelerometro
std_alg_rollpitch = 0.02; % [rad]
std_alg_yaw = 0.05;       % [rad] il mag è meno preciso
R_ALG = diag([std_alg_rollpitch^2, std_alg_rollpitch^2, std_alg_yaw^2]);

%% -------------------------------------------------------------
%% GENERAZIONE MISURE SENSORI (BO, ALG, aG) PER IL PROGETTO AUV
%% -------------------------------------------------------------
disp('Generazione misure per specifiche C-F-E-aG-ALG-BO...');

% --- 1. BIAS GIROSCOPI E RUMORE IMU (Specifica aG) ---
% Usiamo std_dev_acc e std_dev_gyro definite sopra
acc_meas = zeros(length(time), 3);
gyro_meas = zeros(length(time), 3);

for i = 1:length(time)
    acc_meas(i,:) = acc_b(i,:) + std_dev_acc * randn(1,3);
    gyro_meas(i,:) = omega_b(i,:) + bias_gyro + std_dev_gyro * randn(1,3);
end

% --- 2. MISURE BEARING ONLY (Specifica BO) ---
% Usiamo P_bases_ned e N_bases definite all'inizio dello script
bearings_true = zeros(length(time), N_bases);
bearings_meas = zeros(length(time), N_bases);

for i = 1:length(time)
    for j = 1:N_bases
        % Differenza di coordinate in NED: atan2(East, North)
        delta_N = P_bases_ned(j,1) - pos_ned(i,1);
        delta_E = P_bases_ned(j,2) - pos_ned(i,2);
        
        bearings_true(i,j) = atan2(delta_E, delta_N);
        % Aggiunta rumore bianco e mantenimento angolo tra -pi e pi
        bearings_meas(i,j) = wrapToPi(bearings_true(i,j) + std_dev_BO * randn());
    end
end

% --- 3. MAGNETOMETRO (Per la specifica ALG) ---
% Simuliamo la lettura del campo magnetico terrestre proiettata in body frame
mag_n = [1; 0; 0]; 
mag_meas = zeros(length(time), 3);

for i = 1:length(time)
    % Matrice di rotazione da Nav a Body al tempo i
    C_n_b = rotmat(quat_b(i,:),'frame'); 
    
    % Proiezione del vettore campo magnetico nel Body frame
    mag_b_true = C_n_b * mag_n;
    % Aggiunta rumore bianco (std_dev_mag definita sopra)
    mag_meas(i,:) = mag_b_true' + std_dev_mag * randn(1,3);
end

% --- 4. SENSORE DI PROFONDITÀ (Pressure Gauge) ---
std_dev_depth = 0.2; % [m]
depth_meas = zeros(length(time), 1);
for i = 1:length(time)
    depth_meas(i) = pos_ned(i,3) + std_dev_depth * randn();
end
R_depth = std_dev_depth^2;

% --- 5. VERSIONI DECIMATE DI BO E ALG, ALLE FREQUENZE DICHIARATE ---
dec_BO = round(1 / (freq_BO * ST));   % campioni base (100 Hz) per ogni "ping" acustico
idx_BO = 1:dec_BO:length(time);
time_BO = time(idx_BO);

bearings_meas_BO = zeros(length(idx_BO), N_bases);
for i = 1:length(idx_BO)
    ii = idx_BO(i);
    for j = 1:N_bases
        delta_N = P_bases_ned(j,1) - pos_ned(ii,1);
        delta_E = P_bases_ned(j,2) - pos_ned(ii,2);
        bearing_true_ij = atan2(delta_E, delta_N);
        bearings_meas_BO(i,j) = wrapToPi(bearing_true_ij + std_dev_BO * randn());
    end
end

dec_ALG = round(1 / (freq_mag * ST));
idx_ALG = 1:dec_ALG:length(time);
time_ALG = time(idx_ALG);

mag_meas_ALG = zeros(length(idx_ALG), 3);
for i = 1:length(idx_ALG)
    ii = idx_ALG(i);
    C_n_b_i = rotmat(quat_b(ii,:), 'frame');
    mag_b_true_i = C_n_b_i * mag_n;
    mag_meas_ALG(i,:) = mag_b_true_i' + std_dev_mag * randn(1,3);
end

fprintf('BO decimato: %d campioni a %.1f Hz\n', ...
    length(idx_BO), freq_BO);
fprintf('ALG/magnetometro decimato: %d campioni a %.1f Hz\n', ...
    length(idx_ALG), freq_mag);

%% Fine Script
disp('--------------------------------------------------');
disp('Variabili pronte da importare come Input in Simulink:');
disp(' 1) acc_meas (Accelerometro con rumore, NO bias) - 100 Hz');
disp(' 2) gyro_meas (Giroscopio con rumore e Bias Costante) - 100 Hz');
disp(' 3) depth_meas (Profondimetro) - 100 Hz');
disp(' 4) bearings_meas (versione densa, 100 Hz - uso storico)');
disp(' 5) mag_meas (versione densa, 100 Hz - uso storico)');
disp(' 6) bearings_meas_BO + time_BO (versione decimata, 5 Hz - CONSIGLIATA)');
disp(' 7) mag_meas_ALG + time_ALG (versione decimata, 10 Hz - CONSIGLIATA)');
disp('--------------------------------------------------');