%{
%% Estrazione dati da Simulink
% "squeeze" converte da 3D (es. 12x1x20001) a 2D (12x20001)
est_data = squeeze(out.x_est); 
mech_data = squeeze(out.x_mech);

% Trasponiamo le matrici per avere il Tempo sulle righe e gli Stati sulle colonne
if size(est_data, 1) < size(est_data, 2)
    est_data = est_data';
end
if size(mech_data, 1) < size(mech_data, 2)
    mech_data = mech_data';
end

% Estrazione Posizioni
pos_est = est_data(:, 1:3);
pos_mech = mech_data(:, 1:3);

% Allineamento delle dimensioni (ora N sarà 20001!)
N = min([size(pos_ned, 1), size(pos_est, 1), size(pos_mech, 1)]);

time_plot = time(1:N);
pos_ned_plot = pos_ned(1:N, :);
pos_est_plot = pos_est(1:N, :);
pos_mech_plot = pos_mech(1:N, :);

%% Plot: Confronto Traiettoria 3D (Doppia Vista)
figure('Name', 'Confronto Navigazione 3D', 'Position', [100, 100, 1000, 500]);

% Subplot 1: Visione globale (per apprezzare la divergenza MECH)
subplot(1, 2, 1);
plot3(pos_ned_plot(:,2), pos_ned_plot(:,1), -pos_ned_plot(:,3), 'k', 'LineWidth', 2); hold on;
plot3(pos_est_plot(:,2), pos_est_plot(:,1), -pos_est_plot(:,3), 'b--', 'LineWidth', 2);
plot3(pos_mech_plot(:,2), pos_mech_plot(:,1), -pos_mech_plot(:,3), 'r:', 'LineWidth', 1.5);
grid on; % Nota: non mettiamo 'axis equal' qui per via della scala estrema
title('Visione Globale (Deriva MECH)');
xlabel('East [m]'); ylabel('North [m]'); zlabel('Up [m]');
legend('True', 'EKF', 'MECH', 'Location', 'best');

% Subplot 2: Zoom sull'area operativa (escludendo la MECH)
subplot(1, 2, 2);
plot3(pos_ned_plot(:,2), pos_ned_plot(:,1), -pos_ned_plot(:,3), 'k', 'LineWidth', 2); hold on;
plot3(pos_est_plot(:,2), pos_est_plot(:,1), -pos_est_plot(:,3), 'b--', 'LineWidth', 2);
grid on; axis equal;
title('Zoom Area Operativa (True vs EKF)');
xlabel('East [m]'); ylabel('North [m]'); zlabel('Up [m]');
legend('True Trajectory', 'EKF Estimate', 'Location', 'best');

%% Plot: Errore di Posizione nel Tempo
error_est = pos_ned_plot - pos_est_plot;
error_mech = pos_ned_plot - pos_mech_plot;

figure('Name', 'Errore di Posizione (EKF vs MECH)');
subplot(3,1,1);
plot(time_plot, error_est(:,1), 'b', time_plot, error_mech(:,1), 'r');
grid on; title('Errore Posizione NORD'); legend('EKF', 'MECH');

subplot(3,1,2);
plot(time_plot, error_est(:,2), 'b', time_plot, error_mech(:,2), 'r');
grid on; title('Errore Posizione EST');

subplot(3,1,3);
plot(time_plot, error_est(:,3), 'b', time_plot, error_mech(:,3), 'r');
grid on; title('Errore Posizione DOWN');
xlabel('Time [s]');

N_plot = min([length(time), size(est_data, 1), size(mech_data, 1)]);
t_p = time(1:N_plot);

% Stime e meccanizzazione
p_true = pos_ned(1:N_plot, :);
v_true = vel_n(1:N_plot, :);
p_ekf  = est_data(1:N_plot, 1:3);
v_ekf  = est_data(1:N_plot, 4:6);
eul_e  = est_data(1:N_plot, 7:9);

p_mech = mech_data(1:N_plot, 1:3);
v_mech = mech_data(1:N_plot, 4:6);

%% ------------------------------------------------------------------------
% FIGURA 1: MECCANIZZAZIONE PURA VS EKF (Divergenza Inerziale)
% ------------------------------------------------------------------------
figure('Name', 'Confronto Meccanizzazione Strapdown vs EKF', 'Position', [60, 60, 1200, 650]);

err_p_mech = sqrt(sum((p_true - p_mech).^2, 2));
err_p_ekf  = sqrt(sum((p_true - p_ekf).^2, 2));
err_v_mech = sqrt(sum((v_true - v_mech).^2, 2));
err_v_ekf  = sqrt(sum((v_true - v_ekf).^2, 2));

subplot(2, 1, 1);
semilogy(t_p, err_p_mech, 'r', 'LineWidth', 1.5); hold on;
semilogy(t_p, err_p_ekf,  'b', 'LineWidth', 1.5); grid on;
title('Errore 3D Posizione: Meccanizzazione Pura (Open-Loop) vs EKF');
ylabel('Errore [m] (Scala Log)');
legend('Dead Reckoning (Solo IMU)', 'EKF Integrato', 'Location', 'best');

subplot(2, 1, 2);
semilogy(t_p, err_v_mech, 'r', 'LineWidth', 1.5); hold on;
semilogy(t_p, err_v_ekf,  'b', 'LineWidth', 1.5); grid on;
title('Errore 3D Velocità: Meccanizzazione Pura vs EKF');
xlabel('Tempo [s]'); ylabel('Errore [m/s] (Scala Log)');
legend('Dead Reckoning (Solo IMU)', 'EKF Integrato', 'Location', 'best');

%% ------------------------------------------------------------------------
% FIGURA 2: NORMA DELL'ERRORE TEMPORALE (Posizione, Velocità, Assetto)
% ------------------------------------------------------------------------
figure('Name', 'Norme Euclidee degli Errori EKF', 'Position', [90, 90, 1100, 750]);

rpy_ref = rpy(1:N_plot, :);
%rpy_ref(:, 3) = -rpy_ref(:, 3);

err_rpy_rad = wrapToPi(rpy_ref - eul_e);
err_eul_norm = rad2deg(sqrt(sum(err_rpy_rad.^2, 2)));

subplot(3, 1, 1);
plot(t_p, err_p_ekf, 'b', 'LineWidth', 1.3); grid on;
title('Norma Errore Posizione ||e_p||_2');
ylabel('[m]');

subplot(3, 1, 2);
plot(t_p, err_v_ekf, 'g', 'LineWidth', 1.3); grid on;
title('Norma Errore Velocità ||e_v||_2');
ylabel('[m/s]');

subplot(3, 1, 3);
plot(t_p, err_eul_norm, 'm', 'LineWidth', 1.3); grid on;
title('Norma Errore Assetto ||e_{\eta}||_2');
xlabel('Tempo [s]'); ylabel('[deg]');

%% ------------------------------------------------------------------------
% FIGURA 3: RESIDUI INNOVAZIONE BEARING ONLY E BOUND 3-SIGMA
% ------------------------------------------------------------------------
innov_data = squeeze(out.innov);
S_diag = squeeze(out.S_cov);
if size(innov_data, 1) < size(innov_data, 2), innov_data = innov_data'; end
  
figure('Name', 'Residui Innovazione BO (Boe 1-4)', 'Position', [120, 120, 1100, 700]);
for b = 1:4
    subplot(2, 2, b);
    sigma_inn = sqrt(squeeze(S_diag(3 + b, 3 + b, 1:N_plot)));
    plot(t_p, rad2deg(innov_data(1:N_plot, 3 + b)), 'Color', [0.2 0.4 0.8]); hold on;
    plot(t_p,  3*rad2deg(sigma_inn), 'r--', 'LineWidth', 1.2);
    plot(t_p, -3*rad2deg(sigma_inn), 'r--', 'LineWidth', 1.2);
    grid on;
    title(sprintf('Innovazione Bearing Boa %d', b));
    ylabel('[deg]'); xlabel('Tempo [s]');
    if b == 1, legend('Residuo y - h(x)', '\pm 3\sigma Bound'); end
end

%% ------------------------------------------------------------------------
% FIGURA 4: TRAIETTORIA 3D CON TERNA SOLIDALE (QUIVER BODY FRAME) E BOE
% ------------------------------------------------------------------------
figure('Name', 'Scenario Operativo 3D e Orientazione Body', 'Position', [150, 150, 1000, 800]);

% Plot traiettoria ed EKF
plot3(p_true(:,2), p_true(:,1), -p_true(:,3), 'k-', 'LineWidth', 1.8); hold on;
plot3(p_ekf(:,2),  p_ekf(:,1),  -p_ekf(:,3),  'b--', 'LineWidth', 1.2);

% Plot Boe LBL
plot3(P_bases_ned(:,2), P_bases_ned(:,1), -P_bases_ned(:,3), 'r^', ...
      'MarkerSize', 10, 'MarkerFaceColor', 'r');

% Campionamento orientazione Body (Quiver)
step_q = round(N_plot / 20); % ~20 frecce lungo il percorso
scale_arrow = 15; % Lunghezza visuale del vettore

for k = 1:step_q:N_plot
    % Matrice di rotazione C_b_n da eul_est
    ph = eul_e(k,1); th = eul_e(k,2); ps = eul_e(k,3);
    C_b_n_k = [cos(th)*cos(ps), sin(ph)*sin(th)*cos(ps)-cos(ph)*sin(ps), cos(ph)*sin(th)*cos(ps)+sin(ph)*sin(ps);
               cos(th)*sin(ps), sin(ph)*sin(th)*sin(ps)+cos(ph)*cos(ps), cos(ph)*sin(th)*sin(ps)-sin(ph)*cos(ps);
              -sin(th),         sin(ph)*cos(th),                         cos(ph)*cos(th)];
    
    origin = [p_ekf(k,2), p_ekf(k,1), -p_ekf(k,3)];
    
    % Asse X Body (Surge / Prua) in Rosso
    x_b = C_b_n_k * [scale_arrow; 0; 0];
    quiver3(origin(1), origin(2), origin(3), x_b(2), x_b(1), -x_b(3), 0, 'Color', 'r', 'LineWidth', 1.5, 'MaxHeadSize', 0.8);
    
    % Asse Y Body (Sway / Dritta) in Verde
    y_b = C_b_n_k * [0; scale_arrow; 0];
    quiver3(origin(1), origin(2), origin(3), y_b(2), y_b(1), -y_b(3), 0, 'Color', 'g', 'LineWidth', 1.2, 'MaxHeadSize', 0.8);
    
    % Asse Z Body (Heave / Chiglia) in Blu
    z_b = C_b_n_k * [0; 0; scale_arrow];
    quiver3(origin(1), origin(2), origin(3), z_b(2), z_b(1), -z_b(3), 0, 'Color', 'c', 'LineWidth', 1.2, 'MaxHeadSize', 0.8);
end

grid on; axis equal;
title('Scenario Operativo 3D: Traiettoria, Boe LBL e Terne Body');
xlabel('Est [m]'); ylabel('Nord [m]'); zlabel('Quota (-Down) [m]');
legend('Ground Truth', 'EKF Stima', 'Boe LBL (Transponders)', 'X Body (Surge)', 'Location', 'best');
view([-35, 25]);
%}

%% Estrazione dati da Simulink
% "squeeze" converte da 3D (es. 12x1x20001) a 2D (12x20001)
est_data = squeeze(out.x_est); 
mech_data = squeeze(out.x_mech);

% Trasponiamo le matrici per avere il Tempo sulle righe e gli Stati sulle colonne
if size(est_data, 1) < size(est_data, 2)
    est_data = est_data';
end
if size(mech_data, 1) < size(mech_data, 2)
    mech_data = mech_data';
end

% Estrazione Posizioni
pos_est = est_data(:, 1:3);
pos_mech = mech_data(:, 1:3);

% Allineamento delle dimensioni (ora N sarà 20001!)
N = min([size(pos_ned, 1), size(pos_est, 1), size(pos_mech, 1)]);

time_plot = time(1:N);
pos_ned_plot = pos_ned(1:N, :);
pos_est_plot = pos_est(1:N, :);
pos_mech_plot = pos_mech(1:N, :);

%% Plot: Confronto Traiettoria 3D (Doppia Vista)
figure('Name', 'Confronto Navigazione 3D', 'Position', [100, 100, 1000, 500]);

% Subplot 1: Visione globale (per apprezzare la divergenza MECH)
subplot(1, 2, 1);
plot3(pos_ned_plot(:,2), pos_ned_plot(:,1), -pos_ned_plot(:,3), 'k', 'LineWidth', 2); hold on;
plot3(pos_est_plot(:,2), pos_est_plot(:,1), -pos_est_plot(:,3), 'b--', 'LineWidth', 2);
plot3(pos_mech_plot(:,2), pos_mech_plot(:,1), -pos_mech_plot(:,3), 'r:', 'LineWidth', 1.5);
grid on; % Nota: non mettiamo 'axis equal' qui per via della scala estrema
title('Visione Globale (Deriva MECH)');
xlabel('East [m]'); ylabel('North [m]'); zlabel('Up [m]');
legend('True', 'EKF', 'MECH', 'Location', 'best');

% Subplot 2: Zoom sull'area operativa (escludendo la MECH)
subplot(1, 2, 2);
plot3(pos_ned_plot(:,2), pos_ned_plot(:,1), -pos_ned_plot(:,3), 'k', 'LineWidth', 2); hold on;
plot3(pos_est_plot(:,2), pos_est_plot(:,1), -pos_est_plot(:,3), 'b--', 'LineWidth', 2);
grid on; axis equal;
title('Zoom Area Operativa (True vs EKF)');
xlabel('East [m]'); ylabel('North [m]'); zlabel('Up [m]');
legend('True Trajectory', 'EKF Estimate', 'Location', 'best');

%% Plot: Errore di Posizione nel Tempo
error_est = pos_ned_plot - pos_est_plot;
error_mech = pos_ned_plot - pos_mech_plot;

figure('Name', 'Errore di Posizione (EKF vs MECH)');
subplot(3,1,1);
plot(time_plot, error_est(:,1), 'b', time_plot, error_mech(:,1), 'r');
grid on; title('Errore Posizione NORD'); legend('EKF', 'MECH');

subplot(3,1,2);
plot(time_plot, error_est(:,2), 'b', time_plot, error_mech(:,2), 'r');
grid on; title('Errore Posizione EST');

subplot(3,1,3);
plot(time_plot, error_est(:,3), 'b', time_plot, error_mech(:,3), 'r');
grid on; title('Errore Posizione DOWN');
xlabel('Time [s]');

N_plot = min([length(time), size(est_data, 1), size(mech_data, 1)]);
t_p = time(1:N_plot);

% Stime e meccanizzazione
p_true = pos_ned(1:N_plot, :);
v_true = vel_n(1:N_plot, :);
p_ekf  = est_data(1:N_plot, 1:3);
v_ekf  = est_data(1:N_plot, 4:6);
eul_e  = est_data(1:N_plot, 7:9);

p_mech = mech_data(1:N_plot, 1:3);
v_mech = mech_data(1:N_plot, 4:6);

%% ------------------------------------------------------------------------
% FIGURA 1: MECCANIZZAZIONE PURA VS EKF (Divergenza Inerziale)
% ------------------------------------------------------------------------
figure('Name', 'Confronto Meccanizzazione Strapdown vs EKF', 'Position', [60, 60, 1200, 650]);

err_p_mech = sqrt(sum((p_true - p_mech).^2, 2));
err_p_ekf  = sqrt(sum((p_true - p_ekf).^2, 2));
err_v_mech = sqrt(sum((v_true - v_mech).^2, 2));
err_v_ekf  = sqrt(sum((v_true - v_ekf).^2, 2));

subplot(2, 1, 1);
semilogy(t_p, err_p_mech, 'r', 'LineWidth', 1.5); hold on;
semilogy(t_p, err_p_ekf,  'b', 'LineWidth', 1.5); grid on;
title('Errore 3D Posizione: Meccanizzazione Pura (Open-Loop) vs EKF');
ylabel('Errore [m] (Scala Log)');
legend('Dead Reckoning (Solo IMU)', 'EKF Integrato', 'Location', 'best');

subplot(2, 1, 2);
semilogy(t_p, err_v_mech, 'r', 'LineWidth', 1.5); hold on;
semilogy(t_p, err_v_ekf,  'b', 'LineWidth', 1.5); grid on;
title('Errore 3D Velocità: Meccanizzazione Pura vs EKF');
xlabel('Tempo [s]'); ylabel('Errore [m/s] (Scala Log)');
legend('Dead Reckoning (Solo IMU)', 'EKF Integrato', 'Location', 'best');

%% ------------------------------------------------------------------------
% FIGURA 2: NORMA DELL'ERRORE TEMPORALE (Posizione, Velocità, Assetto)
% ------------------------------------------------------------------------
figure('Name', 'Norme Euclidee degli Errori EKF', 'Position', [90, 90, 1100, 750]);

% NOTA: prima qui lo yaw della ground truth veniva negato
% (rpy_ref(:,3) = -rpy_ref(:,3)), toppa per lo stesso problema di segno
% risolto ora alla radice in trajectory_gen_cartesian.m. Con rpy corretto
% non serve piu' alcuna negazione.
rpy_ref = rpy(1:N_plot, :);

err_rpy_rad = wrapToPi(rpy_ref - eul_e);
err_eul_norm = rad2deg(sqrt(sum(err_rpy_rad.^2, 2)));

subplot(3, 1, 1);
plot(t_p, err_p_ekf, 'b', 'LineWidth', 1.3); grid on;
title('Norma Errore Posizione ||e_p||_2');
ylabel('[m]');

subplot(3, 1, 2);
plot(t_p, err_v_ekf, 'g', 'LineWidth', 1.3); grid on;
title('Norma Errore Velocità ||e_v||_2');
ylabel('[m/s]');

subplot(3, 1, 3);
plot(t_p, err_eul_norm, 'm', 'LineWidth', 1.3); grid on;
title('Norma Errore Assetto ||e_{\eta}||_2');
xlabel('Tempo [s]'); ylabel('[deg]');

%% ------------------------------------------------------------------------
% FIGURA 3: RESIDUI INNOVAZIONE BEARING ONLY E BOUND 3-SIGMA
% ------------------------------------------------------------------------
innov_data = squeeze(out.innov);
S_diag = squeeze(out.S_cov);
if size(innov_data, 1) < size(innov_data, 2), innov_data = innov_data'; end
  
figure('Name', 'Residui Innovazione BO (Boe 1-4)', 'Position', [120, 120, 1100, 700]);
for b = 1:4
    subplot(2, 2, b);
    sigma_inn = sqrt(squeeze(S_diag(3 + b, 3 + b, 1:N_plot)));
    plot(t_p, rad2deg(innov_data(1:N_plot, 3 + b)), 'Color', [0.2 0.4 0.8]); hold on;
    plot(t_p,  3*rad2deg(sigma_inn), 'r--', 'LineWidth', 1.2);
    plot(t_p, -3*rad2deg(sigma_inn), 'r--', 'LineWidth', 1.2);
    grid on;
    title(sprintf('Innovazione Bearing Boa %d', b));
    ylabel('[deg]'); xlabel('Tempo [s]');
    if b == 1, legend('Residuo y - h(x)', '\pm 3\sigma Bound'); end
end

%% ------------------------------------------------------------------------
% FIGURA 3bis: RESIDUI INNOVAZIONE ALG (Roll, Pitch, Yaw) E BOUND 3-SIGMA
% ------------------------------------------------------------------------
% Stessa logica della Fig. 3, ma sui primi 3 canali del vettore di
% misura (h_x(1:3) = [phi;theta;psi] in EKF_Update.m, le pseudo-misure
% d'assetto prodotte da Attitude_ALG.m). Prima mancava: la relazione
% validava solo il bearing (Fig. 5.1), non l'assetto, anche se l'ALG e'
% uno dei due canali di correzione del filtro.
labels_alg = {'Roll (\phi)', 'Pitch (\theta)', 'Yaw (\psi)'};
figure('Name', 'Residui Innovazione ALG (Roll/Pitch/Yaw)', 'Position', [140, 140, 1100, 380]);
tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
for c = 1:3
    nexttile;
    sigma_inn_alg = sqrt(squeeze(S_diag(c, c, 1:N_plot)));
    plot(t_p, rad2deg(innov_data(1:N_plot, c)), 'Color', [0.2 0.4 0.8]); hold on;
    plot(t_p,  3*rad2deg(sigma_inn_alg), 'r--', 'LineWidth', 1.2);
    plot(t_p, -3*rad2deg(sigma_inn_alg), 'r--', 'LineWidth', 1.2);
    grid on;
    title(sprintf('Innovazione ALG - %s', labels_alg{c}));
    ylabel('[deg]'); xlabel('Tempo [s]');
    if c == 1, legend('Residuo y - h(x)', '\pm 3\sigma Bound'); end
end

%% ------------------------------------------------------------------------
% FIGURA 3ter: RESIDUO INNOVAZIONE PROFONDITA' E BOUND 3-SIGMA
% ------------------------------------------------------------------------
% Ultimo canale mancante del vettore di misura: h_x(end) = p_D in
% EKF_Update.m, indice 8 in innov_data/S_diag.
sigma_inn_dep = sqrt(squeeze(S_diag(8, 8, 1:N_plot)));
figure('Name', 'Residuo Innovazione Profondita''', 'Position', [160, 160, 700, 380]);
plot(t_p, innov_data(1:N_plot, 8), 'Color', [0.2 0.4 0.8]); hold on;
plot(t_p,  3*sigma_inn_dep, 'r--', 'LineWidth', 1.2);
plot(t_p, -3*sigma_inn_dep, 'r--', 'LineWidth', 1.2);
grid on;
title('Innovazione Profondita'' (p_D)');
ylabel('[m]'); xlabel('Tempo [s]');
legend('Residuo y - h(x)', '\pm 3\sigma Bound');

%% ------------------------------------------------------------------------
% FIGURA 4: TRAIETTORIA 3D CON TERNA SOLIDALE (QUIVER BODY FRAME) E BOE
% ------------------------------------------------------------------------
figure('Name', 'Scenario Operativo 3D e Orientazione Body', 'Position', [150, 150, 1000, 800]);

% Plot traiettoria ed EKF
plot3(p_true(:,2), p_true(:,1), -p_true(:,3), 'k-', 'LineWidth', 1.8); hold on;
plot3(p_ekf(:,2),  p_ekf(:,1),  -p_ekf(:,3),  'b--', 'LineWidth', 1.2);

% Plot Boe LBL
plot3(P_bases_ned(:,2), P_bases_ned(:,1), -P_bases_ned(:,3), 'r^', ...
      'MarkerSize', 10, 'MarkerFaceColor', 'r');

% Campionamento orientazione Body (Quiver)
step_q = round(N_plot / 20); % ~20 frecce lungo il percorso
scale_arrow = 15; % Lunghezza visuale del vettore

for k = 1:step_q:N_plot
    % Matrice di rotazione C_b_n da eul_est
    ph = eul_e(k,1); th = eul_e(k,2); ps = eul_e(k,3);
    C_b_n_k = [cos(th)*cos(ps), sin(ph)*sin(th)*cos(ps)-cos(ph)*sin(ps), cos(ph)*sin(th)*cos(ps)+sin(ph)*sin(ps);
               cos(th)*sin(ps), sin(ph)*sin(th)*sin(ps)+cos(ph)*cos(ps), cos(ph)*sin(th)*sin(ps)-sin(ph)*cos(ps);
              -sin(th),         sin(ph)*cos(th),                         cos(ph)*cos(th)];
    
    origin = [p_ekf(k,2), p_ekf(k,1), -p_ekf(k,3)];
    
    % Asse X Body (Surge / Prua) in Rosso
    x_b = C_b_n_k * [scale_arrow; 0; 0];
    quiver3(origin(1), origin(2), origin(3), x_b(2), x_b(1), -x_b(3), 0, 'Color', 'r', 'LineWidth', 1.5, 'MaxHeadSize', 0.8);
    
    % Asse Y Body (Sway / Dritta) in Verde
    y_b = C_b_n_k * [0; scale_arrow; 0];
    quiver3(origin(1), origin(2), origin(3), y_b(2), y_b(1), -y_b(3), 0, 'Color', 'g', 'LineWidth', 1.2, 'MaxHeadSize', 0.8);
    
    % Asse Z Body (Heave / Chiglia) in Blu
    z_b = C_b_n_k * [0; 0; scale_arrow];
    quiver3(origin(1), origin(2), origin(3), z_b(2), z_b(1), -z_b(3), 0, 'Color', 'c', 'LineWidth', 1.2, 'MaxHeadSize', 0.8);
end

grid on; axis equal;
title('Scenario Operativo 3D: Traiettoria, Boe LBL e Terne Body');
xlabel('Est [m]'); ylabel('Nord [m]'); zlabel('Quota (-Down) [m]');
legend('Ground Truth', 'EKF Stima', 'Boe LBL (Transponders)', 'X Body (Surge)', 'Location', 'best');
view([-35, 25]);