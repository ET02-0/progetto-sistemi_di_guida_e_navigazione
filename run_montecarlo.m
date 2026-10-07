% Esegue la pipeline completa N_RUNS volte con seed diversi, per avere
% statistiche di RMSE (media +/- deviazione standard) invece di un
% singolo numero da un singolo run casuale. NON richiama plot_results.m
% / validation_auv.m (altrimenti si aprirebbero decine di figure): fa
% solo l'estrazione minima necessaria per l'RMSE.
%
% ATTENZIONE: ogni run simula ~310 s di missione a 100 Hz: con N_RUNS=20
% puo' volerci qualche minuto. Abbassa N_RUNS per una prova veloce.


N_RUNS = 3;
seeds = 1:N_RUNS;
traj_sel = 2;
show_diagnostics = false;

rmse_pos_3D = zeros(N_RUNS, 1);
rmse_vel_3D = zeros(N_RUNS, 1);
rmse_roll   = zeros(N_RUNS, 1);
rmse_pitch  = zeros(N_RUNS, 1);
rmse_yaw    = zeros(N_RUNS, 1);
rmse_bg     = zeros(N_RUNS, 3);

for run_idx = 1:N_RUNS
    fprintf('\n=== Run Monte Carlo %d/%d (seed=%d) ===\n', run_idx, N_RUNS, seeds(run_idx));
    rng(seeds(run_idx));

    waypoints_gen;
    trajectory_gen_cartesian;
    sensors_auv;
    out = sim('rov_sim');

    % Estrazione minima (stessa logica di validation_auv.m/calcolo_rmse.m)
    est_data = squeeze(out.x_est);
    if size(est_data, 1) < size(est_data, 2), est_data = est_data'; end
    N = min(length(time), size(est_data, 1));

    pos_true_k = pos_ned(1:N, :);
    vel_true_k = vel_n(1:N, :);
    rpy_true_k = rpy(1:N, :);

    pos_est_k = est_data(1:N, 1:3);
    vel_est_k = est_data(1:N, 4:6);
    eul_est_k = est_data(1:N, 7:9);
    bg_est_k  = est_data(1:N, 10:12);

    err_pos_k = pos_true_k - pos_est_k;
    err_vel_k = vel_true_k - vel_est_k;
    err_eul_k = rad2deg(wrapToPi(rpy_true_k - eul_est_k));

    rmse_pos_3D(run_idx) = sqrt(mean(sum(err_pos_k.^2, 2)));
    rmse_vel_3D(run_idx) = sqrt(mean(sum(err_vel_k.^2, 2)));
    rmse_roll(run_idx)   = sqrt(mean(err_eul_k(:,1).^2));
    rmse_pitch(run_idx)  = sqrt(mean(err_eul_k(:,2).^2));
    rmse_yaw(run_idx)    = sqrt(mean(err_eul_k(:,3).^2));

    tail_idx = round(0.9*N):N;
    bg_regime_k = mean(bg_est_k(tail_idx, :), 1);
    rmse_bg(run_idx, :) = abs(bias_gyro - bg_regime_k);
end

%% Statistiche riassuntive
fprintf('\n============================================================\n');
fprintf('RISULTATI MONTE CARLO (%d run, seed 1..%d)\n', N_RUNS, N_RUNS);
fprintf('============================================================\n');
fprintf('RMSE Posizione 3D [m]:   media=%.4f  std=%.4f  min=%.4f  max=%.4f\n', ...
    mean(rmse_pos_3D), std(rmse_pos_3D), min(rmse_pos_3D), max(rmse_pos_3D));
fprintf('RMSE Velocita'' 3D [m/s]: media=%.4f  std=%.4f  min=%.4f  max=%.4f\n', ...
    mean(rmse_vel_3D), std(rmse_vel_3D), min(rmse_vel_3D), max(rmse_vel_3D));
fprintf('RMSE Roll [deg]:  media=%.4f  std=%.4f\n', mean(rmse_roll), std(rmse_roll));
fprintf('RMSE Pitch [deg]: media=%.4f  std=%.4f\n', mean(rmse_pitch), std(rmse_pitch));
fprintf('RMSE Yaw [deg]:   media=%.4f  std=%.4f\n', mean(rmse_yaw), std(rmse_yaw));
fprintf('Errore bias a regime [rad/s]: X=%.5f+/-%.5f  Y=%.5f+/-%.5f  Z=%.5f+/-%.5f\n', ...
    mean(rmse_bg(:,1)), std(rmse_bg(:,1)), mean(rmse_bg(:,2)), std(rmse_bg(:,2)), ...
    mean(rmse_bg(:,3)), std(rmse_bg(:,3)));

%% Salvataggio e grafico riassuntivo
if ~exist('results', 'dir'), mkdir('results'); end
save(fullfile('results', 'montecarlo_results.mat'), ...
    'rmse_pos_3D', 'rmse_vel_3D', 'rmse_roll', 'rmse_pitch', 'rmse_yaw', 'rmse_bg', 'seeds');

figure('Name', 'Monte Carlo - dispersione RMSE Posizione 3D');
histogram(rmse_pos_3D);
xlabel('RMSE Posizione 3D [m]'); ylabel('Numero di run'); grid on;
title(sprintf('Distribuzione RMSE su %d run Monte Carlo', N_RUNS));