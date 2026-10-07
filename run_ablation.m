%% run_ablation.m
% Confronta alcune varianti del sistema di navigazione per mostrare,
% quantitativamente, perche' ogni scelta di progetto serve davvero (non
% solo "con 4 boe il rango e' pieno" come in observability.m, ma "con 4
% boe l'RMSE e' X, con 2 e' Y").
%
%  A) EKF completo vs Meccanizzazione Pura (Dead Reckoning): gia'
%     disponibile da un singolo run (out.x_mech), nessuna simulazione
%     aggiuntiva necessaria.
%  B) Numero di boe BO (1, 2, 3, 4): NON ridimensiona P_bases_ned o
%     bearings_meas (i blocchi MATLAB Function di Simulink richiedono
%     dimensioni fisse decise in compilazione: cambiarle tra un sim() e
%     l'altro rompe l'anello di retroazione predict/update). Si
%     "disattiva" una boa gonfiando la sua varianza in R_BO a un valore
%     enorme, stessa tecnica del punto C) per l'ALG: il guadagno di
%     Kalman per quel canale diventa trascurabile, dimensioni invariate.
%  C) Con vs senza correzione ALG: disattivata gonfiando R_ALG.
%
% NON include un confronto "con/senza stima del bias giroscopico" come
% variante separata, perche' richiederebbe ristrutturare lo stato (da 12
% a 9 componenti) - un cambiamento strutturale da decidere con calma, non
% automatizzato qui. Il punto A) mostra comunque l'effetto pratico di non
% avere alcuna correzione, bias incluso (Meccanizzazione_Pura.m usa
% gyro_meas grezzo, senza sottrarre alcun bias).
%
% ATTENZIONE - DA VERIFICARE PRIMA DI FIDARSI DEI RISULTATI:
% questo script presume che il blocco EKF_Update in Simulink legga la
% matrice di misura combinata da una variabile di workspace chiamata "R"
% (il nome del parametro nella firma di EKF_Update.m), costruita qui con
% blkdiag(R_ALG, R_BO, R_depth). Se il blocco Constant/From Workspace
% collegato all'ingresso R del tuo modello e' configurato con un nome
% diverso, i punti B) e C) non avranno alcun effetto reale sulla
% simulazione pur non dando errore.

clear; clc; close all;

traj_sel = 2;
show_diagnostics = false;
rng(42); % stesso seed per ogni variante: le differenze nei risultati
         % vengono dalla variante testata, non dal rumore casuale

%% --- Run di base: traiettoria e sensori generati una sola volta ---
waypoints_gen;
trajectory_gen_cartesian;
sensors_auv;

results_ablation = struct();

%% --- A) EKF vs Meccanizzazione Pura ---
R = blkdiag(R_ALG, R_BO, R_depth); %#ok<NASGU>
out = sim('rov_sim');

est_data = squeeze(out.x_est);
mech_data = squeeze(out.x_mech);
if size(est_data, 1) < size(est_data, 2), est_data = est_data'; end
if size(mech_data, 1) < size(mech_data, 2), mech_data = mech_data'; end
N = min([length(time), size(est_data,1), size(mech_data,1)]);

pos_true_0 = pos_ned(1:N,:);
err_pos_ekf  = pos_true_0 - est_data(1:N,1:3);
err_pos_mech = pos_true_0 - mech_data(1:N,1:3);

results_ablation.A_rmse_pos_ekf  = sqrt(mean(sum(err_pos_ekf.^2,2)));
results_ablation.A_rmse_pos_mech = sqrt(mean(sum(err_pos_mech.^2,2)));

fprintf('\n=== A) EKF vs Meccanizzazione Pura ===\n');
fprintf('RMSE posizione 3D - EKF:  %.4f m\n', results_ablation.A_rmse_pos_ekf);
fprintf('RMSE posizione 3D - MECH: %.4f m (atteso >> EKF, divergenza Dead Reckoning)\n', ...
    results_ablation.A_rmse_pos_mech);

%% --- B) Numero di boe BO: 1, 2, 3, 4 (via inflazione di R_BO) ---
% Si sovrascrive direttamente la variabile R_BO (non solo una R
% combinata nuova): se il blocco Simulink ricostruisce R internamente
% leggendo R_ALG/R_BO/R_depth dal workspace invece di usare una singola
% variabile R, questo garantisce che la modifica abbia effetto in ogni
% caso. R_BO_base conserva l'originale per il ripristino a fine ciclo.
fprintf('\n=== B) Sweep numero di boe (via inflazione di R_BO) ===\n');
rmse_pos_vs_nboe = zeros(4,1);
rmse_bg_vs_nboe  = zeros(4,3);
R_BO_base = R_BO;

for n_boe = 1:4
    R_BO = diag(ones(N_bases,1) * 1e6); %#ok<NASGU> % tutte "disattivate" di default
    R_BO(1:n_boe, 1:n_boe) = R_BO_base(1:n_boe, 1:n_boe); % le prime n_boe attive
    R = blkdiag(R_ALG, R_BO, R_depth); %#ok<NASGU>

    out = sim('rov_sim');

    est_data_b = squeeze(out.x_est);
    if size(est_data_b, 1) < size(est_data_b, 2), est_data_b = est_data_b'; end
    Nb = min(length(time), size(est_data_b,1));

    pos_true_k = pos_ned(1:Nb,:);
    err_pos_k = pos_true_k - est_data_b(1:Nb,1:3);
    rmse_pos_vs_nboe(n_boe) = sqrt(mean(sum(err_pos_k.^2,2)));

    bg_est_k = est_data_b(1:Nb, 10:12);
    tail_idx = round(0.9*Nb):Nb;
    rmse_bg_vs_nboe(n_boe,:) = abs(bias_gyro - mean(bg_est_k(tail_idx,:),1));

    fprintf('  %d boa/e attive: RMSE posizione 3D = %.4f m, errore bias a regime = [%.5f %.5f %.5f] rad/s\n', ...
        n_boe, rmse_pos_vs_nboe(n_boe), rmse_bg_vs_nboe(n_boe,1), rmse_bg_vs_nboe(n_boe,2), rmse_bg_vs_nboe(n_boe,3));
end
R_BO = R_BO_base; % ripristino, prima del punto C)

results_ablation.B_rmse_pos_vs_nboe = rmse_pos_vs_nboe;
results_ablation.B_rmse_bg_vs_nboe  = rmse_bg_vs_nboe;

figure('Name', 'Ablation - RMSE posizione vs numero di boe');
plot(1:4, rmse_pos_vs_nboe, '-o', 'LineWidth', 1.5);
xlabel('Numero di boe attive'); ylabel('RMSE posizione 3D [m]'); grid on;
title('Effetto pratico del numero di boe sull''RMSE di posizione');
xticks(1:4);

%% --- C) Con vs senza correzione ALG ---
% Stesso principio del punto B): si sovrascrive direttamente R_ALG, non
% solo una R combinata nuova.
fprintf('\n=== C) Con vs senza correzione ALG ===\n');

rmse_eul_with_alg = sqrt(mean(rad2deg(wrapToPi(rpy(1:N,:) - est_data(1:N,7:9))).^2, 1));

R_ALG_base = R_ALG;
R_ALG = diag([1e6, 1e6, 1e6]);  % incertezza enorme = correzione ALG disattivata
R = blkdiag(R_ALG, R_BO, R_depth); 
out = sim('rov_sim');

est_data_noalg = squeeze(out.x_est);
if size(est_data_noalg, 1) < size(est_data_noalg, 2), est_data_noalg = est_data_noalg'; end
N2 = min(length(time), size(est_data_noalg,1));
rmse_eul_without_alg = sqrt(mean(rad2deg(wrapToPi(rpy(1:N2,:) - est_data_noalg(1:N2,7:9))).^2, 1));
R_ALG = R_ALG_base; % ripristino

results_ablation.C_rmse_eul_with_alg    = rmse_eul_with_alg;
results_ablation.C_rmse_eul_without_alg = rmse_eul_without_alg;

fprintf('RMSE assetto [Roll Pitch Yaw] CON ALG:    [%.3f %.3f %.3f] deg\n', rmse_eul_with_alg);
fprintf('RMSE assetto [Roll Pitch Yaw] SENZA ALG:  [%.3f %.3f %.3f] deg\n', rmse_eul_without_alg);

%% --- Salvataggio ---
if ~exist('results', 'dir'), mkdir('results'); end
save(fullfile('results', 'ablation_results.mat'), 'results_ablation');
fprintf('\nRisultati salvati in results/ablation_results.mat\n');