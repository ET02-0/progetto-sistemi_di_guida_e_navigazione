%{
disp('-------------------------------------------');
disp('   CALCOLO RMSE (Root Mean Square Error)   ');
disp('-------------------------------------------');

eul_est = -est_data(1:N, 7:9);
eul_est = est_data(1:N, 7:9);

%% 1. Calcolo Errore Posizione [m]
err_pos = pos_true(1:N, :) - pos_est(1:N, :);

rmse_pos_n = sqrt(mean(err_pos(:,1).^2));
rmse_pos_e = sqrt(mean(err_pos(:,2).^2));
rmse_pos_d = sqrt(mean(err_pos(:,3).^2));

% RMSE 3D (Distanza media assoluta nello spazio)
rmse_pos_3D = sqrt(mean(sum(err_pos.^2, 2)));

%% 2. Calcolo Errore Velocità [m/s]
err_vel = vel_true(1:N, :) - vel_est(1:N, :);

rmse_vel_n = sqrt(mean(err_vel(:,1).^2));
rmse_vel_e = sqrt(mean(err_vel(:,2).^2));
rmse_vel_d = sqrt(mean(err_vel(:,3).^2));

% RMSE 3D Velocità
rmse_vel_3D = sqrt(mean(sum(err_vel.^2, 2)));

%% 3. Calcolo Errore Assetto (Eulero) [deg]
% Usiamo la Ground Truth (rpy) creata nel generatore di traiettorie
err_eul_rad = rpy(1:N, :) - eul_est(1:N, :);

% FONDAMENTALE: Eseguiamo il wrap dell'errore angolare per evitare che 
% un salto da +pi a -pi venga calcolato come un errore gigantesco di 2*pi
err_eul_rad = wrapToPi(err_eul_rad);
err_eul_deg = rad2deg(wrapToPi(rpy(1:N, :) - eul_est(1:N, :)));

rmse_roll  = sqrt(mean(err_eul_deg(:,1).^2));
rmse_pitch = sqrt(mean(err_eul_deg(:,2).^2));
rmse_yaw   = sqrt(mean(err_eul_deg(:,3).^2));

%% 4. Stampa dei Risultati in Command Window
fprintf('\n> RMSE POSIZIONE [m]\n');
fprintf('  Nord:  %.4f m\n', rmse_pos_n);
fprintf('  Est:   %.4f m\n', rmse_pos_e);
fprintf('  Down:  %.4f m\n', rmse_pos_d);
fprintf('  --> 3D Globale: %.4f m\n', rmse_pos_3D);

fprintf('\n> RMSE VELOCITÀ [m/s]\n');
fprintf('  Nord:  %.4f m/s\n', rmse_vel_n);
fprintf('  Est:   %.4f m/s\n', rmse_vel_e);
fprintf('  Down:  %.4f m/s\n', rmse_vel_d);
fprintf('  --> 3D Globale: %.4f m/s\n', rmse_vel_3D);

fprintf('\n> RMSE ASSETTO [deg]\n');
fprintf('  Roll:  %.4f°\n', rmse_roll);
fprintf('  Pitch: %.4f°\n', rmse_pitch);
fprintf('  Yaw:   %.4f°\n', rmse_yaw);
fprintf('-------------------------------------------\n');
%}

disp('-------------------------------------------');
disp('   CALCOLO RMSE (Root Mean Square Error)   ');
disp('-------------------------------------------');

% NOTA: prima qui c'era eul_est = -est_data(...), una toppa per un
% disallineamento di segno che in realta' veniva da come rpy (ground
% truth) era calcolato in trajectory_gen_cartesian.m (vedi li' per il
% dettaglio). Corretta la causa, qui non serve piu' negare nulla: la
% stima EKF (est_data(:,7:9)) e la ground truth (rpy) usano ora la
% stessa convenzione.
eul_est = est_data(1:N, 7:9);

%% 1. Calcolo Errore Posizione [m]
err_pos = pos_true(1:N, :) - pos_est(1:N, :);

rmse_pos_n = sqrt(mean(err_pos(:,1).^2));
rmse_pos_e = sqrt(mean(err_pos(:,2).^2));
rmse_pos_d = sqrt(mean(err_pos(:,3).^2));

% RMSE 3D (Distanza media assoluta nello spazio)
rmse_pos_3D = sqrt(mean(sum(err_pos.^2, 2)));

%% 2. Calcolo Errore Velocità [m/s]
err_vel = vel_true(1:N, :) - vel_est(1:N, :);

rmse_vel_n = sqrt(mean(err_vel(:,1).^2));
rmse_vel_e = sqrt(mean(err_vel(:,2).^2));
rmse_vel_d = sqrt(mean(err_vel(:,3).^2));

% RMSE 3D Velocità
rmse_vel_3D = sqrt(mean(sum(err_vel.^2, 2)));

%% 3. Calcolo Errore Assetto (Eulero) [deg]
% Usiamo la Ground Truth (rpy) creata nel generatore di traiettorie.
% Il wrap dell'errore angolare evita che un salto da +pi a -pi venga
% calcolato come un errore gigantesco di 2*pi.
err_eul_deg = rad2deg(wrapToPi(rpy(1:N, :) - eul_est(1:N, :)));

rmse_roll  = sqrt(mean(err_eul_deg(:,1).^2));
rmse_pitch = sqrt(mean(err_eul_deg(:,2).^2));
rmse_yaw   = sqrt(mean(err_eul_deg(:,3).^2));

%% 4. Calcolo Errore Bias Giroscopi [rad/s]
% bg_est e bias_gyro sono definiti in validation_auv.m e sensors_auv.m
% (eseguiti prima nella pipeline di main.m).
err_bg = repmat(bias_gyro, N, 1) - bg_est(1:N, :);

rmse_bg_x = sqrt(mean(err_bg(:,1).^2));
rmse_bg_y = sqrt(mean(err_bg(:,2).^2));
rmse_bg_z = sqrt(mean(err_bg(:,3).^2));

% Errore "a regime" sull'ultimo 10% della simulazione (stesso criterio
% gia' usato in validation_auv.m), utile perche' il transitorio iniziale
% di convergenza del bias pesa molto sull'RMSE calcolato su tutta la
% missione.
tail_idx = round(0.9 * N):N;
bg_regime = mean(bg_est(tail_idx, :), 1);
err_bg_regime = bias_gyro - bg_regime;
err_bg_regime_perc = abs(err_bg_regime ./ bias_gyro) * 100;

%% 5. Stampa dei Risultati in Command Window
fprintf('\n> RMSE POSIZIONE [m]\n');
fprintf('  Nord:  %.4f m\n', rmse_pos_n);
fprintf('  Est:   %.4f m\n', rmse_pos_e);
fprintf('  Down:  %.4f m\n', rmse_pos_d);
fprintf('  --> 3D Globale: %.4f m\n', rmse_pos_3D);

fprintf('\n> RMSE VELOCITÀ [m/s]\n');
fprintf('  Nord:  %.4f m/s\n', rmse_vel_n);
fprintf('  Est:   %.4f m/s\n', rmse_vel_e);
fprintf('  Down:  %.4f m/s\n', rmse_vel_d);
fprintf('  --> 3D Globale: %.4f m/s\n', rmse_vel_3D);

fprintf('\n> RMSE ASSETTO [deg]\n');
fprintf('  Roll:  %.4f°\n', rmse_roll);
fprintf('  Pitch: %.4f°\n', rmse_pitch);
fprintf('  Yaw:   %.4f°\n', rmse_yaw);

fprintf('\n> RMSE BIAS GIROSCOPI (su tutta la missione) [rad/s]\n');
fprintf('  X: %.6f  Y: %.6f  Z: %.6f\n', rmse_bg_x, rmse_bg_y, rmse_bg_z);
fprintf('> Errore bias a regime (ultimo 10%% missione)\n');
fprintf('  X: %.6f rad/s (%.2f%%)  Y: %.6f rad/s (%.2f%%)  Z: %.6f rad/s (%.2f%%)\n', ...
    err_bg_regime(1), err_bg_regime_perc(1), ...
    err_bg_regime(2), err_bg_regime_perc(2), ...
    err_bg_regime(3), err_bg_regime_perc(3));
fprintf('-------------------------------------------\n');

%% 6. Tabella riassuntiva + export CSV e LaTeX
Grandezza = { ...
    'Posizione Nord'; 'Posizione Est'; 'Posizione Down'; 'Posizione 3D'; ...
    'Velocità Nord'; 'Velocità Est'; 'Velocità Down'; 'Velocità 3D'; ...
    'Assetto Roll'; 'Assetto Pitch'; 'Assetto Yaw'; ...
    'Bias giroscopio X (a regime)'; 'Bias giroscopio Y (a regime)'; 'Bias giroscopio Z (a regime)'};

RMSE = [rmse_pos_n; rmse_pos_e; rmse_pos_d; rmse_pos_3D; ...
        rmse_vel_n; rmse_vel_e; rmse_vel_d; rmse_vel_3D; ...
        rmse_roll; rmse_pitch; rmse_yaw; ...
        abs(err_bg_regime(1)); abs(err_bg_regime(2)); abs(err_bg_regime(3))];

Unita = {'m'; 'm'; 'm'; 'm'; 'm/s'; 'm/s'; 'm/s'; 'm/s'; 'deg'; 'deg'; 'deg'; ...
         'rad/s'; 'rad/s'; 'rad/s'};

rmse_table = table(Grandezza, RMSE, Unita);

if ~exist('results', 'dir')
    mkdir('results');
end

writetable(rmse_table, fullfile('results', 'rmse_table.csv'));

fid = fopen(fullfile('results', 'rmse_table.tex'), 'w', 'n', 'UTF-8');
fprintf(fid, '%% Generato automaticamente da calcolo_rmse.m - non modificare a mano.\n');
fprintf(fid, '%% Includere nel capitolo dei risultati con: \\input{results/rmse_table.tex}\n');
fprintf(fid, '\\begin{table}[h]\n');
fprintf(fid, '    \\centering\n');
fprintf(fid, '    \\begin{tabular}{lrl}\n');
fprintf(fid, '        \\toprule\n');
fprintf(fid, '        \\textbf{Grandezza} & \\textbf{RMSE} & \\textbf{Unità} \\\\\n');
fprintf(fid, '        \\midrule\n');
for r = 1:height(rmse_table)
    fprintf(fid, '        %s & %.4f & %s \\\\\n', ...
        rmse_table.Grandezza{r}, rmse_table.RMSE(r), rmse_table.Unita{r});
end
fprintf(fid, '        \\bottomrule\n');
fprintf(fid, '    \\end{tabular}\n');
fprintf(fid, '    \\caption{Errori quadratici medi (RMSE) della stima EKF rispetto alla Ground Truth.}\n');
fprintf(fid, '    \\label{tab:rmse_risultati}\n');
fprintf(fid, '\\end{table}\n');
fclose(fid);

fprintf('\nTabella RMSE esportata in results/rmse_table.csv e results/rmse_table.tex\n');