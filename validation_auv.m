%{
%% 0. Estrazione Dati e Allineamento Temporale
disp('Estrazione dati da Simulink...');
% bias_gyro e' gia' definito in sensors_auv.m (eseguito prima in main.m) e
% condiviso nel workspace di base. NON ridichiararlo qui: una versione
% precedente lo sovrascriveva con [0.02, -0.01, 0.005], diverso dal valore
% realmente iniettato nei sensori ([0.015, -0.01, 0.005]). Questo falsava
% sia il grafico di convergenza del bias (Fig. 5.2) sia la stampa
% dell'errore percentuale a regime, confrontando la stima con un
% riferimento sbagliato sull'asse X.
assert(exist('bias_gyro', 'var') == 1, ...
    ['bias_gyro non trovato nel workspace: eseguire sensors_auv.m ', ...
     'prima di validation_auv.m (vedi main.m).']);

% Estrazione Stato (squeeze rimuove le dimensioni vuote 3D -> 2D)
est_data = squeeze(out.x_est);
mech_data = squeeze(out.x_mech);
if size(est_data, 1) < size(est_data, 2), est_data = est_data'; end
if size(mech_data, 1) < size(mech_data, 2), mech_data = mech_data'; end

P_data = out.P_est;
N = min([length(time), size(est_data, 1), size(mech_data, 1), size(P_data, 3)]);
t_plot = time(1:N);

% Ground Truth
pos_true = pos_ned(1:N, :);
vel_true = vel_n(1:N, :);
acc_true = acc_n(1:N, :);
omega_true = omega_b(1:N, :);
rpy_true = rpy(1:N, :); % ora coerente in segno con eul_est (vedi trajectory_gen_cartesian.m)

% Stime EKF
pos_est = est_data(1:N, 1:3);
vel_est = est_data(1:N, 4:6);
eul_est = est_data(1:N, 7:9);
bg_est  = est_data(1:N, 10:12);

%% Calcolo Variabili per Plot

% 1. Consistenza Integrazioni
pos_int_v = cumtrapz(t_plot, vel_true) + pos_true(1,:);
vel_int_a = cumtrapz(t_plot, acc_true) + vel_true(1,:);
pos_int_a = cumtrapz(t_plot, vel_int_a) + pos_true(1,:);

% 2. Dinamiche Omega
omega_meas = gyro_meas(1:N, :);
omega_filt = omega_meas - bg_est;

% 3. Errori e bound +/- 3 sigma per TUTTI i blocchi di stato (prima
% c'era solo per la posizione: qui si estende a velocita', assetto e
% bias, leggendo le rispettive sotto-diagonali di P_data).
sigma_pos = zeros(N, 3);
sigma_vel = zeros(N, 3);
sigma_eul = zeros(N, 3);
sigma_bg  = zeros(N, 3);
for k = 1:N
    P_k = P_data(:, :, k);
    sigma_pos(k, :) = sqrt(diag(P_k(1:3, 1:3)))';
    sigma_vel(k, :) = sqrt(diag(P_k(4:6, 4:6)))';
    sigma_eul(k, :) = sqrt(diag(P_k(7:9, 7:9)))';
    sigma_bg(k, :)  = sqrt(diag(P_k(10:12, 10:12)))';
end

err_pos = pos_true - pos_est;
err_vel = vel_true - vel_est;
err_eul = wrapToPi(rpy_true - eul_est); % il wrap evita salti artificiali a +/-180 deg
err_bg  = repmat(bias_gyro, N, 1) - bg_est;

%% Figura 1: Consistenza Cinematica (Nord/Est/Down)
figure('Name', 'Consistenza Cinematica', 'Position', [50, 50, 1300, 380]);
tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
plot(t_plot, pos_true(:,1), 'k', t_plot, pos_int_v(:,1), 'b--', t_plot, pos_int_a(:,1), 'r:');
title('Consistenza - Nord'); ylabel('[m]'); xlabel('Tempo [s]'); grid on;
legend('True', 'Int(Vel)', 'Int(Acc)', 'Location', 'best');

nexttile;
plot(t_plot, pos_true(:,2), 'k', t_plot, pos_int_v(:,2), 'b--', t_plot, pos_int_a(:,2), 'r:');
title('Consistenza - Est'); xlabel('Tempo [s]'); grid on;

nexttile;
plot(t_plot, pos_true(:,3), 'k', t_plot, pos_int_v(:,3), 'b--', t_plot, pos_int_a(:,3), 'r:');
title('Consistenza - Down'); xlabel('Tempo [s]'); grid on;

%% Figura 2: Convergenza Bias Giroscopi (valori assoluti)
figure('Name', 'Convergenza Bias Giroscopi', 'Position', [50, 50, 1300, 380]);
tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
titles_bg = {'Bias X (Roll)', 'Bias Y (Pitch)', 'Bias Z (Yaw)'};
colors_bg = {'r', 'g', 'b'};
for i = 1:3
    nexttile;
    plot(t_plot, ones(size(t_plot))*bias_gyro(i), 'k--', 'LineWidth', 1.5); hold on;
    plot(t_plot, bg_est(:,i), colors_bg{i}, 'LineWidth', 1.5);
    title(titles_bg{i}); ylabel('[rad/s]'); xlabel('Tempo [s]'); grid on;
    if i==1, legend('True Bias', 'EKF Bias', 'Location', 'best'); end
end

%% Figura 3: Dinamiche Velocita' Angolare (Omega)
figure('Name', 'Dinamiche Omega', 'Position', [50, 50, 1300, 380]);
tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
titles_om = {'\omega_x (Roll Rate)', '\omega_y (Pitch Rate)', '\omega_z (Yaw Rate)'};
for i = 1:3
    nexttile;
    plot(t_plot, omega_true(:,i), 'k', t_plot, omega_meas(:,i), 'r:', t_plot, omega_filt(:,i), 'b', 'LineWidth', 1.2);
    title(titles_om{i}); ylabel('[rad/s]'); xlabel('Tempo [s]'); grid on;
    if i==1, legend('True', 'Noisy', 'Compensato dal bias stimato', 'Location', 'best'); end
    % NOTA: "Compensato dal bias stimato", non "Filtered": omega_filt e'
    % semplicemente omega_meas - bg_est, sottrazione del bias, non un
    % filtraggio del rumore (vedi EKF_Predict.m: il rumore bianco del
    % giroscopio resta intatto, solo il bias costante viene rimosso).
end

%% Figura 4-7: Errore e Bound +/- 3 sigma, un blocco di stato per figura
plot_err_bound_block('Posizione', {'Nord', 'Est', 'Down'}, '[m]', ...
    t_plot, err_pos, sigma_pos);

plot_err_bound_block('Velocità', {'Nord', 'Est', 'Down'}, '[m/s]', ...
    t_plot, err_vel, sigma_vel);

plot_err_bound_block('Assetto', {'Roll', 'Pitch', 'Yaw'}, '[deg]', ...
    t_plot, rad2deg(err_eul), rad2deg(sigma_eul));

plot_err_bound_block('Bias Giroscopio', {'X', 'Y', 'Z'}, '[rad/s]', ...
    t_plot, err_bg, sigma_bg);

%% Figura 8: NIS (Normalized Innovation Squared) - consistenza globale
% Controlla se il filtro e' correttamente tarato: se lo e', NIS_k deve
% cadere nell'intervallo di confidenza chi-quadro con "df" gradi di
% liberta' (df = dimensione del vettore di misura) circa il 95% delle
% volte. Un NIS sistematicamente troppo alto indica un filtro troppo
% "sicuro di se'" (P sottostimata rispetto all'errore reale); troppo
% basso, un filtro troppo conservativo (P sovrastimata). E' un test
% quantitativo, mentre "i residui restano dentro +/-3 sigma" (Fig. 5.1
% originale) e' solo un controllo visivo canale per canale.
%
% ATTENZIONE: se il blocco EKF_Update viene invocato piu' spesso di
% quanto le misure piu' lente (bearing a 5 Hz, ALG a 10 Hz, vedi la nota
% aperta in sensors_auv.m) vengano davvero aggiornate, l'innovazione
% mostrera' un pattern periodico "a gradini": e' il modo piu' diretto
% per verificare se le frequenze dichiarate nella relazione sono
% rispettate dal modello Simulink.
% NOTA: prima qui c'era "if isfield(out,'innov') && isfield(out,'S_cov')".
% isfield() non e' affidabile su un oggetto Simulink.SimulationOutput
% (non e' una struct semplice): dava falso anche quando out.innov
% funzionava perfettamente in accesso diretto (come gia' faceva
% plot_results.m senza alcun controllo). Uso try/catch invece.
try
    innov_data = squeeze(out.innov);
    S_cov_data = out.S_cov;
    if size(innov_data, 1) < size(innov_data, 2), innov_data = innov_data'; end
    N_nis = min(N, size(innov_data, 1));

    df = size(innov_data, 2);
    nis = zeros(N_nis, 1);
    for k = 1:N_nis
        ik = innov_data(k, :)';
        Sk = S_cov_data(:, :, k);
        nis(k) = ik' * (Sk \ ik);
    end

    if exist('chi2inv', 'file') == 2
        nis_lo = chi2inv(0.025, df);
        nis_hi = chi2inv(0.975, df);
    elseif df == 8
        nis_lo = 2.180; nis_hi = 17.535; % valori tabulati chi-quadro, df=8, 95%
        warning(['Statistics and Machine Learning Toolbox non trovato: ', ...
                 'uso bound NIS tabulati per df=8.']);
    else
        nis_lo = NaN; nis_hi = NaN;
        warning(['Statistics and Machine Learning Toolbox non trovato e df~=8: ', ...
                 'impossibile calcolare i bound NIS.']);
    end

    pct_in_bound = 100 * mean(nis >= nis_lo & nis <= nis_hi);
    fprintf('\nNIS: %.1f%% dei campioni entro il bound chi-quadro 95%% (df=%d, atteso ~95%%)\n', ...
        pct_in_bound, df);

    figure('Name', 'Consistenza del filtro: NIS', 'Position', [50, 50, 1000, 400]);
    plot(t_plot(1:N_nis), nis, 'b'); hold on; grid on;
    yline(nis_lo, 'r--', 'LineWidth', 1.2);
    yline(nis_hi, 'r--', 'LineWidth', 1.2);
    xlabel('Tempo [s]'); ylabel('NIS');
    title(sprintf('Normalized Innovation Squared (df=%d, bound chi-quadro 95%%)', df));
    legend('NIS', 'Bound 95%', 'Location', 'best');
catch ME
    warning(['Impossibile calcolare il NIS da out.innov/out.S_cov: %s. ', ...
             'Verificare che il modello Simulink esponga questi segnali in uscita.'], ME.message);
end

%% Stampa errore percentuale a regime in console
tail_idx = round(0.9 * N):N;
bg_regime = mean(bg_est(tail_idx, :));
err_perc = abs((bg_regime - bias_gyro) ./ bias_gyro) * 100;
fprintf('\nErrore Percentuale Bias a Regime: X=%.2f%%, Y=%.2f%%, Z=%.2f%%\n', err_perc(1), err_perc(2), err_perc(3));


%% Funzione locale: una figura 1x3 errore + bound +/- 3 sigma
function plot_err_bound_block(block_name, axis_labels, unit_str, t_plot, err, sigma)
    figure('Name', sprintf('Errore e Bound 3sigma - %s', block_name), ...
        'Position', [50, 50, 1300, 380]);
    tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    for i = 1:3
        nexttile;
        plot(t_plot, err(:,i), 'b', 'LineWidth', 1.3); hold on;
        plot(t_plot,  3*sigma(:,i), 'r--', 'LineWidth', 1.1);
        plot(t_plot, -3*sigma(:,i), 'r--', 'LineWidth', 1.1);
        title(sprintf('%s - %s (3\\sigma)', block_name, axis_labels{i}));
        xlabel('Tempo [s]'); ylabel(unit_str); grid on;
        if i == 1, legend('Errore', '+/- 3\sigma', 'Location', 'best'); end
    end
end
%}
%{
%% 0. Estrazione Dati e Allineamento Temporale
disp('Estrazione dati da Simulink...');
% bias_gyro e' gia' definito in sensors_auv.m (eseguito prima in main.m) e
% condiviso nel workspace di base. NON ridichiararlo qui: una versione
% precedente lo sovrascriveva con [0.02, -0.01, 0.005], diverso dal valore
% realmente iniettato nei sensori ([0.015, -0.01, 0.005]). Questo falsava
% sia il grafico di convergenza del bias (Fig. 5.2) sia la stampa
% dell'errore percentuale a regime, confrontando la stima con un
% riferimento sbagliato sull'asse X.
assert(exist('bias_gyro', 'var') == 1, ...
    ['bias_gyro non trovato nel workspace: eseguire sensors_auv.m ', ...
     'prima di validation_auv.m (vedi main.m).']);

% Estrazione Stato (squeeze rimuove le dimensioni vuote 3D -> 2D)
est_data = squeeze(out.x_est);
mech_data = squeeze(out.x_mech);
if size(est_data, 1) < size(est_data, 2), est_data = est_data'; end
if size(mech_data, 1) < size(mech_data, 2), mech_data = mech_data'; end

P_data = out.P_est;
N = min([length(time), size(est_data, 1), size(mech_data, 1), size(P_data, 3)]);
t_plot = time(1:N);

% Ground Truth
pos_true = pos_ned(1:N, :);
vel_true = vel_n(1:N, :);
acc_true = acc_n(1:N, :);
omega_true = omega_b(1:N, :);
rpy_true = rpy(1:N, :); % ora coerente in segno con eul_est (vedi trajectory_gen_cartesian.m)

% Stime EKF
pos_est = est_data(1:N, 1:3);
vel_est = est_data(1:N, 4:6);
eul_est = est_data(1:N, 7:9);
bg_est  = est_data(1:N, 10:12);

%% Calcolo Variabili per Plot

% 1. Consistenza Integrazioni
pos_int_v = cumtrapz(t_plot, vel_true) + pos_true(1,:);
vel_int_a = cumtrapz(t_plot, acc_true) + vel_true(1,:);
pos_int_a = cumtrapz(t_plot, vel_int_a) + pos_true(1,:);

% 2. Dinamiche Omega
omega_meas = gyro_meas(1:N, :);
omega_filt = omega_meas - bg_est;

% 3. Errori e bound +/- 3 sigma per TUTTI i blocchi di stato (prima
% c'era solo per la posizione: qui si estende a velocita', assetto e
% bias, leggendo le rispettive sotto-diagonali di P_data).
sigma_pos = zeros(N, 3);
sigma_vel = zeros(N, 3);
sigma_eul = zeros(N, 3);
sigma_bg  = zeros(N, 3);
for k = 1:N
    P_k = P_data(:, :, k);
    sigma_pos(k, :) = sqrt(diag(P_k(1:3, 1:3)))';
    sigma_vel(k, :) = sqrt(diag(P_k(4:6, 4:6)))';
    sigma_eul(k, :) = sqrt(diag(P_k(7:9, 7:9)))';
    sigma_bg(k, :)  = sqrt(diag(P_k(10:12, 10:12)))';
end

err_pos = pos_true - pos_est;
err_vel = vel_true - vel_est;
err_eul = wrapToPi(rpy_true - eul_est); % il wrap evita salti artificiali a +/-180 deg
err_bg  = repmat(bias_gyro, N, 1) - bg_est;

%% Figura 1: Consistenza Cinematica (Nord/Est/Down)
figure('Name', 'Consistenza Cinematica', 'Position', [50, 50, 1300, 380]);
tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
plot(t_plot, pos_true(:,1), 'k', t_plot, pos_int_v(:,1), 'b--', t_plot, pos_int_a(:,1), 'r:');
title('Consistenza - Nord'); ylabel('[m]'); xlabel('Tempo [s]'); grid on;
legend('True', 'Int(Vel)', 'Int(Acc)', 'Location', 'best');

nexttile;
plot(t_plot, pos_true(:,2), 'k', t_plot, pos_int_v(:,2), 'b--', t_plot, pos_int_a(:,2), 'r:');
title('Consistenza - Est'); xlabel('Tempo [s]'); grid on;

nexttile;
plot(t_plot, pos_true(:,3), 'k', t_plot, pos_int_v(:,3), 'b--', t_plot, pos_int_a(:,3), 'r:');
title('Consistenza - Down'); xlabel('Tempo [s]'); grid on;

%% Figura 2: Convergenza Bias Giroscopi (valori assoluti)
figure('Name', 'Convergenza Bias Giroscopi', 'Position', [50, 50, 1300, 380]);
tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
titles_bg = {'Bias X (Roll)', 'Bias Y (Pitch)', 'Bias Z (Yaw)'};
colors_bg = {'r', 'g', 'b'};
for i = 1:3
    nexttile;
    plot(t_plot, ones(size(t_plot))*bias_gyro(i), 'k--', 'LineWidth', 1.5); hold on;
    plot(t_plot, bg_est(:,i), colors_bg{i}, 'LineWidth', 1.5);
    title(titles_bg{i}); ylabel('[rad/s]'); xlabel('Tempo [s]'); grid on;
    if i==1, legend('True Bias', 'EKF Bias', 'Location', 'best'); end
end

%% Figura 3: Dinamiche Velocita' Angolare (Omega)
figure('Name', 'Dinamiche Omega', 'Position', [50, 50, 1300, 380]);
tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
titles_om = {'\omega_x (Roll Rate)', '\omega_y (Pitch Rate)', '\omega_z (Yaw Rate)'};
for i = 1:3
    nexttile;
    plot(t_plot, omega_true(:,i), 'k', t_plot, omega_meas(:,i), 'r:', t_plot, omega_filt(:,i), 'b', 'LineWidth', 1.2);
    title(titles_om{i}); ylabel('[rad/s]'); xlabel('Tempo [s]'); grid on;
    if i==1, legend('True', 'Noisy', 'Compensato dal bias stimato', 'Location', 'best'); end
    % NOTA: "Compensato dal bias stimato", non "Filtered": omega_filt e'
    % semplicemente omega_meas - bg_est, sottrazione del bias, non un
    % filtraggio del rumore (vedi EKF_Predict.m: il rumore bianco del
    % giroscopio resta intatto, solo il bias costante viene rimosso).
end

%% Figura 4-7: Errore e Bound +/- 3 sigma, un blocco di stato per figura
plot_err_bound_block('Posizione', {'Nord', 'Est', 'Down'}, '[m]', ...
    t_plot, err_pos, sigma_pos);

plot_err_bound_block('Velocità', {'Nord', 'Est', 'Down'}, '[m/s]', ...
    t_plot, err_vel, sigma_vel);

plot_err_bound_block('Assetto', {'Roll', 'Pitch', 'Yaw'}, '[deg]', ...
    t_plot, rad2deg(err_eul), rad2deg(sigma_eul));

plot_err_bound_block('Bias Giroscopio', {'X', 'Y', 'Z'}, '[rad/s]', ...
    t_plot, err_bg, sigma_bg);

%% Figura 8: NIS (Normalized Innovation Squared) - consistenza globale
% Controlla se il filtro e' correttamente tarato: se lo e', NIS_k deve
% cadere nell'intervallo di confidenza chi-quadro con "df" gradi di
% liberta' (df = dimensione del vettore di misura) circa il 95% delle
% volte. Un NIS sistematicamente troppo alto indica un filtro troppo
% "sicuro di se'" (P sottostimata rispetto all'errore reale); troppo
% basso, un filtro troppo conservativo (P sovrastimata). E' un test
% quantitativo, mentre "i residui restano dentro +/-3 sigma" (Fig. 5.1
% originale) e' solo un controllo visivo canale per canale.
%
% ATTENZIONE: se il blocco EKF_Update viene invocato piu' spesso di
% quanto le misure piu' lente (bearing a 5 Hz, ALG a 10 Hz, vedi la nota
% aperta in sensors_auv.m) vengano davvero aggiornate, l'innovazione
% mostrera' un pattern periodico "a gradini": e' il modo piu' diretto
% per verificare se le frequenze dichiarate nella relazione sono
% rispettate dal modello Simulink.
if isfield(out, 'innov') && isfield(out, 'S_cov')
    innov_data = squeeze(out.innov);
    S_cov_data = out.S_cov;
    if size(innov_data, 1) < size(innov_data, 2), innov_data = innov_data'; end
    N_nis = min(N, size(innov_data, 1));

    df = size(innov_data, 2);
    nis = zeros(N_nis, 1);
    for k = 1:N_nis
        ik = innov_data(k, :)';
        Sk = S_cov_data(:, :, k);
        nis(k) = ik' * (Sk \ ik);
    end

    if exist('chi2inv', 'file') == 2
        nis_lo = chi2inv(0.025, df);
        nis_hi = chi2inv(0.975, df);
    elseif df == 8
        nis_lo = 2.180; nis_hi = 17.535; % valori tabulati chi-quadro, df=8, 95%
        warning(['Statistics and Machine Learning Toolbox non trovato: ', ...
                 'uso bound NIS tabulati per df=8.']);
    else
        nis_lo = NaN; nis_hi = NaN;
        warning(['Statistics and Machine Learning Toolbox non trovato e df~=8: ', ...
                 'impossibile calcolare i bound NIS.']);
    end

    pct_in_bound = 100 * mean(nis >= nis_lo & nis <= nis_hi);
    fprintf('\nNIS: %.1f%% dei campioni entro il bound chi-quadro 95%% (df=%d, atteso ~95%%)\n', ...
        pct_in_bound, df);

    figure('Name', 'Consistenza del filtro: NIS', 'Position', [50, 50, 1000, 400]);
    plot(t_plot(1:N_nis), nis, 'b'); hold on; grid on;
    yline(nis_lo, 'r--', 'LineWidth', 1.2);
    yline(nis_hi, 'r--', 'LineWidth', 1.2);
    xlabel('Tempo [s]'); ylabel('NIS');
    title(sprintf('Normalized Innovation Squared (df=%d, bound chi-quadro 95%%)', df));
    legend('NIS', 'Bound 95%', 'Location', 'best');
else
    warning(['out.innov e/o out.S_cov non trovati: impossibile calcolare il NIS. ', ...
             'Verificare che il modello Simulink esponga questi segnali in uscita.']);
end

%% Stampa errore percentuale a regime in console
tail_idx = round(0.9 * N):N;
bg_regime = mean(bg_est(tail_idx, :));
err_perc = abs((bg_regime - bias_gyro) ./ bias_gyro) * 100;
fprintf('\nErrore Percentuale Bias a Regime: X=%.2f%%, Y=%.2f%%, Z=%.2f%%\n', err_perc(1), err_perc(2), err_perc(3));


%% Funzione locale: una figura 1x3 errore + bound +/- 3 sigma
function plot_err_bound_block(block_name, axis_labels, unit_str, t_plot, err, sigma)
    figure('Name', sprintf('Errore e Bound 3sigma - %s', block_name), ...
        'Position', [50, 50, 1300, 380]);
    tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    for i = 1:3
        nexttile;
        plot(t_plot, err(:,i), 'b', 'LineWidth', 1.3); hold on;
        plot(t_plot,  3*sigma(:,i), 'r--', 'LineWidth', 1.1);
        plot(t_plot, -3*sigma(:,i), 'r--', 'LineWidth', 1.1);
        title(sprintf('%s - %s (3\\sigma)', block_name, axis_labels{i}));
        xlabel('Tempo [s]'); ylabel(unit_str); grid on;
        if i == 1, legend('Errore', '+/- 3\sigma', 'Location', 'best'); end
    end
end
%}
%% 0. Estrazione Dati e Allineamento Temporale
disp('Estrazione dati da Simulink...');
% bias_gyro e' gia' definito in sensors_auv.m (eseguito prima in main.m) e
% condiviso nel workspace di base. NON ridichiararlo qui: una versione
% precedente lo sovrascriveva con [0.02, -0.01, 0.005], diverso dal valore
% realmente iniettato nei sensori ([0.015, -0.01, 0.005]). Questo falsava
% sia il grafico di convergenza del bias (Fig. 5.2) sia la stampa
% dell'errore percentuale a regime, confrontando la stima con un
% riferimento sbagliato sull'asse X.
assert(exist('bias_gyro', 'var') == 1, ...
    ['bias_gyro non trovato nel workspace: eseguire sensors_auv.m ', ...
     'prima di validation_auv.m (vedi main.m).']);

% Estrazione Stato (squeeze rimuove le dimensioni vuote 3D -> 2D)
est_data = squeeze(out.x_est);
mech_data = squeeze(out.x_mech);
if size(est_data, 1) < size(est_data, 2), est_data = est_data'; end
if size(mech_data, 1) < size(mech_data, 2), mech_data = mech_data'; end

P_data = out.P_est;
N = min([length(time), size(est_data, 1), size(mech_data, 1), size(P_data, 3)]);
t_plot = time(1:N);

% Ground Truth
pos_true = pos_ned(1:N, :);
vel_true = vel_n(1:N, :);
acc_true = acc_n(1:N, :);
omega_true = omega_b(1:N, :);
rpy_true = rpy(1:N, :); % ora coerente in segno con eul_est (vedi trajectory_gen_cartesian.m)

% Stime EKF
pos_est = est_data(1:N, 1:3);
vel_est = est_data(1:N, 4:6);
eul_est = est_data(1:N, 7:9);
bg_est  = est_data(1:N, 10:12);

%% Calcolo Variabili per Plot

% 1. Consistenza Integrazioni
pos_int_v = cumtrapz(t_plot, vel_true) + pos_true(1,:);
vel_int_a = cumtrapz(t_plot, acc_true) + vel_true(1,:);
pos_int_a = cumtrapz(t_plot, vel_int_a) + pos_true(1,:);

% 2. Dinamiche Omega
omega_meas = gyro_meas(1:N, :);
omega_filt = omega_meas - bg_est;

% 3. Errori e bound +/- 3 sigma per TUTTI i blocchi di stato (prima
% c'era solo per la posizione: qui si estende a velocita', assetto e
% bias, leggendo le rispettive sotto-diagonali di P_data).
sigma_pos = zeros(N, 3);
sigma_vel = zeros(N, 3);
sigma_eul = zeros(N, 3);
sigma_bg  = zeros(N, 3);
for k = 1:N
    P_k = P_data(:, :, k);
    sigma_pos(k, :) = sqrt(diag(P_k(1:3, 1:3)))';
    sigma_vel(k, :) = sqrt(diag(P_k(4:6, 4:6)))';
    sigma_eul(k, :) = sqrt(diag(P_k(7:9, 7:9)))';
    sigma_bg(k, :)  = sqrt(diag(P_k(10:12, 10:12)))';
end

err_pos = pos_true - pos_est;
err_vel = vel_true - vel_est;
err_eul = wrapToPi(rpy_true - eul_est); % il wrap evita salti artificiali a +/-180 deg
err_bg  = repmat(bias_gyro, N, 1) - bg_est;

%% Figura 1: Consistenza Cinematica (Nord/Est/Down)
figure('Name', 'Consistenza Cinematica', 'Position', [50, 50, 1300, 380]);
tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
plot(t_plot, pos_true(:,1), 'k', t_plot, pos_int_v(:,1), 'b--', t_plot, pos_int_a(:,1), 'r:');
title('Consistenza - Nord'); ylabel('[m]'); xlabel('Tempo [s]'); grid on;
legend('True', 'Int(Vel)', 'Int(Acc)', 'Location', 'best');

nexttile;
plot(t_plot, pos_true(:,2), 'k', t_plot, pos_int_v(:,2), 'b--', t_plot, pos_int_a(:,2), 'r:');
title('Consistenza - Est'); xlabel('Tempo [s]'); grid on;

nexttile;
plot(t_plot, pos_true(:,3), 'k', t_plot, pos_int_v(:,3), 'b--', t_plot, pos_int_a(:,3), 'r:');
title('Consistenza - Down'); xlabel('Tempo [s]'); grid on;

%% Figura 2: Convergenza Bias Giroscopi (valori assoluti)
figure('Name', 'Convergenza Bias Giroscopi', 'Position', [50, 50, 1300, 380]);
tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
titles_bg = {'Bias X (Roll)', 'Bias Y (Pitch)', 'Bias Z (Yaw)'};
colors_bg = {'r', 'g', 'b'};
for i = 1:3
    nexttile;
    plot(t_plot, ones(size(t_plot))*bias_gyro(i), 'k--', 'LineWidth', 1.5); hold on;
    plot(t_plot, bg_est(:,i), colors_bg{i}, 'LineWidth', 1.5);
    title(titles_bg{i}); ylabel('[rad/s]'); xlabel('Tempo [s]'); grid on;
    if i==1, legend('True Bias', 'EKF Bias', 'Location', 'best'); end
end

%% Figura 3: Dinamiche Velocita' Angolare (Omega)
figure('Name', 'Dinamiche Omega', 'Position', [50, 50, 1300, 380]);
tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
titles_om = {'\omega_x (Roll Rate)', '\omega_y (Pitch Rate)', '\omega_z (Yaw Rate)'};
for i = 1:3
    nexttile;
    plot(t_plot, omega_true(:,i), 'k', t_plot, omega_meas(:,i), 'r:', t_plot, omega_filt(:,i), 'b', 'LineWidth', 1.2);
    title(titles_om{i}); ylabel('[rad/s]'); xlabel('Tempo [s]'); grid on;
    if i==1, legend('True', 'Noisy', 'Compensato dal bias stimato', 'Location', 'best'); end
    % NOTA: "Compensato dal bias stimato", non "Filtered": omega_filt e'
    % semplicemente omega_meas - bg_est, sottrazione del bias, non un
    % filtraggio del rumore (vedi EKF_Predict.m: il rumore bianco del
    % giroscopio resta intatto, solo il bias costante viene rimosso).
end

%% Percentuale di campioni entro il bound +/- 3 sigma, canale per canale
% Controllo quantitativo (non solo visivo) di quanto l'errore reale si
% mantenga dentro l'intervallo di confidenza dichiarato da P. Per un
% filtro ben tarato ci si aspetta una percentuale molto alta (il bound e'
% a 3 deviazioni standard, quindi teoricamente ~99.7% per un errore
% gaussiano, ma la soglia pratica di riferimento qui e' semplicemente
% "vicino al 100%", senza uscite sistematiche prolungate).
fprintf('\n--- Percentuale di campioni entro il bound +/- 3 sigma ---\n');
pct_within_block('Posizione', {'Nord', 'Est', 'Down'}, err_pos, sigma_pos);
pct_within_block('Velocità', {'Nord', 'Est', 'Down'}, err_vel, sigma_vel);
pct_within_block('Assetto', {'Roll', 'Pitch', 'Yaw'}, err_eul, sigma_eul); % rad, coerente con sigma_eul
pct_within_block('Bias Giroscopio', {'X', 'Y', 'Z'}, err_bg, sigma_bg);

%% Figura 4-7: Errore e Bound +/- 3 sigma, un blocco di stato per figura
plot_err_bound_block('Posizione', {'Nord', 'Est', 'Down'}, '[m]', ...
    t_plot, err_pos, sigma_pos);

plot_err_bound_block('Velocità', {'Nord', 'Est', 'Down'}, '[m/s]', ...
    t_plot, err_vel, sigma_vel);

plot_err_bound_block('Assetto', {'Roll', 'Pitch', 'Yaw'}, '[deg]', ...
    t_plot, rad2deg(err_eul), rad2deg(sigma_eul));

plot_err_bound_block('Bias Giroscopio', {'X', 'Y', 'Z'}, '[rad/s]', ...
    t_plot, err_bg, sigma_bg);

%% Figura 8: NIS (Normalized Innovation Squared) - consistenza globale
% Controlla se il filtro e' correttamente tarato: se lo e', NIS_k deve
% cadere nell'intervallo di confidenza chi-quadro con "df" gradi di
% liberta' (df = dimensione del vettore di misura) circa il 95% delle
% volte. Un NIS sistematicamente troppo alto indica un filtro troppo
% "sicuro di se'" (P sottostimata rispetto all'errore reale); troppo
% basso, un filtro troppo conservativo (P sovrastimata). E' un test
% quantitativo, mentre "i residui restano dentro +/-3 sigma" (Fig. 5.1
% originale) e' solo un controllo visivo canale per canale.
%
% ATTENZIONE: se il blocco EKF_Update viene invocato piu' spesso di
% quanto le misure piu' lente (bearing a 5 Hz, ALG a 10 Hz, vedi la nota
% aperta in sensors_auv.m) vengano davvero aggiornate, l'innovazione
% mostrera' un pattern periodico "a gradini": e' il modo piu' diretto
% per verificare se le frequenze dichiarate nella relazione sono
% rispettate dal modello Simulink.
% NOTA: prima qui c'era "if isfield(out,'innov') && isfield(out,'S_cov')".
% isfield() non e' affidabile su un oggetto Simulink.SimulationOutput
% (non e' una struct semplice): dava falso anche quando out.innov
% funzionava perfettamente in accesso diretto (come gia' faceva
% plot_results.m senza alcun controllo). Uso try/catch invece.
try
    innov_data = squeeze(out.innov);
    S_cov_data = out.S_cov;
    if size(innov_data, 1) < size(innov_data, 2), innov_data = innov_data'; end
    N_nis = min(N, size(innov_data, 1));

    df = size(innov_data, 2);
    nis = zeros(N_nis, 1);
    for k = 1:N_nis
        ik = innov_data(k, :)';
        Sk = S_cov_data(:, :, k);
        nis(k) = ik' * (Sk \ ik);
    end

    if exist('chi2inv', 'file') == 2
        nis_lo = chi2inv(0.025, df);
        nis_hi = chi2inv(0.975, df);
    elseif df == 8
        nis_lo = 2.180; nis_hi = 17.535; % valori tabulati chi-quadro, df=8, 95%
        warning(['Statistics and Machine Learning Toolbox non trovato: ', ...
                 'uso bound NIS tabulati per df=8.']);
    else
        nis_lo = NaN; nis_hi = NaN;
        warning(['Statistics and Machine Learning Toolbox non trovato e df~=8: ', ...
                 'impossibile calcolare i bound NIS.']);
    end

    pct_in_bound = 100 * mean(nis >= nis_lo & nis <= nis_hi);
    fprintf('\nNIS: %.1f%% dei campioni entro il bound chi-quadro 95%% (df=%d, atteso ~95%%)\n', ...
        pct_in_bound, df);

    % Stesso conteggio escludendo il primo secondo (100 campioni a 100 Hz):
    % se la percentuale sale molto vicino al 95%, lo scostamento visto
    % sopra e' dovuto al transitorio iniziale (P0 ancora largo, prima
    % delle prime correzioni), non a una taratura sbagliata a regime.
    warmup_samples = min(100, N_nis - 1);
    pct_in_bound_no_warmup = 100 * mean(nis(warmup_samples+1:end) >= nis_lo & ...
                                         nis(warmup_samples+1:end) <= nis_hi);
    fprintf('NIS (esclusi i primi %d campioni, ~1 s): %.1f%% entro il bound\n', ...
        warmup_samples, pct_in_bound_no_warmup);

    figure('Name', 'Consistenza del filtro: NIS', 'Position', [50, 50, 1000, 400]);
    plot(t_plot(1:N_nis), nis, 'b'); hold on; grid on;
    yline(nis_lo, 'r--', 'LineWidth', 1.2);
    yline(nis_hi, 'r--', 'LineWidth', 1.2);
    xlabel('Tempo [s]'); ylabel('NIS');
    title(sprintf('Normalized Innovation Squared (df=%d, bound chi-quadro 95%%)', df));
    legend('NIS', 'Bound 95%', 'Location', 'best');
catch ME
    warning(['Impossibile calcolare il NIS da out.innov/out.S_cov: %s. ', ...
             'Verificare che il modello Simulink esponga questi segnali in uscita.'], ME.message);
end

%% Stampa errore percentuale a regime in console
tail_idx = round(0.9 * N):N;
bg_regime = mean(bg_est(tail_idx, :));
err_perc = abs((bg_regime - bias_gyro) ./ bias_gyro) * 100;
fprintf('\nErrore Percentuale Bias a Regime: X=%.2f%%, Y=%.2f%%, Z=%.2f%%\n', err_perc(1), err_perc(2), err_perc(3));


%% Funzione locale: percentuale entro il bound, canale per canale e per blocco
function pct_within_block(block_name, axis_labels, err, sigma)
    pct_axes = zeros(1,3);
    for i = 1:3
        pct_axes(i) = 100 * mean(abs(err(:,i)) <= 3*sigma(:,i));
    end
    pct_all = 100 * mean(all(abs(err) <= 3*sigma, 2));
    fprintf('%s: %s=%.2f%%  %s=%.2f%%  %s=%.2f%%  (tutti e 3 insieme: %.2f%%)\n', ...
        block_name, axis_labels{1}, pct_axes(1), axis_labels{2}, pct_axes(2), ...
        axis_labels{3}, pct_axes(3), pct_all);
end

%% Funzione locale: una figura 1x3 errore + bound +/- 3 sigma
function plot_err_bound_block(block_name, axis_labels, unit_str, t_plot, err, sigma)
    figure('Name', sprintf('Errore e Bound 3sigma - %s', block_name), ...
        'Position', [50, 50, 1300, 380]);
    tiledlayout(1, 3, 'TileSpacing', 'compact', 'Padding', 'compact');
    for i = 1:3
        nexttile;
        plot(t_plot, err(:,i), 'b', 'LineWidth', 1.3); hold on;
        plot(t_plot,  3*sigma(:,i), 'r--', 'LineWidth', 1.1);
        plot(t_plot, -3*sigma(:,i), 'r--', 'LineWidth', 1.1);
        title(sprintf('%s - %s (3\\sigma)', block_name, axis_labels{i}));
        xlabel('Tempo [s]'); ylabel(unit_str); grid on;
        if i == 1, legend('Errore', '+/- 3\sigma', 'Location', 'best'); end
    end
end