% Stima empirica (Monte Carlo) di R_ALG, invece dei valori scelti a mano
% in sensors_auv.m (std_alg_rollpitch = 0.005, std_alg_yaw = 0.015).
%
% Simula tante coppie (acc_meas, mag_meas) rumorose attorno a un assetto
% nominale e guarda la covarianza campionaria dell'euler_meas prodotto da
% Attitude_ALG.m: da' una stima giustificata di R_ALG al posto di un
% numero scelto a occhio (vedi spiegazione completa in chat sul perche'
% questo si puo' fare per R_ALG ma non allo stesso modo per Q).
%
% Richiede solo Attitude_ALG.m sul path (non serve il resto della
% pipeline / Simulink).

%% Parametri di rumore (devono coincidere con sensors_auv.m)
std_dev_acc = 0.01;  % [m/s^2]
std_dev_mag = 0.03;  % [adimensionale, sul campo normalizzato]
g = 9.81;
mag_n = [1; 0; 0];

%% Punti operativi: alcuni assetti nominali rappresentativi
% (in volo rettilineo roll/pitch sono piccoli; durante le virate il
% rollio arriva a ~0.08 rad = 4.6 deg, vedi waypoints_gen.m)
test_attitudes_deg = [0, 0, 0; ...      % assetto nominale, prua a Nord
    0, 0, 90; ...     % prua a Est
    4.6, 0, 45; ...   % rollio da virata, prua a 45 deg
    0, 5, 0];         % piccolo beccheggio

N_MC = 5000; % numero di campioni Monte Carlo per punto operativo

fprintf('Stima Monte Carlo di R_ALG (%d campioni per punto operativo)\n\n', N_MC);

R_ALG_estimates = cell(size(test_attitudes_deg, 1), 1);

for p = 1:size(test_attitudes_deg, 1)
    phi0   = deg2rad(test_attitudes_deg(p, 1));
    theta0 = deg2rad(test_attitudes_deg(p, 2));
    psi0   = deg2rad(test_attitudes_deg(p, 3));

    % C_b_n identica a quella usata in EKF_Predict.m / Meccanizzazione_Pura.m
    C_b_n = [cos(theta0)*cos(psi0), sin(phi0)*sin(theta0)*cos(psi0)-cos(phi0)*sin(psi0), cos(phi0)*sin(theta0)*cos(psi0)+sin(phi0)*sin(psi0);
        cos(theta0)*sin(psi0), sin(phi0)*sin(theta0)*sin(psi0)+cos(phi0)*cos(psi0), cos(phi0)*sin(theta0)*sin(psi0)-sin(phi0)*cos(psi0);
        -sin(theta0),           sin(phi0)*cos(theta0),                               cos(phi0)*cos(theta0)];
    C_n_b = C_b_n.';

    % Specifica forza ideale in assenza di manovre (bassa dinamica,
    % l'ipotesi su cui si basa Attitude_ALG.m): f_b = C_n_b*[0;0;-g]
    acc_true_b = C_n_b * [0; 0; -g];
    mag_true_b = C_n_b * mag_n;

    euler_samples = zeros(N_MC, 3);
    for k = 1:N_MC
        acc_noisy = acc_true_b + std_dev_acc * randn(3,1);
        mag_noisy = mag_true_b + std_dev_mag * randn(3,1);
        euler_samples(k, :) = Attitude_ALG(acc_noisy, mag_noisy)';
    end

    R_ALG_mc = cov(euler_samples);
    R_ALG_estimates{p} = R_ALG_mc;

    fprintf('Assetto [roll=%.1f pitch=%.1f yaw=%.1f] deg:\n', test_attitudes_deg(p,:));
    fprintf('  std MC      [roll pitch yaw] = [%.5f %.5f %.5f] rad\n', sqrt(diag(R_ALG_mc)));
    fprintf('  std attuale [roll pitch yaw] = [0.00500 0.00500 0.01500] rad (sensors_auv.m)\n\n');
end

%% Riepilogo: caso peggiore sui punti testati
all_stds = cell2mat(cellfun(@(R) sqrt(diag(R))', R_ALG_estimates, 'UniformOutput', false));
worst_case_std = max(all_stds, [], 1);

fprintf('------------------------------------------------------------\n');
fprintf('Deviazione standard peggiore sui punti operativi testati:\n');
fprintf('  [roll pitch yaw] = [%.5f %.5f %.5f] rad\n', worst_case_std);
fprintf('Se e'' piu'' piccola dei valori attuali, quelli in sensors_auv.m\n');
fprintf('sono gia'' conservativi. Se e'' piu'' grande, li stanno sottostimando.\n');
fprintf('------------------------------------------------------------\n');