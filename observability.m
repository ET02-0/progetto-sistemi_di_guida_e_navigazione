%{
disp('Inizio analisi di Osservabilità (Symbolic Math)...');

%% 1. Definizione Variabili Simboliche
% Stato x = [pN, pE, pD, vN, vE, vD, phi, theta, psi, bgx, bgy, bgz]^T
syms pN pE pD vN vE vD phi theta psi bgx bgy bgz real
x = [pN; pE; pD; vN; vE; vD; phi; theta; psi; bgx; bgy; bgz];

% Ingressi u = [ax, ay, az, wx, wy, wz]^T (Accelerazioni e Omega misurate)
syms ax ay az wx wy wz real
u = [ax; ay; az; wx; wy; wz];

% Posizione Boe (Assumiamo 4 boe per l'analisi)
syms N1 E1 N2 E2 N3 E3 N4 E4 real
buoys = [N1; E1; N2; E2; N3; E3; N4; E4];

g = 9.81; % Gravità

%% 2. Equazioni Dinamiche (Modello Continuo x_dot = f(x,u))
% Matrice di Rotazione da Body a NED (R_b^n)
R_bn = [cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
        cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
       -sin(theta),          sin(phi)*cos(theta),                            cos(phi)*cos(theta)];

% Matrice di Trasformazione T (da ratei body a derivate angoli di Eulero)
T_mat = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
         0, cos(phi),           -sin(phi);
         0, sin(phi)/cos(theta), cos(phi)/cos(theta)];

% Derivate dello stato
p_dot = [vN; vE; vD];
v_dot = R_bn * [ax; ay; az] + [0; 0; g];
eul_dot = T_mat * ([wx; wy; wz] - [bgx; bgy; bgz]);
bg_dot = [0; 0; 0];

f = [p_dot; v_dot; eul_dot; bg_dot];

%% 3. Equazioni di Misura (y = h(x))
% 3 angoli di Eulero, 4 Bearing (Boe), 1 Profondità
h_eul = [phi; theta; psi];
% Nota: usiamo atan per facilitare la derivazione simbolica rispetto ad atan2
h_b1  = atan((E1 - pE) / (N1 - pN));
h_b2  = atan((E2 - pE) / (N2 - pN));
h_b3  = atan((E3 - pE) / (N3 - pN));
h_b4  = atan((E4 - pE) / (N4 - pN));
h_dep = pD;

h = [h_eul; h_b1; h_b2; h_b3; h_b4; h_dep];

%% 4. Calcolo Jacobiane Simboliche (F e H)
disp('Calcolo Jacobiane simboliche F e H...');
F_sym = jacobian(f, x);
H_sym = jacobian(h, x);

%% 5. Valutazione Numerica su un Punto Operativo
% Scegliamo uno stato generico dell'AUV (non banale/non zero per evitare singolarità)
disp('Sostituzione punto operativo numerico...');

x_val = [100; 50; 20; ...      % Posizione (N, E, D)
         1.5; 0.2; 0; ...      % Velocità
         0.1; -0.05; 0.8; ...  % Eulero (Roll, Pitch, Yaw in rad)
         0.02; -0.01; 0.005];  % Bias Giroscopi

u_val = [0.5; 0.1; -9.8; ...   % Acc. body misurata
         0.01; 0.02; 0.05];    % Omega body misurata

% Posizioni delle 4 Boe (come in sensors_auv.m)
buoy_val = [-50; -50; 250; -50; 250; 200; -50; 200]; 

% Sostituzione numerica
F_num = double(subs(F_sym, [x; u; buoys], [x_val; u_val; buoy_val]));
H_num = double(subs(H_sym, [x; buoys], [x_val; buoy_val]));

%% 6. Costruzione Matrice di Osservabilità O_EKF e Calcolo Rango
disp('Costruzione Matrice di Osservabilità O_EKF...');
n_states = length(x);
O_EKF = H_num;

% O_EKF = [H; H*F; H*F^2; ... ; H*F^(n-1)]
for i = 1:(n_states - 1)
    O_EKF = [O_EKF; H_num * (F_num^i)];
end

%% 7. Risultato Finale
rango_O = rank(O_EKF);

fprintf('\n----------------------------------------\n');
fprintf('RISULTATO ANALISI DI OSSERVABILITÀ:\n');
fprintf('Numero di stati del sistema: %d\n', n_states);
fprintf('Rango della matrice O_EKF:   %d\n', rango_O);

if rango_O == n_states
    fprintf('=> IL SISTEMA È COMPLETAMENTE OSSERVABILE.\n');
    fprintf('   L''EKF è in grado di stimare tutti gli stati (incluso il bias) correttamente.\n');
else
    fprintf('=> ATTENZIONE: Il sistema NON è completamente osservabile!\n');
end
fprintf('----------------------------------------\n');
%}

%% observability.m
% Analisi di osservabilita' locale del sistema EKF a 12 stati.
%
% Rispetto alla versione precedente:
%  - le posizioni delle boe non sono piu' ridichiarate a mano (fonte di
%    possibile disallineamento con sensors_auv.m): si usa direttamente
%    P_bases_ned dal workspace;
%  - l'osservabilita' viene testata con 1, 2, 3 e 4 boe (non solo 4), per
%    dimostrare davvero quante ne servono, invece di limitarsi a
%    verificare che "con 4 funziona";
%  - se main.m ha gia' girato la pipeline completa, i punti operativi non
%    sono piu' un singolo valore inventato a mano: si campionano piu'
%    istanti reali lungo la traiettoria simulata (posizione, velocita',
%    assetto, bias stimato, ingressi IMU misurati), per verificare che il
%    rango non dipenda da una scelta fortunata del punto di
%    linearizzazione;
%  - oltre al rango, si riportano il valore singolare minimo e il numero
%    di condizionamento di O_EKF, che danno un'idea del margine (un rango
%    pieno ma con sigma_min ~ 0 e' "osservabile sulla carta" ma debole in
%    pratica).

disp('Inizio analisi di Osservabilita'' (Symbolic Math)...');

%% 1. Definizione Variabili Simboliche
% Stato x = [pN, pE, pD, vN, vE, vD, phi, theta, psi, bgx, bgy, bgz]^T
syms pN pE pD vN vE vD phi theta psi bgx bgy bgz real
x = [pN; pE; pD; vN; vE; vD; phi; theta; psi; bgx; bgy; bgz];
n_states = length(x);

% Ingressi u = [ax, ay, az, wx, wy, wz]^T (Accelerazioni e Omega misurate)
syms ax ay az wx wy wz real
u = [ax; ay; az; wx; wy; wz];

% Posizioni delle boe come vettori simbolici indicizzati, cosi' da poter
% selezionare comodamente un sottoinsieme (1..4) piu' avanti.
Nb = sym('Nb', [1 4], 'real');
Eb = sym('Eb', [1 4], 'real');
buoys = reshape([Nb; Eb], [], 1); % [N1;E1;N2;E2;N3;E3;N4;E4]

g = 9.81; % Gravita'

%% 2. Equazioni Dinamiche (Modello Continuo x_dot = f(x,u))
R_bn = [cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi);
        cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi), cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi);
       -sin(theta),          sin(phi)*cos(theta),                            cos(phi)*cos(theta)];

T_mat = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
         0, cos(phi),           -sin(phi);
         0, sin(phi)/cos(theta), cos(phi)/cos(theta)];

p_dot = [vN; vE; vD];
v_dot = R_bn * [ax; ay; az] + [0; 0; g];
eul_dot = T_mat * ([wx; wy; wz] - [bgx; bgy; bgz]);
bg_dot = [0; 0; 0];

f = [p_dot; v_dot; eul_dot; bg_dot];

%% 3. Equazioni di Misura (y = h(x)), boa per boa
h_eul = [phi; theta; psi];

h_b = sym('h_b', [4 1]);
for j = 1:4
    h_b(j) = atan((Eb(j) - pE) / (Nb(j) - pN));
end

h_dep = pD;

h_full = [h_eul; h_b; h_dep]; % 8x1, tutte e 4 le boe (caso di riferimento)

%% 4. Jacobiane simboliche (calcolate una sola volta)
disp('Calcolo Jacobiane simboliche F e H (tutte le 4 boe)...');
F_sym = jacobian(f, x);
H_full_sym = jacobian(h_full, x); % 8x12; righe 1-3 Eulero, 4-7 boe 1-4, 8 profondita'

%% 5. Punti operativi: dalla traiettoria reale se disponibile
% Se main.m ha gia' eseguito l'intera pipeline (waypoints_gen,
% trajectory_gen_cartesian, sensors_auv, sim, validation_auv,
% calcolo_rmse), in workspace ci sono i dati veri della simulazione: li
% usiamo per campionare piu' punti operativi reali invece di un singolo
% valore inventato a mano. Se observability.m viene eseguito da solo
% (senza il resto della pipeline), si ricade su un unico punto di
% riferimento come nella versione precedente.
has_real_traj = exist('pos_ned', 'var') && exist('vel_n', 'var') && ...
    exist('rpy', 'var') && exist('acc_meas', 'var') && exist('gyro_meas', 'var');

if has_real_traj
    if exist('bg_est', 'var')
        bias_for_op_point = bg_est; % stima corrente del filtro (piu' realistico)
    elseif exist('bias_gyro', 'var')
        bias_for_op_point = repmat(bias_gyro, size(pos_ned, 1), 1); % fallback: bias vero costante
    else
        bias_for_op_point = zeros(size(pos_ned, 1), 3);
    end

    N_traj = min([size(pos_ned, 1), size(vel_n, 1), size(rpy, 1), ...
                  size(acc_meas, 1), size(gyro_meas, 1), size(bias_for_op_point, 1)]);

    n_test_points = 5;
    test_idx = round(linspace(1, N_traj, n_test_points));

    op_points = struct('x_val', {}, 'u_val', {}, 'label', {});
    for k = 1:n_test_points
        idx = test_idx(k);
        x_val = [pos_ned(idx, :).'; vel_n(idx, :).'; rpy(idx, :).'; bias_for_op_point(idx, :).'];
        u_val = [acc_meas(idx, :).'; gyro_meas(idx, :).'];
        op_points(k).x_val = x_val;
        op_points(k).u_val = u_val;
        op_points(k).label = sprintf('t = %.1f s (campione %d/%d)', time(idx), k, n_test_points);
    end
    disp('Punti operativi presi dalla traiettoria simulata reale.');
else
    x_val = [100; 50; 20; ...
             1.5; 0.2; 0; ...
             0.1; -0.05; 0.8; ...
             0.02; -0.01; 0.005];
    u_val = [0.5; 0.1; -9.8; ...
             0.01; 0.02; 0.05];
    op_points = struct('x_val', {x_val}, 'u_val', {u_val}, ...
                        'label', {'punto operativo fisso (traiettoria non trovata in workspace)'});
    disp('Traiettoria reale non trovata in workspace: uso un unico punto operativo di riferimento.');
end

%% 6. Posizioni delle boe: dal workspace, non ridichiarate a mano
if exist('P_bases_ned', 'var')
    buoy_val = reshape(P_bases_ned(:, 1:2).', [], 1); % [N1;E1;N2;E2;N3;E3;N4;E4]
else
    % Fallback, identico a sensors_auv.m, solo se eseguito standalone
    buoy_val = [-50; -50; 250; -50; 250; 200; -50; 200];
    warning('P_bases_ned non trovato in workspace: uso le posizioni di fallback di sensors_auv.m.');
end

%% 7. Sweep: per ogni punto operativo, per N_boe = 1..4
% Righe di H_full corrispondenti a [Eulero(3), boa_j(1), profondita'(1)]
row_eul = 1:3;
row_dep = 8;

n_points = numel(op_points);
results = table('Size', [n_points * 4, 6], ...
    'VariableTypes', {'double', 'double', 'double', 'double', 'double', 'string'}, ...
    'VariableNames', {'PuntoOp', 'N_boe', 'Rango', 'SigmaMin', 'CondNumber', 'Label'});

row_out = 1;
for k = 1:n_points
    x_val_k = op_points(k).x_val;
    u_val_k = op_points(k).u_val;

    F_num = double(subs(F_sym, [x; u], [x_val_k; u_val_k]));
    H_full_num = double(subs(H_full_sym, [x; buoys], [x_val_k; buoy_val]));

    for n_boe = 1:4
        row_boe = 3 + (1:n_boe);
        H_test = H_full_num([row_eul, row_boe, row_dep], :);

        O_test = H_test;
        for i = 1:(n_states - 1)
            O_test = [O_test; H_test * (F_num^i)]; %#ok<AGROW>
        end

        rango = rank(O_test);
        sv = svd(O_test);
        sigma_min = sv(end);
        cond_num = sv(1) / sv(end);

        results(row_out, :) = {k, n_boe, rango, sigma_min, cond_num, string(op_points(k).label)};
        row_out = row_out + 1;
    end
end

disp(' ');
disp('----------------------------------------------------------------');
disp('RISULTATO SWEEP OSSERVABILITA'' (rango, sigma_min, condizionamento)');
disp('----------------------------------------------------------------');
disp(results);

%% 8. Riepilogo: numero minimo di boe che da' rango pieno, per ogni punto
disp(' ');
for k = 1:n_points
    sub = results(results.PuntoOp == k, :);
    idx_full = find(sub.Rango == n_states, 1, 'first');
    if isempty(idx_full)
        fprintf('%s: rango pieno (%d) MAI raggiunto nemmeno con 4 boe!\n', ...
            op_points(k).label, n_states);
    else
        fprintf('%s: rango pieno (%d) raggiunto con %d boa/e (sigma_min = %.3e)\n', ...
            op_points(k).label, n_states, sub.N_boe(idx_full), sub.SigmaMin(idx_full));
    end
end

%% 9. Grafico: sigma_min in funzione del numero di boe, un punto operativo per curva
figure('Name', 'Osservabilita'': sigma_min vs numero di boe');
hold on; grid on;
for k = 1:n_points
    sub = results(results.PuntoOp == k, :);
    semilogy(sub.N_boe, sub.SigmaMin, '-o', 'LineWidth', 1.5, 'DisplayName', op_points(k).label);
end
xlabel('Numero di boe usate'); ylabel('\sigma_{min}(O_{EKF}) (scala log)');
title('Margine di osservabilita'' al variare del numero di boe');
legend('Location', 'best', 'Interpreter', 'none');
xticks(1:4);