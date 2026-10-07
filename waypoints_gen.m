ST = 1/100; % 100 Hz
dt = ST;

if ~exist('traj_sel', 'var')
    traj_sel = 1; % Default: Ispezione condotta
end

switch traj_sel
    case 1
        %% TRAIETTORIA 1: Ispezione Condotta / Pipeline
        t_max = 200;
        time = (0:ST:t_max)';
        pos_ned_n = zeros(length(time),1);
        pos_ned_e = zeros(length(time),1);
        pos_ned_d = zeros(length(time),1);
        yaw = zeros(length(time),1);
        pitch = zeros(length(time),1);
        roll = zeros(length(time),1);
        
        t1_end = 10; t2_end = 40; t3_end = 100; t4_end = 140;
        v_cruise = 2.0; desc_rate = 0.5;
        v_horiz = sqrt(v_cruise^2 - desc_rate^2);
        turn_radius = 50; angle_rate = v_cruise / turn_radius;
        
        n = 0; e = 0; d = 0;
        for i = 1:length(time)
            t = time(i);
            if t <= t1_end
                pos_ned_n(i) = n; pos_ned_e(i) = e; pos_ned_d(i) = d;
            elseif t <= t2_end
                dt_p = t - t1_end;
                pos_ned_n(i) = n + v_horiz * dt_p;
                pos_ned_d(i) = d + desc_rate * dt_p;
                pitch(i) = atan2(desc_rate, v_horiz);
            elseif t <= t3_end
                dt_p = t - t2_end;
                pos_ned_n(i) = pos_ned_n(round(t2_end/ST)+1) + v_cruise * dt_p;
                pos_ned_d(i) = pos_ned_d(round(t2_end/ST)+1);
            elseif t <= t4_end
                dt_p = t - t3_end;
                th = angle_rate * dt_p;
                pos_ned_n(i) = pos_ned_n(round(t3_end/ST)+1) + turn_radius * sin(th);
                pos_ned_e(i) = e + turn_radius * (1 - cos(th));
                pos_ned_d(i) = pos_ned_d(round(t3_end/ST)+1);
                yaw(i) = th; roll(i) = 0.05;
            else
                dt_p = t - t4_end;
                hd = angle_rate * (t4_end - t3_end);
                pos_ned_n(i) = pos_ned_n(round(t4_end/ST)+1) + v_cruise * dt_p * cos(hd);
                pos_ned_e(i) = pos_ned_e(round(t4_end/ST)+1) + v_cruise * dt_p * sin(hd);
                pos_ned_d(i) = pos_ned_d(round(t4_end/ST)+1);
                yaw(i) = hd;
            end
        end

    case 2
        %% TRAIETTORIA 2: Lawnmower Survey Orizzontale (Piano XY)
        v = 1.5; depth_const = 15;
        L_leg = 80; R_turn = 15;
        t_leg = L_leg / v;
        t_turn = (pi * R_turn) / v;
        w_turn = pi / t_turn;

        % Tabella segmenti: [tipo (1=leg, 2=turn_dx, 3=turn_sx), durata]
        segments = [1, t_leg; 2, t_turn; 1, t_leg; 3, t_turn; 1, t_leg; 2, t_turn; 1, t_leg];

        % t_max calcolato dalla durata reale del percorso (+ margine di
        % assestamento), non piu' un valore fisso. Con t_max=240 fisso la
        % simulazione si fermava a meta' della terza virata (il percorso
        % completo dura sum(segments(:,2)) = 4*t_leg + 3*t_turn ~= 307.6 s
        % con questi parametri) e la quarta passata non veniva mai
        % eseguita. Se cambi v, L_leg o R_turn, t_max si aggiorna da solo.
        t_max = sum(segments(:,2)) + 2;
        time = (0:ST:t_max)';
        pos_ned_n = zeros(length(time),1);
        pos_ned_e = zeros(length(time),1);
        pos_ned_d = zeros(length(time),1);
        yaw = zeros(length(time),1);
        pitch = zeros(length(time),1);
        roll = zeros(length(time),1);

        % Profilo a 4 passate parallele
        curr_n = 0; curr_e = 0; curr_yaw = 0;
        pos_ned_d(:) = depth_const;
        t_accum = 0;
        
        idx = 1;
        for s = 1:size(segments,1)
            type = segments(s,1);
            dur = segments(s,2);
            n_steps = round(dur/ST);
            for k = 1:n_steps
                if idx > length(time), break; end
                if type == 1
                    curr_n = curr_n + v * ST * cos(curr_yaw);
                    curr_e = curr_e + v * ST * sin(curr_yaw);
                    roll(idx) = 0;
                elseif type == 2
                    curr_yaw = curr_yaw + w_turn * ST;
                    curr_n = curr_n + v * ST * cos(curr_yaw);
                    curr_e = curr_e + v * ST * sin(curr_yaw);
                    roll(idx) = 0.08;
                elseif type == 3
                    curr_yaw = curr_yaw - w_turn * ST;
                    curr_n = curr_n + v * ST * cos(curr_yaw);
                    curr_e = curr_e + v * ST * sin(curr_yaw);
                    roll(idx) = -0.08;
                end
                pos_ned_n(idx) = curr_n;
                pos_ned_e(idx) = curr_e;
                yaw(idx) = curr_yaw;
                idx = idx + 1;
            end
        end
        while idx <= length(time)
            pos_ned_n(idx) = curr_n; pos_ned_e(idx) = curr_e;
            yaw(idx) = curr_yaw; idx = idx + 1;
        end

    case 3
        %% TRAIETTORIA 3: Profilazione Verticale / Yo-Yo (Piano XZ)
        t_max = 200;
        time = (0:ST:t_max)';
        v_forward = 1.5;
        d_mean = 20; d_amp = 12;
        omega_z = 2 * pi / 50; % Ciclo yo-yo ogni 50 secondi
        
        pos_ned_n = v_forward * time;
        pos_ned_e = zeros(length(time),1);
        pos_ned_d = d_mean + d_amp * sin(omega_z * time);
        
        v_d_prof = d_amp * omega_z * cos(omega_z * time);
        pitch = atan2(v_d_prof, v_forward);
        yaw = zeros(length(time),1);
        roll = zeros(length(time),1);

    case 4
        %% TRAIETTORIA 4: Spirale Elicoidale 3D (Helix Dive)
        t_max = 200;
        time = (0:ST:t_max)';
        R_helix = 45;
        w_helix = 2 * pi / 60; % Giro completo ogni 60 s
        v_z = 0.15; % Discesa verticale [m/s]
        
        pos_ned_n = R_helix * sin(w_helix * time);
        pos_ned_e = R_helix * (1 - cos(w_helix * time));
        pos_ned_d = 5 + v_z * time;
        
        v_tan = R_helix * w_helix;
        pitch = repmat(atan2(v_z, v_tan), length(time), 1);
        yaw = w_helix * time;
        roll = repmat(0.06, length(time), 1);

    case 5
        %% TRAIETTORIA 5: Lemniscata 3D (Figura a 8 con ondulazione)
        t_max = 220;
        time = (0:ST:t_max)';
        A = 60; % Ampiezza anelli [m]
        w_8 = 2 * pi / 110; % 2 lobi in 110 s
        
        % Lemniscata di Gerono sul piano orizzontale
        pos_ned_n = A * sin(w_8 * time);
        pos_ned_e = A * sin(w_8 * time) .* cos(w_8 * time);
        pos_ned_d = 18 + 7 * sin(2 * w_8 * time);
        
        % Velocità cinematiche per derivazione assetto
        v_n = [diff(pos_ned_n)/ST; 0];
        v_e = [diff(pos_ned_e)/ST; 0];
        v_d = [diff(pos_ned_d)/ST; 0];
        
        yaw = atan2(v_e, v_n);
        pitch = atan2(v_d, sqrt(v_n.^2 + v_e.^2));
        roll = 0.1 * sin(w_8 * time);
end

% Assegnazione matrici
pos_ned = [pos_ned_n, pos_ned_e, pos_ned_d];
euler_angles_deg = rad2deg([yaw, pitch, roll]);

total_trajectory = [pos_ned, euler_angles_deg];

% Waypoints campionati uniformemente per waypointTrajectory
num_waypoints = 60;
ind = round(linspace(1, length(time), num_waypoints));
waypoints_ned = [time(ind), total_trajectory(ind, :)];

% Plot 3D diagnostico
figure('Name', sprintf('Traiettoria Selezionata: Tipo %d', traj_sel));
plot3(pos_ned(:,2), pos_ned(:,1), -pos_ned(:,3), 'b', 'LineWidth', 2);
hold on; grid on; axis equal;
plot3(waypoints_ned(:,3), waypoints_ned(:,2), -waypoints_ned(:,4), 'ro', 'MarkerFaceColor', 'r');
title(sprintf('Traiettoria AUV 3D (Caso %d)', traj_sel));
xlabel('East [m]'); ylabel('North [m]'); zlabel('Quota (-Down) [m]');
legend('Traiettoria Continua', 'Waypoints', 'Location', 'best');