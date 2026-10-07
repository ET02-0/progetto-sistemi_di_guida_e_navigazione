%% ANIMATION_AUV_3D.m - Playback Dinamico Traiettoria AUV e Boe LBL
% Richiede nel Workspace: time, pos_ned, est_data, P_bases_ned

N_anim = min([length(time), size(pos_ned, 1), size(est_data, 1)]);
t_a = time(1:N_anim);

% Posizioni e assetto stimato EKF
p_true = pos_ned(1:N_anim, :);
p_ekf  = est_data(1:N_anim, 1:3);
eul_e  = est_data(1:N_anim, 7:9); % [Roll, Pitch, Yaw]

% Setup Finestra
fig = figure('Name', 'Animazione 3D AUV - Missione SGN', ...
             'Position', [100, 50, 1200, 800], 'Color', [0.1 0.12 0.15]);
ax = axes('Parent', fig, 'Color', [0.15 0.18 0.22]);
hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');
ax.GridColor = [0.4 0.45 0.5];
ax.XColor = [0.8 0.8 0.8]; ax.YColor = [0.8 0.8 0.8]; ax.ZColor = [0.8 0.8 0.8];

% Limiti assi con margine
all_pos = [p_true; P_bases_ned];
margin = 25;
xlim(ax, [min(all_pos(:,2))-margin, max(all_pos(:,2))+margin]);
ylim(ax, [min(all_pos(:,1))-margin, max(all_pos(:,1))+margin]);
zlim(ax, [-max(all_pos(:,3))-margin, -min(all_pos(:,3))+margin]);

xlabel(ax, 'Est [m]', 'Color', 'w', 'FontWeight', 'bold');
ylabel(ax, 'Nord [m]', 'Color', 'w', 'FontWeight', 'bold');
zlabel(ax, 'Quota (-Down) [m]', 'Color', 'w', 'FontWeight', 'bold');
title(ax, 'Playback 3D AUV: Stima EKF e Rilevamenti Acustici LBL', 'Color', 'w');
view(ax, [-35, 28]);

% Plot Boe LBL
plot3(ax, P_bases_ned(:,2), P_bases_ned(:,1), -P_bases_ned(:,3), '^', ...
      'MarkerSize', 12, 'MarkerFaceColor', [1 0.2 0.2], 'MarkerEdgeColor', 'w');
for b = 1:size(P_bases_ned, 1)
    text(ax, P_bases_ned(b,2)+2, P_bases_ned(b,1)+2, -P_bases_ned(b,3)+4, ...
         sprintf('Boa %d', b), 'Color', [1 0.4 0.4], 'FontWeight', 'bold');
end

% Linee di traiettoria
h_path_true = plot3(ax, nan, nan, nan, 'Color', [0.4 0.4 0.4], 'LineWidth', 1.2, 'LineStyle', ':');
h_path_ekf  = plot3(ax, nan, nan, nan, 'Color', [0.2 0.8 1],   'LineWidth', 2);

% Fasci acustici LBL (Bearing lines)
h_beams = gobjects(4, 1);
for b = 1:4
    h_beams(b) = plot3(ax, [nan nan], [nan nan], [nan nan], ...
                       'Color', [1 0.9 0.2 0.5], 'LineWidth', 1.3, 'LineStyle', '--');
end

% Definizione Geometria 3D Veicolo (Siluro)
L_auv = 4.5; R_auv = 0.55;
[x_cyl, y_cyl, z_cyl] = cylinder(R_auv, 16);
z_cyl = (z_cyl - 0.5) * L_auv; % Asse principale lungo Z locale

% Trasformiamo le coordinate del cilindro allineate lungo X Body (Surge)
geom_x = z_cyl;
geom_y = x_cyl;
geom_z = y_cyl;
auv_patch = surf(ax, geom_x, geom_y, geom_z, 'FaceColor', [1 0.6 0.1], ...
                 'EdgeColor', 'none', 'FaceAlpha', 0.95);
camlight('headlight'); lighting gouraud;
%{
% Box Informativo Overlay
txt_info = text(ax, 0.03, 0.92, '', 'Units', 'normalized', 'Color', 'w', ...
                'FontSize', 11, 'FontName', 'Courier', 'FontWeight', 'bold', ...
                'BackgroundColor', [0.05 0.05 0.08 0.7], 'EdgeColor', [0.3 0.3 0.3]);
%}
% Velocità animazione (step di campionamento)
step = 10; % Disegna 1 punto ogni 10 (10 Hz effettivi con IMU a 100 Hz)

%% Loop di Animazione
for k = 1:step:N_anim
    if ~ishandle(fig), break; end
    
    % Aggiorna scie
    set(h_path_true, 'XData', p_true(1:k, 2), 'YData', p_true(1:k, 1), 'ZData', -p_true(1:k, 3));
    set(h_path_ekf,  'XData', p_ekf(1:k, 2),  'YData', p_ekf(1:k, 1),  'ZData', -p_ekf(1:k, 3));
    
    % Posizione corrente EKF [Est, Nord, Quota]
    p_cur = [p_ekf(k, 2), p_ekf(k, 1), -p_ekf(k, 3)];
    
    % Aggiorna fasci LBL verso l'AUV
    for b = 1:4
        set(h_beams(b), 'XData', [P_bases_ned(b,2), p_cur(1)], ...
                        'YData', [P_bases_ned(b,1), p_cur(2)], ...
                        'ZData', [-P_bases_ned(b,3), p_cur(3)]);
    end
    
    % Orientazione veicolo da eul_e (NED standard ZYX)
    ph = eul_e(k, 1); th = eul_e(k, 2); ps = eul_e(k, 3);
    C_b_n = [cos(th)*cos(ps), sin(ph)*sin(th)*cos(ps)-cos(ph)*sin(ps), cos(ph)*sin(th)*cos(ps)+sin(ph)*sin(ps);
             cos(th)*sin(ps), sin(ph)*sin(th)*sin(ps)+cos(ph)*cos(ps), cos(ph)*sin(th)*sin(ps)-sin(ph)*cos(ps);
            -sin(th),         sin(ph)*cos(th),                         cos(ph)*cos(th)];
    
    % Ruota e trasla vertici AUV per il frame grafico [Est, Nord, Up]
    pts = [geom_x(:), geom_y(:), geom_z(:)]';
    pts_rot = C_b_n * pts;
    
    % Mappatura NED -> Grafico: X_gr = Nord, Y_gr = Est, Z_gr = -Down
    X_body_plot = reshape(pts_rot(2,:) + p_cur(1), size(geom_x));
    Y_body_plot = reshape(pts_rot(1,:) + p_cur(2), size(geom_y));
    Z_body_plot = reshape(-pts_rot(3,:) + p_cur(3), size(geom_z));
    
    set(auv_patch, 'XData', X_body_plot, 'YData', Y_body_plot, 'ZData', Z_body_plot);
    
    % Telemetria a schermo
    str_tel = sprintf('T: %6.1f s | Q: %4.1f m\nVel:  %4.2f m/s\nYaw: %+6.1f°\nPitch:%+6.1f°\nRoll: %+6.1f°', ...
                      t_a(k), p_ekf(k,3), norm(est_data(k,4:6)), ...
                      rad2deg(ps), rad2deg(th), rad2deg(ph));
    %set(txt_info, 'String', str_tel);
    
    drawnow limitrate;
end