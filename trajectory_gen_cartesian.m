g = 9.81;  % Accelerazione di gravità [m/s^2]

AutoOrientation = 0;
%% Generazione Traiettoria (Flat Earth - NED)

% Generazione traiettoria utilizzando i waypoint forniti da WP_Gen.m
if AutoOrientation == 1
    trajectory = waypointTrajectory(waypoints_ned(:,2:4), ...
    'TimeOfArrival',waypoints_ned(:,1), 'AutoPitch', true, 'AutoBank', true,'SampleRate',1/ST);
else
    % Assumiamo che waypoints_ned(:,5:7) sia [yaw, pitch, roll] in gradi
    trajectory = waypointTrajectory(waypoints_ned(:,2:4), ...
    'TimeOfArrival', waypoints_ned(:,1), ...
    'Orientation', quaternion(waypoints_ned(:,5:7),"eulerd","ZYX","frame"), ...
    'SampleRate', 1/ST);
end

% tInfo restituisce la tabella dei vincoli specificati
tInfo = waypointInfo(trajectory);
t_end = tInfo.TimeOfArrival(end);

%% Estrazione Campioni Traiettoria Ideale
trajectory.reset();
time = (0:ST:(tInfo.TimeOfArrival(end)-ST))';                                      

% Preallocazione vettori (Nav = Navigation frame, Body = Body frame)
quat_b = zeros(length(time),1,"quaternion");    % Quaternione da nav a body, espresso in body
vel_n = zeros(length(time),3);                  % Velocità in nav frame [m/s]
acc_n = zeros(length(time),3);                  % Accelerazione in nav frame [m/s^2]
pos_ned = zeros(length(time),3);                % Posizione in NED [m]
omega_n = zeros(length(time),3);                % Velocità angolare in nav frame [rad/s]

count = 1;
while ~isDone(trajectory)
   [pos_ned(count,:), quat_b(count), vel_n(count,:), acc_n(count,:), omega_n(count,:)] = trajectory();  
   count = count + 1;
end

%% Plot Traiettoria Ideale (Verifica 3D)
figure('Name', 'Traiettoria 3D AUV');
plot3(tInfo.Waypoints(:,2), tInfo.Waypoints(:,1), -tInfo.Waypoints(:,3), "r*", 'MarkerSize', 8)
hold on; grid on; axis equal;
plot3(pos_ned(:,2), pos_ned(:,1), -pos_ned(:,3), "b", 'LineWidth', 2);
title("Traiettoria AUV (Flat Earth)")
xlabel("East [m]")
ylabel("North [m]")
zlabel("Up (Negative Down) [m]")
legend('Waypoints', 'Traiettoria Continua')
hold off

%% Creazione Misure Inerziali Ideali (Body Frame)
omega_b = zeros(length(time),3);
acc_b = zeros(length(time),3);
rpy = zeros(length(time),3);

for idx=1:length(time)
    q_in = quat_b(idx,:);    
    rm = rotmat(q_in,'frame');                              % Matrice di rotazione da nav a body
    
    % Trasformazione grandezze da Nav a Body
    omega_b(idx,:) = (rm * omega_n(idx,:)')';               
    
    % Per l'accelerometro in body frame, sommiamo l'accelerazione cinematica e togliamo la gravità (che è [0;0;g] in NED)
    g_n = [0; 0; g];
    acc_b(idx,:) = (rm * (acc_n(idx,:)' - g_n))';     
    
    % Estrazione angoli di Eulero (Roll, Pitch, Yaw): NON tramite
    % rotm2eul(rm,'ZYX'). rm e' nav->body (verificato numericamente:
    % rm = C_b_n^T, vedi test_attitude_convention.m), ma rotm2eul con
    % 'ZYX' si aspetta l'altra convenzione (body->nav): il risultato non
    % era un semplice segno invertito, erano angoli sbagliati (es. con
    % input 10/20/30 deg usciva 1.12/-22.24/-28.45 deg). Questo
    % corrompeva la ground truth di assetto usata poi in
    % calcolo_rmse.m e plot_results.m.
    %
    % Si inverte invece direttamente la stessa C_b_n(phi,theta,psi) usata
    % in EKF_Predict.m / Meccanizzazione_Pura.m
    % (C_b_n = Rz(psi)*Ry(theta)*Rx(phi), body->nav) con la formula
    % chiusa standard per la sequenza 321/ZYX, applicata a C_b_n = rm.':
    Cbn = rm.';
    phi_k   = atan2(Cbn(3,2), Cbn(3,3));
    theta_k = asin(-Cbn(3,1));
    psi_k   = atan2(Cbn(2,1), Cbn(1,1));
    rpy(idx,:) = [phi_k, theta_k, psi_k]; % [Roll, Pitch, Yaw]
end

%% INIEZIONE DISTURBO: Correnti Marine
disp('Aggiunta correnti marine alla Ground Truth...');

% Definiamo una corrente marina costante (es. circa 1 nodo)
% [Nord, Est, Down] in m/s
v_current = [0.4, 0.3, 0.0]; 

% 1. Aggiorniamo la Velocità Reale (Rispetto al fondale)
% La velocità assoluta è la somma della velocità nei fluidi (propulsione) + corrente
vel_n = vel_n + repmat(v_current, length(time), 1);

% 2. Aggiorniamo la Posizione Reale (Deriva)
% Ricalcoliamo la posizione integrando la nuova velocità assoluta.
% L'AUV verrà fisicamente trascinato fuori dalla rotta ideale tracciata dai waypoint.
pos_ned = pos_ned + time * v_current;

% NOTA FISICA: 
% L'accelerazione inerziale (acc_n) NON viene modificata! 
% Poiché la corrente è costante, la sua derivata è zero. L'IMU non "sente" la corrente.
% L'orientazione (omega_b e angoli di Eulero) NON viene modificata, assumendo 
% che l'AUV continui a puntare il muso verso la rotta originale (crab angle).

% Plot Angoli di Eulero (Verifica Assetto)
figure('Name', 'Angoli di Eulero (Ideali)'); 
plot(time, rpy(:,1)*180/pi, 'r', 'LineWidth', 1.5); hold on;
plot(time, rpy(:,2)*180/pi, 'g', 'LineWidth', 1.5);
plot(time, rpy(:,3)*180/pi, 'b', 'LineWidth', 1.5);
legend("Roll","Pitch","Yaw")
title("Assetto Ideale AUV")
xlabel("Time [s]")
ylabel("[deg]")
grid on; hold off;