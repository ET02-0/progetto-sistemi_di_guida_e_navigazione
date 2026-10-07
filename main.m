%% main.m
close all
clear
clc

format short

% Seed fisso per riproducibilita': stesso rumore sensori ad ogni run,
% utile per confrontare modifiche al filtro sugli stessi dati e per i
% futuri run Monte Carlo (dove invece si cicla su piu' seed espliciti).
rng(42);

%% Include dirs
% Se creerai delle cartelle per le funzioni, aggiungile qui
% addpath("KF_utilities/");

%% Waypoints e Traiettoria
traj_sel = 2;  % Seleziona la traiettoria AUV
waypoints_gen;        % Genera i punti

%% Generazione Traiettoria Completa
% Qui chiamerai lo script che interpola i waypoint. 
% Useremo la cinematica per ricavare omega e accelerazioni ideali.
trajectory_gen_cartesian; 

%% Setup Sensori (Rumori e Covarianze)
sensors_auv;

%% Simulazione
out = sim('rov_sim');

%% Plot dei risultati
plot_results;

%% Validazione
validation_auv;
calcolo_rmse;

%% Osservabilità
observability;

%% Animazione 3D
animazione_3d;

%% Esportazione grafici
export_figures;