Sistemi di Guida e Navigazione: Navigazione Integrata AUV tramite EKF

Questo repository contiene il codice sorgente (MATLAB/Simulink) e la relazione tecnica sviluppati per il progetto del corso di Sistemi di Guida e Navigazione della Laurea Magistrale in Robotica e Automazione dell'Università di Pisa (A.A. 2025/2026).

Panoramica del Progetto: L'obiettivo del lavoro è la progettazione, simulazione e validazione di un sistema di navigazione inerziale integrato per un veicolo sottomarino autonomo (AUV). Il sistema si basa su un Filtro di Kalman Esteso (EKF) ad architettura "coupled" a 12 stati, sviluppato per fondere i dati ad alta frequenza di una piattaforma inerziale con misurazioni acustiche esterne. Lo scopo primario è abbattere la progressiva divergenza degli errori spaziali caratteristica della navigazione Strapdown (Dead Reckoning) pura.

Il sistema sviluppato è in grado di:

Simulare la dinamica sottomarina e l'architettura sensoriale: Modellazione cinematica 3D (su Terra Piatta) dell'AUV lungo una traiettoria operativa di tipo "Lawnmower". Il simulatore inietta intenzionalmente disturbi esogeni persistenti (correnti marine) e modella una suite eterogenea a 100 Hz comprendente IMU (con bias giroscopico incognito), rete acustica LBL, profondimetro e magnetometro.

Stimare lo stato in tempo reale (EKF Coupled): Implementazione in ambiente Simulink di un Filtro di Kalman Esteso per la stima congiunta di posizione, velocità, assetto (angoli di Eulero) e bias dei giroscopi. Il filtro elabora aggiornamenti di posizione basati su misure acustiche puramente angolari (Bearing-Only) e pseudo-misure di assetto pre-calcolate algebricamente (modulo ALG).

Validare l'osservabilità non-lineare: Conduzione di un'analisi formale e quantitativa tramite calcolo Jacobiano simbolico, dimostrando matematicamente che l'intero stato (bias compresi) risulta completamente osservabile a partire da un minimo di 2 transponder acustici, sebbene il sistema ne impieghi 4 per massimizzare la robustezza.

Reiettare disturbi cinematici severi: Dimostrazione della capacità dell'EKF di stimare correttamente la cinematica reale del veicolo (incluso lo scarroccio fluidodinamico non percepibile direttamente dagli accelerometri). Le prestazioni sono validate tramite un abbattimento dell'errore (RMSE) di 5 ordini di grandezza rispetto alla meccanizzazione in anello aperto, con tutti gli stati rigorosamente all'interno dei limiti teorici di confidenza statistica a ±3σ.
