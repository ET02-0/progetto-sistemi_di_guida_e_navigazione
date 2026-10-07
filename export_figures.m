% Salva tutte le figure aperte in results/figures/ come PDF vettoriali,
% senza le icone della toolbar MATLAB (quelle che compaiono nei grafici
% della relazione se si fa uno screenshot), con nomi leggibili basati sul
% campo 'Name' di ciascuna figura.
%
% Da lanciare DOPO aver prodotto tutti i grafici che si vogliono mettere
% in relazione, es. dopo main.m con show_diagnostics = false (cosi' non
% esporti anche le figure diagnostiche intermedie).

out_dir = fullfile('results', 'figures');
if ~exist(out_dir, 'dir')
    mkdir(out_dir);
end

figs = findobj('Type', 'figure');
fprintf('Trovate %d figure aperte da esportare in %s\n', numel(figs), out_dir);

for k = 1:numel(figs)
    fig = figs(k);
    fig_name = get(fig, 'Name');
    if isempty(fig_name)
        fig_name = sprintf('figura_%d', fig.Number);
    end
    % Nome file sicuro: solo lettere, numeri, underscore
    safe_name = regexprep(fig_name, '[^a-zA-Z0-9]', '_');
    safe_name = regexprep(safe_name, '_+', '_');
    file_path = fullfile(out_dir, [safe_name, '.pdf']);

    try
        exportgraphics(fig, file_path, 'ContentType', 'vector');
        fprintf('  [%d/%d] %s -> %s\n', k, numel(figs), fig_name, file_path);
    catch ME
        warning('Esportazione fallita per "%s": %s', fig_name, ME.message);
    end
end

fprintf('Esportazione completata.\n');