% * ***************************************** *
% *  SERIAL FEMM WITH AUTO-SAVE & RESUME      *
% *  Saves After Each (Js, IEA) Combination   *
% *     Electrical Machine Design - NTUA      *
% *              Diploma Thesis               *
% *    Created: 29/09/2025 by F.G.Kyrgios     *
% *    Resume version: 02/2026                *
% * ***************************************** *
%
% Runs a magnetostatic FEMM analysis for every combination of
%   Js (current density [A/mm^2]) x IEA (current angle [deg]) x SR (rotor position [deg])
% and stores flux densities, magnet vector potential, phase flux linkages
% and torque in  results/<model>_complete_data.mat  (input of MTPAMaps.m)

%% 1. Initialize workspace
clear;
clc;

% Paths (repository root = parent folder of this file)
dir_Database = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(dir_Database));
addpath('C:\femm42\mfiles'); % FEMM MATLAB interface - change if FEMM is installed elsewhere

%% 2. Define basic geometric variables
P = 4;           % Pole pairs
Q = 48;          % Slots
deg = pi/180;
LowestHarmonic = P;

%% 4. Select FEMM file
[femmFile, femmPath] = uigetfile('*.fem','Select FEMM file');
if isequal(femmFile,0)
    disp('User canceled.');
    return;
end

%% 5. Angles and offsets
n = 360/LowestHarmonic;
dtheta = 2;   % Rotor position step [deg]
electricalAngleOffset = -(360/Q)*P;
RotorAngleOffset = 360/(2*P);

%% 6. Initialize storage arrays
Js_values = [0.001, 1, 2, 4, 6, 8, 10, 12, 14, 16, 18, 20, 22, 24];   % Grid used for the MP6-1 example
IEA_values = [0 15 30 45 60 75 90 95 100 105 120 135 150 165 180];
SR_values = 0:dtheta:n;

num_Js = length(Js_values);
num_IEA = length(IEA_values);
num_SR = length(SR_values);

%% 7. Setup save file
[~, femmName, ~] = fileparts(femmFile);
saveFolder = fullfile(dir_Database, 'results');
if ~exist(saveFolder, 'dir'), mkdir(saveFolder); end
matFile = fullfile(saveFolder, [femmName '_complete_data.mat']);

%% 8. Check for existing progress and resume
if exist(matFile, 'file')
    fprintf('========================================\n');
    fprintf('  FOUND EXISTING PROGRESS FILE\n');
    fprintf('========================================\n');
    
    % Load saved data
    saved_data = load(matFile, 'data_store', 'progress', 'Js_values', 'IEA_values', 'SR_values');
    
    % Determine actual dimensions from the data structures themselves
    [old_num_Js, old_num_IEA] = size(saved_data.data_store.A);
    
    fprintf('Saved file info:\n');
    fprintf('  Saved Js_values variable: %d entries\n', length(saved_data.Js_values));
    fprintf('  Actual data_store.A size: %dx%d\n', old_num_Js, old_num_IEA);
    fprintf('  Completed: %d/%d combinations\n', saved_data.progress.completed, saved_data.progress.total);
    fprintf('\nCurrent script:\n');
    fprintf('  Js_values: %d entries\n', num_Js);
    fprintf('  IEA_values: %d entries\n', num_IEA);
    fprintf('========================================\n');
    
    % Check if we need to expand or if dimensions match
    if num_Js == old_num_Js && num_IEA == old_num_IEA
        % Dimensions match - just resume
        fprintf('\n✓ Dimensions match. Resuming from saved progress...\n\n');
        
        data_store = saved_data.data_store;
        progress = saved_data.progress;
        
        % Update total in case it was wrong
        progress.total = num_Js * num_IEA;
        
        resume_mode = true;
        
    elseif num_Js >= old_num_Js && num_IEA >= old_num_IEA
        % Need to expand
        fprintf('\n✓ Expanding data structures from %dx%d to %dx%d...\n\n', ...
                old_num_Js, old_num_IEA, num_Js, num_IEA);
        
        % Initialize expanded data structures
        data_store = initialize_data_store(num_Js, num_IEA);
        
        % Copy old data into new structures
        data_store.A(1:old_num_Js, 1:old_num_IEA) = saved_data.data_store.A;
        data_store.B(1:old_num_Js, 1:old_num_IEA) = saved_data.data_store.B;
        data_store.fluxA(1:old_num_Js, 1:old_num_IEA) = saved_data.data_store.fluxA;
        data_store.fluxB(1:old_num_Js, 1:old_num_IEA) = saved_data.data_store.fluxB;
        data_store.fluxC(1:old_num_Js, 1:old_num_IEA) = saved_data.data_store.fluxC;
        data_store.torque(1:old_num_Js, 1:old_num_IEA) = saved_data.data_store.torque;
        
        % Copy mesh data if it exists
        if isfield(saved_data.data_store, 'mesh')
            data_store.mesh = saved_data.data_store.mesh;
        end
        
        % Expand progress tracking
        progress = struct();
        progress.completed_list = false(num_Js, num_IEA);
        progress.completed_list(1:old_num_Js, 1:old_num_IEA) = saved_data.progress.completed_list;
        progress.completed = saved_data.progress.completed;
        progress.total = num_Js * num_IEA;
        progress.last_Js_idx = saved_data.progress.last_Js_idx;
        progress.last_IEA_idx = saved_data.progress.last_IEA_idx;
        
        fprintf('Expansion complete:\n');
        fprintf('  Old combinations: %d (%dx%d)\n', old_num_Js*old_num_IEA, old_num_Js, old_num_IEA);
        fprintf('  New combinations: %d (%dx%d)\n', num_Js*num_IEA, num_Js, num_IEA);
        fprintf('  Already completed: %d\n', progress.completed);
        fprintf('  New combinations to process: %d\n', progress.total - progress.completed);
        
        resume_mode = true;
        
    else
        fprintf('\nWARNING: New parameter arrays are smaller than saved ones!\n');
        fprintf('  Cannot resume. Starting fresh...\n\n');
        resume_mode = false;
        data_store = initialize_data_store(num_Js, num_IEA);
        progress = struct('completed', 0, 'total', num_Js * num_IEA, ...
                         'completed_list', false(num_Js, num_IEA), ...
                         'last_Js_idx', 0, 'last_IEA_idx', 0);
    end
    
else
    % Initialize new data structure
    fprintf('No existing progress file found. Starting fresh...\n\n');
    resume_mode = false;
    data_store = initialize_data_store(num_Js, num_IEA);
    progress = struct('completed', 0, 'total', num_Js * num_IEA, ...
                     'completed_list', false(num_Js, num_IEA), ...
                     'last_Js_idx', 0, 'last_IEA_idx', 0);
end
%% 9. Open FEMM once
openfemm;
opendocument(fullfile(femmPath, femmFile));

%% 10. Main processing loop
overall_start = tic;
combinations_this_session = 0;

for Js_idx = 1:num_Js
    Js = Js_values(Js_idx);
    
    for IEA_idx = 1:num_IEA
        IEA = IEA_values(IEA_idx);
        
        % Skip if already completed
        if progress.completed_list(Js_idx, IEA_idx)
            fprintf('[SKIP] Js=%.1f, IEA=%.1f° (already completed)\n', Js, IEA);
            continue;
        end
        
        combo_num = (Js_idx-1)*num_IEA + IEA_idx;
        fprintf('\n[%d/%d] Processing Js=%.1f, IEA=%.1f°\n', ...
                combo_num, num_Js*num_IEA, Js, IEA);
        combo_start = tic;
        
        try
            % Wire properties
            % Conductor area [mm^2]: Is = Awire*Js. MUST match Awire in MTPAMaps.m
            Awire = 3.86035;
            % Awire = 1.76714586764; % Stranded
            Is = (Awire * Js);
            IdAmp = Is*cos(IEA*deg);
            IqAmp = Is*sin(IEA*deg);
            
            % Loop over rotor positions
            for SR_idx = 1:num_SR
                SR = SR_values(SR_idx);
                
                % Set rotor angle
                mi_modifyboundprop('SlidingBand', 10, RotorAngleOffset + SR);
                
                % Current angle
                tta = P * SR * deg + electricalAngleOffset * deg;
                
                % Park Transformation
                Id = [cos(tta), cos(tta-2*pi/3), cos(tta+2*pi/3)];
                Iq = -[sin(tta), sin(tta-2*pi/3), sin(tta+2*pi/3)];
                Itot = IdAmp*Id + IqAmp*Iq;
                
                % Set currents
                mi_setcurrent('A', Itot(1));
                mi_setcurrent('B', Itot(2));
                mi_setcurrent('C', Itot(3));
                
                % Analyze
                mi_analyze(1);
                mi_loadsolution;
                mo_smooth('off');
                
                if (SR_idx == 1)
                    % Get mesh info on first iteration
                    nn = mo_numelements;
                    b = zeros(num_SR, nn);
                    A = zeros(num_SR, nn);
                    z = zeros(nn, 1);
                    a = zeros(nn, 1);
                    g = zeros(nn, 1);
                    
                    for m = 1:nn
                        elm = mo_getelement(m);
                        z(m) = elm(4) + 1j*elm(5);
                        a(m) = elm(6);
                        g(m) = elm(7);
                    end
                end
                
                % Store flux densities
                for m = 1:nn
                    if(g(m) == 302)
                        A(SR_idx, m) = mo_geta(real(z(m)), imag(z(m)));
                    elseif (g(m) == 101 || g(m) == 201 || g(m) == 103)
                        b(SR_idx, m) = (mo_getb(real(z(m)), imag(z(m))) * [1; 1j]);
                    end
                end
                
                % Compute flux
                circpropsA = mo_getcircuitproperties('A');
                circpropsB = mo_getcircuitproperties('B');
                circpropsC = mo_getcircuitproperties('C');
                
                fluxA_store(SR_idx) = circpropsA(3);
                fluxB_store(SR_idx) = circpropsB(3);
                fluxC_store(SR_idx) = circpropsC(3);
                
                % Compute torque (rotor block labels - model specific coordinates [mm])
                mo_selectblock(45, 18);
                mo_selectblock(35, 14);
                torque_store(SR_idx) = mo_blockintegral(22);
                mi_clearselected();
                
                mo_close;
            end
            
            % Store results for this combination
            data_store.A{Js_idx, IEA_idx} = A;
            data_store.B{Js_idx, IEA_idx} = b;
            data_store.fluxA{Js_idx, IEA_idx} = fluxA_store;
            data_store.fluxB{Js_idx, IEA_idx} = fluxB_store;
            data_store.fluxC{Js_idx, IEA_idx} = fluxC_store;
            data_store.torque{Js_idx, IEA_idx} = torque_store;
            
            % Store mesh info (once)
            if ~isfield(data_store, 'mesh')
                data_store.mesh.z = z;
                data_store.mesh.a = a;
                data_store.mesh.g = g;
                data_store.mesh.nn = nn;
            end
            
            % Update progress
            progress.completed_list(Js_idx, IEA_idx) = true;
            progress.completed = progress.completed + 1;
            progress.last_Js_idx = Js_idx;
            progress.last_IEA_idx = IEA_idx;
            combinations_this_session = combinations_this_session + 1;
            
            % SAVE IMMEDIATELY after each combination
            save(matFile, 'data_store', 'Js_values', 'IEA_values', 'SR_values', 'progress', '-v7.3');
            
            combo_time = toc(combo_start);
            mean_torque = mean(torque_store);
            
            % Calculate time estimates
            elapsed_total = toc(overall_start);
            remaining = progress.total - progress.completed;
            avg_time_per_combo = elapsed_total / combinations_this_session;
            est_remaining = remaining * avg_time_per_combo;
            
            fprintf('  ✓ Complete in %.1f min | Torque: %.3f Nm\n', combo_time/60, mean_torque);
            fprintf('  Progress: %d/%d (%.1f%%) | Elapsed: %.1f min | Est. remaining: %.1f min\n', ...
                    progress.completed, progress.total, ...
                    100*progress.completed/progress.total, ...
                    elapsed_total/60, est_remaining/60);
            fprintf('  ✓ Saved to: %s\n', matFile);
            
        catch ME
            fprintf('  ✗ ERROR: %s\n', ME.message);
            fprintf('  Skipping this combination and continuing...\n');
            
            % Save error info
            progress.last_error.Js_idx = Js_idx;
            progress.last_error.IEA_idx = IEA_idx;
            progress.last_error.message = ME.message;
            save(matFile, 'data_store', 'Js_values', 'IEA_values', 'SR_values', 'progress', '-v7.3');
            
            continue;
        end
    end
end

%% 11. Close FEMM
closefemm;

%% 12. Final summary
fprintf('\n========================================\n');
fprintf('  PROCESSING COMPLETE\n');
fprintf('========================================\n');
fprintf('Total combinations: %d/%d\n', progress.completed, progress.total);
fprintf('Total time: %.1f minutes (%.2f hours)\n', toc(overall_start)/60, toc(overall_start)/3600);
fprintf('Average time per combination: %.1f minutes\n', toc(overall_start)/60/combinations_this_session);
fprintf('Final data saved to: %s\n', matFile);

if progress.completed < progress.total
    fprintf('\nWARNING: %d combinations failed or were skipped.\n', progress.total - progress.completed);
    fprintf('Re-run this script to retry failed combinations.\n');
end

fprintf('========================================\n');

%% ========================================================================
%% HELPER FUNCTION
%% ========================================================================

function data_store = initialize_data_store(num_Js, num_IEA)
    data_store = struct();
    data_store.A = cell(num_Js, num_IEA);
    data_store.B = cell(num_Js, num_IEA);
    data_store.fluxA = cell(num_Js, num_IEA);
    data_store.fluxB = cell(num_Js, num_IEA);
    data_store.fluxC = cell(num_Js, num_IEA);
    data_store.torque = cell(num_Js, num_IEA);
end