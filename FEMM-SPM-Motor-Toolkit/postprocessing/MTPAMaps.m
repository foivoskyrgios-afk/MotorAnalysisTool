% * ***************************************** *
% *   MTPA + Field Weakening Efficiency Maps  *
% *     Electrical Machine Design - NTUA      *
% *              Diploma Thesis               *
% *    Created: 02/2026 by F.G.Kyrgios        *
% *    Field weakening revision: 09/2026      *
% * ***************************************** *
%
% MTPA + FIELD WEAKENING maps from FEMM field data
% Find optimal (Js, IEA) for each (RPM, Torque) that minimizes current
% subject to the inverter voltage and current limits
% Plot loss / power / efficiency heatmaps at optimal conditions
% RPM on X-axis, Torque on Y-axis
% EXCLUDING FIRST Js VALUE (0.001)
% PLOTS LIMITED TO T_max_plot MAXIMUM
%
% FIELD WEAKENING HANDLING:
%  - V and I are computed for EVERY (Js, IEA, RPM) point (no pre-masking),
%    so the interpolated V/I surfaces are smooth up to and past the limits
%  - Inverter limits are explicit fmincon inequality constraints:
%       V(Js, IEA, RPM) <= V_limit,  I(Js, IEA, RPM) <= I_limit
%    When V hits the limit the optimizer moves to IEA > 90 deg (negative Id)
%    -> field weakening
%  - The max torque envelope T_env(RPM) is computed per speed and every
%    (RPM, Torque) point above it is left EMPTY (no nearest-neighbour fill),
%    so the torque fall-off at high speed is visible on the maps
%  - RPM range must extend past the base speed, otherwise V never reaches
%    V_limit and no field weakening region exists (see RPM_max)

clear; clc;

%% 1. Load the saved data
[file, path] = uigetfile('*.mat', 'Select the saved _complete_data.mat file');
if isequal(file, 0)
    disp('User canceled.');
    return;
end

load(fullfile(path, file));

%% EXCLUDE FIRST Js VALUE
fprintf('Original Js_values: %d points\n', length(Js_values));
fprintf('Excluding first Js value (%.3f)\n', Js_values(1));

Js_values = Js_values(2:end);
num_Js = length(Js_values);

fprintf('New Js_values: %d points\n\n', num_Js);

%% 2. Define loss calculation parameters
P = 4;
Q = 48;
L = 48;
m = 3;
Diameter = 100*10^(-3);
SPD = (pi*Diameter)/Q;
Wts = 2*SPD;
t = gcd(Q, 2*P);
deg = pi/180;

Turns = 13;
FillFactor = 0.778;
Dwire = 2.25676 * 10^(-3);
TemperatureRise = 150;

omag = 0.556*10^6;
ce = 0.53;
ch = 108.7;
cs = 0.95;

TempCoefficient = 0.00393;
CondWire = 58.*10^6;
owire = (CondWire)/(1+TemperatureRise*TempCoefficient);
cePhase = (pi^2/8)*Dwire^2*FillFactor*owire;

r_Cu20 = 1.75 * 10^(-6) * (10^(-2));
r_Cu = r_Cu20*(1+TempCoefficient*TemperatureRise);
l_turn = 1.2*(2*L+2*Wts)*(10^(-3));
Awire = 3.86035 * 10^(-6);
TotalPhaseResistance = (Turns*Q)/m * r_Cu * l_turn/Awire;

%% 3. Process each (Js, IEA) combination - STARTING FROM INDEX 2

electricalAngleOffset = -(360/Q)*P;
RotorAngleOffset = 360/(2*P);

num_IEA = length(IEA_values);
num_SR  = length(SR_values);

% Speed range - MUST go past the base speed to see field weakening.
% With the previous 10:10:400 range the voltage at the optimum only reached
% ~34 V (< 60 V limit), so the voltage limit was never active.
RPM_step = 10;
RPM_max  = 1500;
RPM_values = RPM_step:RPM_step:RPM_max;
num_RPM = length(RPM_values);

% Inverter limits
V_limit = 60;   % Maximum inverter peak phase voltage [V] - compared with Vtotal = |Vdq| (amplitude-invariant -> peak)
I_limit = 60;   % Maximum inverter peak phase current [A] - compared with Itotal = |Idq| (peak)
fprintf('Inverter limits: V <= %.0f V, I <= %.0f A\n', V_limit, I_limit);
fprintf('Speed range: %d to %d RPM (%d points)\n\n', RPM_values(1), RPM_values(end), num_RPM);

% Initialize results matrix - NO masking, every point is stored
Idin = nan(num_Js, num_IEA, num_RPM);
Iqin = nan(num_Js, num_IEA, num_RPM);
Vdin = nan(num_Js, num_IEA, num_RPM);
Vqin = nan(num_Js, num_IEA, num_RPM);
Pin  = nan(num_Js, num_IEA, num_RPM);
Pm   = nan(num_Js, num_IEA, num_RPM);
Rs   = nan(num_Js, num_IEA, num_RPM);
BackEMFD = nan(num_Js, num_IEA, num_RPM);
BackEMFQ = nan(num_Js, num_IEA, num_RPM);

PhaseOhmic_loss = nan(num_Js, num_IEA, num_RPM);
Prox_loss       = nan(num_Js, num_IEA, num_RPM);
Hyst_loss       = nan(num_Js, num_IEA, num_RPM);
Eddy_loss       = nan(num_Js, num_IEA, num_RPM);
Magnet_loss     = nan(num_Js, num_IEA, num_RPM);
Total_loss      = nan(num_Js, num_IEA, num_RPM);
Vinverter       = nan(num_Js, num_IEA, num_RPM);
Iinverter       = nan(num_Js, num_IEA, num_RPM);
Is_stator       = nan(num_Js, num_IEA);
Efficiency      = nan(num_Js, num_IEA, num_RPM);
Torque          = nan(num_Js, num_IEA);

fprintf('Processing FEA data for all operating points...\n');

SR_rad = deg2rad(SR_values);

for Js_idx = 1:num_Js
    Js = Js_values(Js_idx);

    for IEA_idx = 1:num_IEA
        IEA = IEA_values(IEA_idx);

        fprintf('Processing Js=%.1f, IEA=%.1f...\n', Js, IEA);

        data_Js_idx = Js_idx + 1;

        A      = data_store.A{data_Js_idx, IEA_idx};
        b      = data_store.B{data_Js_idx, IEA_idx};
        fluxA  = t * L * data_store.fluxA{data_Js_idx, IEA_idx};
        fluxB  = t * L * data_store.fluxB{data_Js_idx, IEA_idx};
        fluxC  = t * L * data_store.fluxC{data_Js_idx, IEA_idx};
        torque = t * L * data_store.torque{data_Js_idx, IEA_idx};
        meanTorque = mean(torque);

        a_mesh = data_store.mesh.a;
        g      = data_store.mesh.g;

        Is     = (Awire * Js * 10^6);
        IdAmp  = Is * cos(IEA * deg);
        IqAmp  = Is * sin(IEA * deg);
        Iphase = sqrt(IdAmp^2 + IqAmp^2) / sqrt(2);

        PhaseOhmic = 3 * TotalPhaseResistance * Iphase^2;

        Is_stator(Js_idx, IEA_idx) = Is;

        ns = num_SR;

        bxfft = abs(fft(real(b))) * (2/ns);
        byfft = abs(fft(imag(b))) * (2/ns);
        bsq   = (bxfft .* bxfft) + (byfft .* byfft);

        lengthunits = 0.001;
        v = a_mesh * L * lengthunits^3;

        Jm   = fft(A) * (2/ns);
        gmag = (g == 302);
        vmag = v'*gmag;
        vcol = v.*gmag;
        Jo   = (Jm*vcol) / vmag;
        Jm   = Jm - Jo * gmag';

        % ---- Speed-independent pre-computation (hoisted out of RPM loop) ----
        % dFlux/dTheta and its dq transform do not depend on speed:
        % V_dq = omega * dFlux_dq/dTheta
        dfluxA_dSR = ppval(fnder(spline(SR_rad, fluxA), 1), SR_rad);
        dfluxB_dSR = ppval(fnder(spline(SR_rad, fluxB), 1), SR_rad);
        dfluxC_dSR = ppval(fnder(spline(SR_rad, fluxC), 1), SR_rad);

        dVoltD = zeros(size(SR_values));
        dVoltQ = zeros(size(SR_values));

        for k = 1:num_SR
            SR  = SR_values(k);
            tta = P*SR*deg + electricalAngleOffset*deg;
            Va  = 2/3 * [cos(tta),        -sin(tta)       ];
            Vb  = 2/3 * [cos(tta-2*pi/3), -sin(tta-2*pi/3)];
            Vc  = 2/3 * [cos(tta+2*pi/3), -sin(tta+2*pi/3)];
            dVoltDQ   = dfluxA_dSR(k)*Va + dfluxB_dSR(k)*Vb + dfluxC_dSR(k)*Vc;
            dVoltD(k) = dVoltDQ(1);
            dVoltQ(k) = dVoltDQ(2);
        end

        EMFD_per_omega = mean(dVoltD);
        EMFQ_per_omega = mean(dVoltQ);

        % Volume-weighted B^2 / Jm^2 spectra per region (speed independent)
        bsq_rotor  = bsq * (v .* (g == 201));
        bsq_stator = bsq * (v .* (g == 101));
        bsq_wind   = bsq * (v .* (g == 103));
        Jm2v       = (abs(Jm).^2) * v;

        harmonics = 0:(ns-1);
        harmonics = harmonics .* (harmonics < (ns/2));

        for RPM_idx = 1:num_RPM
            RPM       = RPM_values(RPM_idx);
            Frequency = RPM / 60;
            w         = Frequency * harmonics;
            omega     = RPM * (2*pi)/60;

            BackEMFD(Js_idx, IEA_idx, RPM_idx) = omega * EMFD_per_omega;
            BackEMFQ(Js_idx, IEA_idx, RPM_idx) = omega * EMFQ_per_omega;

            % Compute all loss components
            hysteresis_rotor_loss = ((ch * w) * bsq_rotor) / cs;
            eddy_rotor_loss = ((ce * (w .* w)) * bsq_rotor) / cs;
            rotor_loss = t * (hysteresis_rotor_loss + eddy_rotor_loss);

            hysteresis_stator_loss = ((ch * w) * bsq_stator) / cs;
            eddy_stator_loss = ((ce * (w .* w)) * bsq_stator) / cs;
            stator_loss = t * (hysteresis_stator_loss + eddy_stator_loss);

            hysteresis_loss = t * (hysteresis_stator_loss + hysteresis_rotor_loss);
            eddy_loss = t * (eddy_stator_loss + eddy_rotor_loss);
            P_parallel = (2/3)*(eddy_loss + hysteresis_loss);

            prox_loss = t * ((cePhase * w .* w) * bsq_wind);

            magnet_loss = t * (1/2) * ((omag * (2*pi*w).^2) * Jm2v);
            P_series = (2/3)*(prox_loss + PhaseOhmic);
            total_loss = rotor_loss + stator_loss + prox_loss + PhaseOhmic + magnet_loss;

            Rm_val = (BackEMFD(Js_idx, IEA_idx, RPM_idx)^2 + BackEMFQ(Js_idx, IEA_idx, RPM_idx)^2) / P_parallel;

            Rs(Js_idx, IEA_idx, RPM_idx) = P_series / (IdAmp^2 + IqAmp^2);

            Idin(Js_idx, IEA_idx, RPM_idx) = IdAmp + BackEMFD(Js_idx, IEA_idx, RPM_idx)/Rm_val;
            Vdin(Js_idx, IEA_idx, RPM_idx) = Idin(Js_idx, IEA_idx, RPM_idx) * Rs(Js_idx, IEA_idx, RPM_idx) + BackEMFD(Js_idx, IEA_idx, RPM_idx);
            Iqin(Js_idx, IEA_idx, RPM_idx) = IqAmp + BackEMFQ(Js_idx, IEA_idx, RPM_idx)/Rm_val;
            Vqin(Js_idx, IEA_idx, RPM_idx) = Iqin(Js_idx, IEA_idx, RPM_idx) * Rs(Js_idx, IEA_idx, RPM_idx) + BackEMFQ(Js_idx, IEA_idx, RPM_idx);

            Vtotal = sqrt(Vdin(Js_idx, IEA_idx, RPM_idx)^2 + Vqin(Js_idx, IEA_idx, RPM_idx)^2);
            Itotal = sqrt(Idin(Js_idx, IEA_idx, RPM_idx)^2 + Iqin(Js_idx, IEA_idx, RPM_idx)^2);

            Pm(Js_idx, IEA_idx, RPM_idx) = meanTorque * omega;

            % Store everything - inverter limits are applied later as
            % optimizer constraints, not by deleting data here
            PhaseOhmic_loss(Js_idx, IEA_idx, RPM_idx) = PhaseOhmic;
            Prox_loss(Js_idx, IEA_idx, RPM_idx)       = prox_loss;
            Hyst_loss(Js_idx, IEA_idx, RPM_idx)       = hysteresis_loss;
            Eddy_loss(Js_idx, IEA_idx, RPM_idx)       = eddy_loss;
            Magnet_loss(Js_idx, IEA_idx, RPM_idx)     = magnet_loss;
            Total_loss(Js_idx, IEA_idx, RPM_idx)      = total_loss;
            Pin(Js_idx, IEA_idx, RPM_idx)             = Total_loss(Js_idx, IEA_idx, RPM_idx) + Pm(Js_idx, IEA_idx, RPM_idx);
            Efficiency(Js_idx, IEA_idx, RPM_idx)      = Pm(Js_idx, IEA_idx, RPM_idx) / (Total_loss(Js_idx, IEA_idx, RPM_idx) + Pm(Js_idx, IEA_idx, RPM_idx));
            Vinverter(Js_idx, IEA_idx, RPM_idx)       = Vtotal;
            Iinverter(Js_idx, IEA_idx, RPM_idx)       = Itotal;
        end

        Torque(Js_idx, IEA_idx) = meanTorque;
    end
end

feasible_raw = (Vinverter <= V_limit) & (Iinverter <= I_limit);
total_pts    = numel(feasible_raw);
masked_pts   = sum(~feasible_raw(:));
fprintf('\nFEA data processing complete!\n');
fprintf('Raw grid points violating limits: %d / %d (%.1f%%)\n', ...
    masked_pts, total_pts, 100*masked_pts/total_pts);
fprintf('Max raw Vinverter at %d RPM: %.1f V\n\n', RPM_values(end), max(Vinverter(:, :, end), [], 'all'));

if all(feasible_raw(:))
    warning(['The voltage/current limits are never reached in the whole speed range. ' ...
             'No field weakening will appear - increase RPM_max.']);
end

%% ========================================================================
%% INTERPOLATION FUNCTIONS
%% ========================================================================

% Interpolants are built directly on the FEA (Js, IEA) grid. 'makima' gives
% smooth surfaces (better fmincon gradients) without spline overshoot.
% The previous dense 3-D arrays are not needed (same information, GBs of RAM).

fprintf('Creating interpolation functions...\n');

make_F = @(M) griddedInterpolant({Js_values, IEA_values}, M, 'makima', 'none');

F_T = make_F(Torque);

F_Ohmic  = cell(num_RPM, 1);
F_Prox   = cell(num_RPM, 1);
F_Hyst   = cell(num_RPM, 1);
F_Eddy   = cell(num_RPM, 1);
F_Magnet = cell(num_RPM, 1);
F_Total  = cell(num_RPM, 1);
F_Eff    = cell(num_RPM, 1);
F_V      = cell(num_RPM, 1);
F_Iinv   = cell(num_RPM, 1);
F_Pm     = cell(num_RPM, 1);
F_Pin    = cell(num_RPM, 1);

for RPM_idx = 1:num_RPM
    F_Ohmic{RPM_idx}  = make_F(PhaseOhmic_loss(:, :, RPM_idx));
    F_Prox{RPM_idx}   = make_F(Prox_loss(:, :, RPM_idx));
    F_Hyst{RPM_idx}   = make_F(Hyst_loss(:, :, RPM_idx));
    F_Eddy{RPM_idx}   = make_F(Eddy_loss(:, :, RPM_idx));
    F_Magnet{RPM_idx} = make_F(Magnet_loss(:, :, RPM_idx));
    F_Total{RPM_idx}  = make_F(Total_loss(:, :, RPM_idx));
    F_Eff{RPM_idx}    = make_F(Efficiency(:, :, RPM_idx));
    F_V{RPM_idx}      = make_F(Vinverter(:, :, RPM_idx));
    F_Iinv{RPM_idx}   = make_F(Iinverter(:, :, RPM_idx));
    F_Pm{RPM_idx}     = make_F(Pm(:, :, RPM_idx));
    F_Pin{RPM_idx}    = make_F(Pin(:, :, RPM_idx));
end

% Dense search grid - used for the torque envelope and for initial guesses
Js_search  = linspace(min(Js_values),  max(Js_values),  400);
IEA_search = linspace(min(IEA_values), max(IEA_values), 721);
[Js_S, IEA_S] = ndgrid(Js_search, IEA_search);
T_S = F_T(Js_S, IEA_S);

fprintf('Interpolation functions created!\n\n');

%% ========================================================================
%% OPTIMIZER SETTINGS
%% ========================================================================

Js_min  = min(Js_values);
Js_max  = max(Js_values);
IEA_min = min(IEA_values);
IEA_max = max(IEA_values);

lb = [Js_min, IEA_min];
ub = [Js_max, IEA_max];

options = optimoptions('fmincon', ...
    'Display', 'off', ...
    'Algorithm', 'sqp', ...
    'MaxIterations', 400, ...
    'MaxFunctionEvaluations', 1200, ...
    'OptimalityTolerance', 1e-8, ...
    'StepTolerance', 1e-10, ...
    'ConstraintTolerance', 1e-6);

con_tol = 1e-3;   % accepted constraint violation [V] / [A]
T_tol   = 0.01;   % accepted torque error [Nm]

%% ========================================================================
%% MAXIMUM TORQUE ENVELOPE (MTPA below base speed, FW above)
%% ========================================================================

fprintf('========================================\n');
fprintf('Maximum Torque Envelope\n');
fprintf('========================================\n\n');

T_env = nan(num_RPM, 1);   % max torque at each speed within inverter limits
x_env = nan(num_RPM, 2);   % (Js, IEA) that gives it
V_env = nan(num_RPM, 1);

feas_S = cell(num_RPM, 1); % feasibility of the search grid per RPM

for RPM_idx = 1:num_RPM
    V_S = F_V{RPM_idx}(Js_S, IEA_S);
    I_S = F_Iinv{RPM_idx}(Js_S, IEA_S);
    feas_S{RPM_idx} = (V_S <= V_limit) & (I_S <= I_limit);

    T_feas = T_S;
    T_feas(~feas_S{RPM_idx}) = NaN;
    [T_grid_max, idx] = max(T_feas(:));

    if isnan(T_grid_max)
        continue;
    end

    x_best = [Js_S(idx), IEA_S(idx)];
    T_best = T_grid_max;

    % Refine the grid maximum with fmincon
    neg_torque = @(x) -F_T(x(1), x(2));
    nonlcon    = @(x) deal([F_V{RPM_idx}(x(1), x(2)) - V_limit; ...
                            F_Iinv{RPM_idx}(x(1), x(2)) - I_limit], []);
    try
        [x_opt, fval, exitflag] = fmincon(neg_torque, x_best, [], [], [], [], ...
            lb, ub, nonlcon, options);
        c_opt = nonlcon(x_opt);
        if exitflag > 0 && all(c_opt <= con_tol) && -fval > T_best
            x_best = x_opt;
            T_best = -fval;
        end
    catch
    end

    T_env(RPM_idx)    = T_best;
    x_env(RPM_idx, :) = x_best;
    V_env(RPM_idx)    = F_V{RPM_idx}(x_best(1), x_best(2));
end

% Base speed = first speed where the voltage limit is active at max torque
voltage_active = V_env >= V_limit - 0.5;
base_idx = find(voltage_active, 1, 'first');
if isempty(base_idx)
    base_RPM = NaN;
    fprintf('Voltage limit never active at max torque -> no field weakening in %d-%d RPM.\n', ...
        RPM_values(1), RPM_values(end));
    fprintf('Increase RPM_max to see the torque fall-off.\n\n');
else
    base_RPM = RPM_values(base_idx);
    fprintf('Base speed (voltage limit reached): ~%d RPM\n', base_RPM);
    fprintf('Max torque: %.2f Nm at %d RPM -> %.2f Nm at %d RPM\n\n', ...
        T_env(1), RPM_values(1), T_env(end), RPM_values(end));
end

%% ========================================================================
%% DEFINE OPTIMIZATION GRID WITH TORQUE LIMITS
%% ========================================================================

fprintf('========================================\n');
fprintf('Defining Optimization Grid\n');
fprintf('========================================\n\n');

T_min_valid = min(T_S(:));
T_max_valid = max(T_env, [], 'omitnan');

fprintf('Available torque range: %.4f to %.4f Nm\n', T_min_valid, T_max_valid);

T_min_plot = 0.5;
T_max_plot = 40;

T_min_opt = max(T_min_valid, T_min_plot);
T_max_opt = min(T_max_valid, T_max_plot);

fprintf('Optimization limited to: %.1f to %.1f Nm\n', T_min_opt, T_max_opt);

Torque_opt = linspace(T_min_opt, T_max_opt, 60);
RPM_opt    = RPM_values;

num_T_opt   = length(Torque_opt);
num_RPM_opt = length(RPM_opt);

fprintf('Optimization grid: %d RPM x %d Torque = %d points\n\n', ...
    num_RPM_opt, num_T_opt, num_RPM_opt * num_T_opt);

%% ========================================================================
%% MTPA / FIELD WEAKENING OPTIMIZATION
%% ========================================================================
%  min  Js                      (Is = Awire*Js -> minimum current)
%  s.t. T(Js, IEA)        = T_target
%       V(Js, IEA, RPM)  <= V_limit
%       I(Js, IEA, RPM)  <= I_limit
%  Below base speed the V constraint is inactive -> pure MTPA.
%  Above base speed it is active -> IEA increases (negative Id) -> FW.

fprintf('========================================\n');
fprintf('MTPA + Field Weakening Optimization\n');
fprintf('========================================\n\n');

Js_optimal  = nan(num_RPM_opt, num_T_opt);
IEA_optimal = nan(num_RPM_opt, num_T_opt);
Is_optimal  = nan(num_RPM_opt, num_T_opt);

PhaseOhmic_loss_opt = nan(num_RPM_opt, num_T_opt);
Prox_loss_opt       = nan(num_RPM_opt, num_T_opt);
Hyst_loss_opt       = nan(num_RPM_opt, num_T_opt);
Eddy_loss_opt       = nan(num_RPM_opt, num_T_opt);
Magnet_loss_opt     = nan(num_RPM_opt, num_T_opt);
Total_loss_opt      = nan(num_RPM_opt, num_T_opt);
Efficiency_opt      = nan(num_RPM_opt, num_T_opt);
Voltage_opt         = nan(num_RPM_opt, num_T_opt);
Vinverter_opt       = nan(num_RPM_opt, num_T_opt);
Iinverter_opt       = nan(num_RPM_opt, num_T_opt);
Pm_opt              = nan(num_RPM_opt, num_T_opt);
Pin_opt             = nan(num_RPM_opt, num_T_opt);

% exitflag_log:  >0 fmincon solution
%                 0 grid-search fallback (fmincon did not converge)
%               -98 torque above the max torque envelope (not reachable)
%               -99 no solution found
exitflag_log = nan(num_RPM_opt, num_T_opt);

objective = @(x) x(1);

fprintf('Starting optimization...\n\n');

tic;
for RPM_idx = 1:num_RPM_opt
    feas = feas_S{RPM_idx};

    for T_idx = 1:num_T_opt
        T_target = Torque_opt(T_idx);

        iter       = (RPM_idx-1)*num_T_opt + T_idx;
        total_iter = num_RPM_opt * num_T_opt;
        if mod(iter, 500) == 0 || iter == 1
            fprintf('Progress: %d/%d (%.1f%%) - Elapsed: %.1f s\n', ...
                iter, total_iter, 100*iter/total_iter, toc);
        end

        % Above the envelope -> torque not achievable at this speed.
        % Leave it EMPTY: this is the field-weakening fall-off.
        if isnan(T_env(RPM_idx)) || T_target > T_env(RPM_idx)
            exitflag_log(RPM_idx, T_idx) = -98;
            continue;
        end

        % Initial guess: feasible grid point with the lowest current that
        % gives at least T_target
        cand = feas & (T_S >= T_target);
        if any(cand(:))
            Js_cand = Js_S;
            Js_cand(~cand) = Inf;
            [~, idx0] = min(Js_cand(:));
            x0 = [Js_S(idx0), IEA_S(idx0)];
        else
            x0 = x_env(RPM_idx, :);
        end

        nonlcon = @(x) deal([F_V{RPM_idx}(x(1), x(2)) - V_limit; ...
                             F_Iinv{RPM_idx}(x(1), x(2)) - I_limit], ...
                            F_T(x(1), x(2)) - T_target);

        x_sol    = [];
        exitflag = -99;
        try
            [x_opt, ~, ef] = fmincon(objective, x0, [], [], [], [], ...
                lb, ub, nonlcon, options);
            [c_opt, ceq_opt] = nonlcon(x_opt);
            if ef > 0 && all(c_opt <= con_tol) && abs(ceq_opt) < T_tol
                x_sol    = x_opt;
                exitflag = ef;
            end
        catch
        end

        % Fallback: best feasible grid point with torque close to target
        if isempty(x_sol)
            near = feas & (abs(T_S - T_target) <= max(0.02*T_target, 0.05));
            if any(near(:))
                Js_near = Js_S;
                Js_near(~near) = Inf;
                [~, idxn] = min(Js_near(:));
                x_sol    = [Js_S(idxn), IEA_S(idxn)];
                exitflag = 0;
            end
        end

        exitflag_log(RPM_idx, T_idx) = exitflag;
        if isempty(x_sol)
            continue;
        end

        Js_optimal(RPM_idx, T_idx)          = x_sol(1);
        IEA_optimal(RPM_idx, T_idx)         = x_sol(2);
        Is_optimal(RPM_idx, T_idx)          = Awire * x_sol(1) * 10^6;
        Prox_loss_opt(RPM_idx, T_idx)       = F_Prox{RPM_idx}(x_sol(1), x_sol(2));
        Hyst_loss_opt(RPM_idx, T_idx)       = F_Hyst{RPM_idx}(x_sol(1), x_sol(2));
        Eddy_loss_opt(RPM_idx, T_idx)       = F_Eddy{RPM_idx}(x_sol(1), x_sol(2));
        Magnet_loss_opt(RPM_idx, T_idx)     = F_Magnet{RPM_idx}(x_sol(1), x_sol(2));
        PhaseOhmic_loss_opt(RPM_idx, T_idx) = F_Ohmic{RPM_idx}(x_sol(1), x_sol(2));
        Total_loss_opt(RPM_idx, T_idx)      = F_Total{RPM_idx}(x_sol(1), x_sol(2));
        Efficiency_opt(RPM_idx, T_idx)      = F_Eff{RPM_idx}(x_sol(1), x_sol(2));
        Voltage_opt(RPM_idx, T_idx)         = F_V{RPM_idx}(x_sol(1), x_sol(2));
        Vinverter_opt(RPM_idx, T_idx)       = Voltage_opt(RPM_idx, T_idx);
        Iinverter_opt(RPM_idx, T_idx)       = F_Iinv{RPM_idx}(x_sol(1), x_sol(2));
        Pm_opt(RPM_idx, T_idx)              = F_Pm{RPM_idx}(x_sol(1), x_sol(2));
        Pin_opt(RPM_idx, T_idx)             = F_Pin{RPM_idx}(x_sol(1), x_sol(2));
    end
end

elapsed = toc;
fprintf('\nOptimization complete! Time: %.1f seconds\n', elapsed);

reachable          = exitflag_log ~= -98;
num_reachable      = sum(reachable(:));
num_fmincon        = sum(exitflag_log(:) > 0);
num_fallback       = sum(exitflag_log(:) == 0);
num_failed         = sum(exitflag_log(:) == -99);
final_success_rate = 100 * (num_fmincon + num_fallback) / max(num_reachable, 1);

fprintf('Reachable points (below envelope): %d / %d\n', num_reachable, numel(exitflag_log));
fprintf('  fmincon: %d | grid fallback: %d | failed: %d\n', num_fmincon, num_fallback, num_failed);
fprintf('Coverage of reachable region: %.1f%%\n\n', final_success_rate);

infeasible_RPM = isnan(T_env);

%% ========================================================================
%% UPSAMPLE FOR SMOOTH RENDERING
%% ========================================================================

RPM_fine    = linspace(min(RPM_opt),    max(RPM_opt),    400);
Torque_fine = linspace(min(Torque_opt), max(Torque_opt), 400);
[RPM_fine_mesh, T_fine_mesh] = meshgrid(RPM_fine, Torque_fine);
[RPM_mesh, T_mesh]           = meshgrid(RPM_opt,  Torque_opt);

% Crisp envelope mask: everything above T_env(RPM) is empty
T_env_fine    = interp1(RPM_opt, T_env, RPM_fine, 'linear');
above_env     = T_fine_mesh > T_env_fine;

% Before upsampling, each RPM row is extended past the envelope with its
% last valid value (so interp2 does not eat a band below the envelope);
% the extension is then removed again by the crisp envelope mask.
to_fine = @(M) upsample_masked(M, RPM_mesh, T_mesh, RPM_fine_mesh, T_fine_mesh, above_env);

Prox_fine   = to_fine(Prox_loss_opt);
Hyst_fine   = to_fine(Hyst_loss_opt);
Eddy_fine   = to_fine(Eddy_loss_opt);
Magnet_fine = to_fine(Magnet_loss_opt);
Ohmic_fine  = to_fine(PhaseOhmic_loss_opt);
Total_fine  = to_fine(Total_loss_opt);
Eff_fine    = to_fine(Efficiency_opt*100);
Pm_fine     = to_fine(Pm_opt);
Pin_fine    = to_fine(Pin_opt);
IEA_fine    = to_fine(IEA_optimal);
V_fine      = to_fine(Vinverter_opt);

% Calculate Core losses (Hysteresis + Eddy)
Core_fine = Hyst_fine + Eddy_fine;

T_env_plot = min(T_env, T_max_opt);
T_env_plot(isnan(T_env)) = NaN;   % min() ignores NaN -> keep unreachable speeds empty

% Crop the speed axis just past the maximum reachable speed
last_feasible = find(~isnan(T_env), 1, 'last');
RPM_max_reach = RPM_opt(last_feasible);
RPM_xlim      = [RPM_opt(1), min(RPM_opt(end), RPM_max_reach + 0.05*(RPM_max_reach - RPM_opt(1)))];
torque_range_str = sprintf('%.1f-%.0f Nm', T_min_opt, T_max_opt);

%% ========================================================================
%% FIGURE 1: COMPONENT LOSSES (2x2)
%% ========================================================================

fprintf('Generating smooth loss heatmaps (%s)...\n', torque_range_str);

figure('Position', [100, 50, 1400, 900], 'Color', 'w');

loss_panels = {
    Core_fine,   'Total Core Losses',  'Core Losses [W]';
    Ohmic_fine,  'Copper Losses',      'Copper Losses [W]';
    Magnet_fine, 'PM Losses',          'PM Losses [W]';
    Prox_fine,   'Proximity Losses',   'Proximity Losses [W]';
};

for p = 1:4
    subplot(2, 2, p);
    data = loss_panels{p, 1};

    h = pcolor(RPM_fine_mesh, T_fine_mesh, data);
    shading interp;
    colormap(gca, turbo);

    cb = colorbar;
    ylabel(cb, loss_panels{p, 3}, 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'k');
    set(cb, 'Color', 'k');

    hold on;
    C = contour(RPM_fine_mesh, T_fine_mesh, data, 10, 'LineColor', 'k', 'LineWidth', 1.0);
    clabel(C, 'FontSize', 8, 'FontWeight', 'bold', 'Color', 'k');
    draw_envelope(RPM_opt, T_env_plot, base_RPM, RPM_xlim);
    hold off;

    xlabel('RPM',         'FontSize', 11, 'FontWeight', 'bold', 'Color', 'k');
    ylabel('Torque [Nm]', 'FontSize', 11, 'FontWeight', 'bold', 'Color', 'k');
    title(loss_panels{p, 2}, 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k');
    set(gca, 'FontSize', 10, 'XColor', 'k', 'YColor', 'k');
    grid off;
end

sgtitle('Component Loss Analysis at MTPA / Field-Weakening Optimal Operating Points', 'FontSize', 14, 'FontWeight', 'bold', 'Color', 'k');

%% ========================================================================
%% FIGURE 2: TOTAL LOSSES
%% ========================================================================

figure('Position', [150, 100, 800, 600], 'Color', 'w');

h = pcolor(RPM_fine_mesh, T_fine_mesh, Total_fine);
shading interp;
colormap(gca, turbo);

cb = colorbar;
ylabel(cb, 'Total Loss [W]', 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'k');
set(cb, 'Color', 'k');

hold on;
C = contour(RPM_fine_mesh, T_fine_mesh, Total_fine, 10, 'LineColor', 'k', 'LineWidth', 1.2);
clabel(C, 'FontSize', 9, 'FontWeight', 'bold', 'Color', 'k');
draw_envelope(RPM_opt, T_env_plot, base_RPM, RPM_xlim);
hold off;

xlabel('RPM',         'FontSize', 13, 'FontWeight', 'bold', 'Color', 'k');
ylabel('Torque [Nm]', 'FontSize', 13, 'FontWeight', 'bold', 'Color', 'k');
title(sprintf('Total Losses at MTPA / FW Optimal Operating Points (%s)', torque_range_str), ...
    'FontSize', 14, 'FontWeight', 'bold', 'Color', 'k');
set(gca, 'FontSize', 11, 'XColor', 'k', 'YColor', 'k');
grid off;

%% ========================================================================
%% FIGURE 3: POWER (Pout and Pin)
%% ========================================================================

figure('Position', [200, 150, 1400, 600], 'Color', 'w');

% OUTPUT POWER
subplot(1, 2, 1);
h = pcolor(RPM_fine_mesh, T_fine_mesh, Pm_fine);
shading interp;
colormap(gca, turbo);

cb = colorbar;
ylabel(cb, 'P_{out} [W]', 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'k');
set(cb, 'Color', 'k');

hold on;
C = contour(RPM_fine_mesh, T_fine_mesh, Pm_fine, 10, 'LineColor', 'k', 'LineWidth', 1.0);
clabel(C, 'FontSize', 8, 'FontWeight', 'bold', 'Color', 'k');
draw_envelope(RPM_opt, T_env_plot, base_RPM, RPM_xlim);
hold off;

xlabel('RPM',         'FontSize', 11, 'FontWeight', 'bold', 'Color', 'k');
ylabel('Torque [Nm]', 'FontSize', 11, 'FontWeight', 'bold', 'Color', 'k');
title('Output Power (P_{out})', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k');
set(gca, 'FontSize', 10, 'XColor', 'k', 'YColor', 'k');
grid off;

% INPUT POWER
subplot(1, 2, 2);
h = pcolor(RPM_fine_mesh, T_fine_mesh, Pin_fine);
shading interp;
colormap(gca, turbo);

cb = colorbar;
ylabel(cb, 'P_{in} [W]', 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'k');
set(cb, 'Color', 'k');

hold on;
C = contour(RPM_fine_mesh, T_fine_mesh, Pin_fine, 10, 'LineColor', 'k', 'LineWidth', 1.0);
clabel(C, 'FontSize', 8, 'FontWeight', 'bold', 'Color', 'k');
draw_envelope(RPM_opt, T_env_plot, base_RPM, RPM_xlim);
hold off;

xlabel('RPM',         'FontSize', 11, 'FontWeight', 'bold', 'Color', 'k');
ylabel('Torque [Nm]', 'FontSize', 11, 'FontWeight', 'bold', 'Color', 'k');
title('Input Power (P_{in})', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k');
set(gca, 'FontSize', 10, 'XColor', 'k', 'YColor', 'k');
grid off;

sgtitle(sprintf('Power Analysis at MTPA / FW Optimal Operating Points (%s)', torque_range_str), ...
    'FontSize', 14, 'FontWeight', 'bold', 'Color', 'k');

%% ========================================================================
%% FIGURE 4: EFFICIENCY
%% ========================================================================

figure('Position', [250, 200, 800, 600], 'Color', 'w');

h = pcolor(RPM_fine_mesh, T_fine_mesh, Eff_fine);
shading interp;
colormap(gca, turbo);

cb = colorbar;
ylabel(cb, 'Efficiency [%]', 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'k');
set(cb, 'Color', 'k');

hold on;
% More detailed contours for low efficiency regions (below 70%)
C = contour(RPM_fine_mesh, T_fine_mesh, Eff_fine, [10 20 30 40 50 60 65 70 75 80 85 90 95], ...
    'LineColor', 'k', 'LineWidth', 1.2);
clabel(C, 'FontSize', 9, 'FontWeight', 'bold', 'Color', 'k');
draw_envelope(RPM_opt, T_env_plot, base_RPM, RPM_xlim);
hold off;

xlabel('RPM',         'FontSize', 13, 'FontWeight', 'bold', 'Color', 'k');
ylabel('Torque [Nm]', 'FontSize', 13, 'FontWeight', 'bold', 'Color', 'k');
title('Efficiency at MTPA / FW Optimal Operating Points', 'FontSize', 14, 'FontWeight', 'bold', 'Color', 'k');
set(gca, 'FontSize', 11, 'XColor', 'k', 'YColor', 'k');
grid off;

%% ========================================================================
%% FIGURE 5: TORQUE-SPEED AND POWER-SPEED ENVELOPE
%% ========================================================================

figure('Position', [300, 250, 1400, 550], 'Color', 'w');

omega_opt = RPM_opt(:) * 2*pi/60;

subplot(1, 2, 1);
plot(RPM_opt, T_env, 'b-', 'LineWidth', 2);
hold on;
yline(T_max_plot, 'k:', 'Plot limit', 'LineWidth', 1.0);
if ~isnan(base_RPM)
    xline(base_RPM, 'r--', sprintf('Base speed ~%d RPM', base_RPM), 'LineWidth', 1.5);
end
hold off;
xlim(RPM_xlim);
xlabel('RPM',                'FontSize', 11, 'FontWeight', 'bold', 'Color', 'k');
ylabel('Max Torque [Nm]',    'FontSize', 11, 'FontWeight', 'bold', 'Color', 'k');
title('Maximum Torque vs Speed', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k');
set(gca, 'FontSize', 10, 'XColor', 'k', 'YColor', 'k');
grid on;

subplot(1, 2, 2);
plot(RPM_opt, T_env .* omega_opt, 'b-', 'LineWidth', 2);
hold on;
if ~isnan(base_RPM)
    xline(base_RPM, 'r--', sprintf('Base speed ~%d RPM', base_RPM), 'LineWidth', 1.5);
end
hold off;
xlim(RPM_xlim);
xlabel('RPM',                'FontSize', 11, 'FontWeight', 'bold', 'Color', 'k');
ylabel('Max Power [W]',      'FontSize', 11, 'FontWeight', 'bold', 'Color', 'k');
title('Maximum Output Power vs Speed', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k');
set(gca, 'FontSize', 10, 'XColor', 'k', 'YColor', 'k');
grid on;

sgtitle(sprintf('Torque-Speed Envelope (V \\leq %.0f V, I \\leq %.0f A)', V_limit, I_limit), ...
    'FontSize', 14, 'FontWeight', 'bold', 'Color', 'k');

%% ========================================================================
%% FIGURE 6: OPTIMAL CURRENT ANGLE AND INVERTER VOLTAGE
%% ========================================================================

figure('Position', [350, 300, 1400, 600], 'Color', 'w');

subplot(1, 2, 1);
h = pcolor(RPM_fine_mesh, T_fine_mesh, IEA_fine);
shading interp;
colormap(gca, turbo);
cb = colorbar;
ylabel(cb, 'IEA [deg]', 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'k');
set(cb, 'Color', 'k');
hold on;
C = contour(RPM_fine_mesh, T_fine_mesh, IEA_fine, 10, 'LineColor', 'k', 'LineWidth', 1.0);
clabel(C, 'FontSize', 8, 'FontWeight', 'bold', 'Color', 'k');
draw_envelope(RPM_opt, T_env_plot, base_RPM, RPM_xlim);
hold off;
xlabel('RPM',         'FontSize', 11, 'FontWeight', 'bold', 'Color', 'k');
ylabel('Torque [Nm]', 'FontSize', 11, 'FontWeight', 'bold', 'Color', 'k');
title('Optimal Current Angle (IEA > MTPA angle \rightarrow FW)', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k');
set(gca, 'FontSize', 10, 'XColor', 'k', 'YColor', 'k');
grid off;

subplot(1, 2, 2);
h = pcolor(RPM_fine_mesh, T_fine_mesh, V_fine);
shading interp;
colormap(gca, turbo);
cb = colorbar;
ylabel(cb, 'V_{inverter} [V]', 'FontSize', 10, 'FontWeight', 'bold', 'Color', 'k');
set(cb, 'Color', 'k');
hold on;
C = contour(RPM_fine_mesh, T_fine_mesh, V_fine, 10, 'LineColor', 'k', 'LineWidth', 1.0);
clabel(C, 'FontSize', 8, 'FontWeight', 'bold', 'Color', 'k');
draw_envelope(RPM_opt, T_env_plot, base_RPM, RPM_xlim);
hold off;
xlabel('RPM',         'FontSize', 11, 'FontWeight', 'bold', 'Color', 'k');
ylabel('Torque [Nm]', 'FontSize', 11, 'FontWeight', 'bold', 'Color', 'k');
title(sprintf('Inverter Voltage (limit %.0f V)', V_limit), 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k');
set(gca, 'FontSize', 10, 'XColor', 'k', 'YColor', 'k');
grid off;

sgtitle('Control Variables at MTPA / FW Optimal Operating Points', ...
    'FontSize', 14, 'FontWeight', 'bold', 'Color', 'k');

%% ========================================================================
%% SAVE RESULTS
%% ========================================================================

fprintf('Saving results...\n');

save('MTPA_optimal_losses_0.5-40Nm.mat', ...
    'RPM_opt', 'Torque_opt', ...
    'Js_optimal', 'IEA_optimal', 'Is_optimal', ...
    'PhaseOhmic_loss_opt', 'Prox_loss_opt', 'Hyst_loss_opt', 'Eddy_loss_opt', ...
    'Magnet_loss_opt', 'Total_loss_opt', ...
    'Efficiency_opt', 'Voltage_opt', ...
    'Vinverter_opt', 'Iinverter_opt', ...
    'Pm_opt', 'Pin_opt', ...
    'exitflag_log', 'infeasible_RPM', ...
    'T_env', 'x_env', 'V_env', 'base_RPM', ...
    'Js_values', 'IEA_values', ...
    'T_min_opt', 'T_max_opt', 'V_limit', 'I_limit');

fprintf('Results saved to MTPA_optimal_losses_0.5-40Nm.mat\n');

%% ========================================================================
%% SUMMARY
%% ========================================================================

fprintf('\n========================================\n');
fprintf('SUMMARY\n');
fprintf('========================================\n');
fprintf('Js range used:  %.3f to %.2f A/mm^2\n', min(Js_values), max(Js_values));
fprintf('Torque range:   %.1f to %.1f Nm\n', min(Torque_opt), max(Torque_opt));
fprintf('Speed range:    %d to %d RPM\n', RPM_opt(1), RPM_opt(end));
fprintf('Base speed:     %s\n', num2str(base_RPM));
fprintf('Max torque:     %.2f Nm (low speed)\n', T_env(find(~isnan(T_env), 1, 'first')));
fprintf('Max speed:      ~%d RPM (torque -> %.2f Nm)\n', RPM_max_reach, T_env(last_feasible));
fprintf('Coverage of reachable region: %.1f%%\n', final_success_rate);
fprintf('\nInverter Limits: V <= %.0f V, I <= %.0f A\n', V_limit, I_limit);
fprintf('Raw points violating limits: %d / %d (%.1f%%)\n', ...
    masked_pts, total_pts, 100*masked_pts/total_pts);
fprintf('RPM rows with no feasible point: %d / %d\n', ...
    sum(infeasible_RPM), num_RPM_opt);
fprintf('Vinverter range at optimal: %.2f to %.2f V\n', ...
    min(Vinverter_opt(:),[], 'omitnan'), max(Vinverter_opt(:),[], 'omitnan'));
fprintf('Iinverter range at optimal: %.2f to %.2f A\n', ...
    min(Iinverter_opt(:),[], 'omitnan'), max(Iinverter_opt(:),[], 'omitnan'));
fprintf('\nLoss Ranges:\n');
fprintf('  PhaseOhmic:  %.2f to %.2f W\n', min(PhaseOhmic_loss_opt(:),[], 'omitnan'), max(PhaseOhmic_loss_opt(:),[], 'omitnan'));
fprintf('  Proximity:   %.2f to %.2f W\n', min(Prox_loss_opt(:),[], 'omitnan'),       max(Prox_loss_opt(:),[], 'omitnan'));
fprintf('  Hysteresis:  %.2f to %.2f W\n', min(Hyst_loss_opt(:),[], 'omitnan'),       max(Hyst_loss_opt(:),[], 'omitnan'));
fprintf('  Eddy:        %.2f to %.2f W\n', min(Eddy_loss_opt(:),[], 'omitnan'),       max(Eddy_loss_opt(:),[], 'omitnan'));
fprintf('  Magnet:      %.2f to %.2f W\n', min(Magnet_loss_opt(:),[], 'omitnan'),     max(Magnet_loss_opt(:),[], 'omitnan'));
fprintf('  Total:       %.2f to %.2f W\n', min(Total_loss_opt(:),[], 'omitnan'),      max(Total_loss_opt(:),[], 'omitnan'));
fprintf('  Efficiency:  %.1f%% to %.1f%%\n', ...
    100*min(Efficiency_opt(:),[], 'omitnan'), 100*max(Efficiency_opt(:),[], 'omitnan'));
fprintf('========================================\n');
fprintf('\nProcessing complete!\n');

%% ========================================================================
%% LOCAL FUNCTIONS
%% ========================================================================

function M_fine = upsample_masked(M, RPM_mesh, T_mesh, RPM_fine_mesh, T_fine_mesh, above_env)
    % M is (num_RPM x num_T). Extend each RPM row along torque with the
    % nearest valid value so linear interpolation keeps its values right up
    % to the envelope, then cut everything above the envelope.
    M_ext  = fillmissing(M, 'nearest', 2);
    M_fine = interp2(RPM_mesh, T_mesh, M_ext', RPM_fine_mesh, T_fine_mesh, 'linear', NaN);
    M_fine(above_env) = NaN;
end

function draw_envelope(RPM_opt, T_env_plot, base_RPM, RPM_xlim)
    % Max torque envelope (MTPA + field weakening) and base speed marker
    plot(RPM_opt, T_env_plot, 'k-', 'LineWidth', 2.5);
    if ~isnan(base_RPM)
        xline(base_RPM, 'w--', 'LineWidth', 1.5);
    end
    xlim(RPM_xlim);
end
