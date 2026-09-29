function [Motor] = CreateWinding(Motor)
    % Design of the stator of an SPM
    % 11/2025 F.G.Kyrgios
    
    Poles = Motor.Configuration.Basic.Poles;
    Slots = Motor.Configuration.Basic.Slots;
    layers = Motor.Configuration.Basic.Layers;
    shift = Motor.Configuration.Basic.Shift;
    bars = Motor.Configuration.Basic.Bars;
    numofbars = Motor.Configuration.Basic.NumOfBars;
    CoilTurns = Motor.Configuration.Stator.CoilTurns;

    % SPP
    q = Motor.Configuration.Basic.q;
    
    % Periodicity
    t = Motor.Configuration.Basic.t;

    % Rotor
    Rri = Motor.Configuration.Rotor.Rri; % Internal radius of rotor
    Rro = Motor.Configuration.Rotor.Rro; % Outer radius of rotor
    
    % Magnet
    Hmag = Motor.Configuration.Magnet.Hmag; % Magnet height
    theta_mag = Motor.Configuration.Magnet.theta_mag; % Magnet range
    Rmag = Rro + Hmag; % Outer radius of magnet
    Wavmag = (360/Poles); % Angle in degrees occupied by each pole
    Wmag = theta_mag * Wavmag; % Angle in degrees occupied by each magnet
    
    % Air-Gap
    l_g = Motor.Configuration.Basic.AirGap;
    
    % Stator
    WbiRatio = Motor.Configuration.Stator.WbiRatio; % Ratio of stator yoke to tooth
    Rso = Motor.Configuration.Stator.Rso; % Outer radius of stator
    
    WtRatio = Motor.Configuration.Stator.WtRatio;
    beta_t =Motor.Configuration.Stator.beta_t;
    
    Wavtip = (360/Slots); % Angle in degrees occupied by each tooth/slot
    
    theta_tip = Motor.Configuration.Stator.theta_tip; % Range of tooth pad
    Wtip = theta_tip * Wavtip; % Angle in degreed occupied by each tooth pad
    
    Wt = WtRatio * Wtip; % Angle in degreed occupied by each tooth body
    
    Wbi = Wt * WbiRatio; % Width of stator yoke

    b_so = (1 - theta_tip) * (Rmag + l_g) * Wavtip; % Length of slot entrance
    
    h_tp = Motor.Configuration.Stator.h_tp; % Tooth pad width
    L_t = Rso - Wbi - l_g - Rmag; % Tooth length
    
    % Central point %
    x0 = 0;
    y0 = 0;

    % Σημείο 1
    x1 = Rmag + l_g;
    y1 = 0;

    % Σημείο 6
    [x6, y6] = rotatePoint(x1 + L_t, 0, x0, y0, x1 + L_t, Wavtip * (pi/180) / 2);
    
    % Σημείο 7
    [x7, y7] = rotatePoint(x1 + L_t + Wbi, 0, x0, y0, x1 + L_t + Wbi, Wavtip * (pi/180) / 2);
 
    % Σημείο 9.1 ανάμεσα σε 4 και 5 (μπαίνει πάντα)
    [x901, y901] = rotatePoint(x1 + (L_t + h_tp + beta_t)/2, 0, x0, y0, x1 + (L_t + h_tp + beta_t)/2, Wt * (pi/180) / 2);

    if bars == 0
        % Σημείο 4
        [x4, y4] = rotatePoint(x1 + h_tp + beta_t, 0, x0, y0, x1 + h_tp + beta_t, Wt * (pi/180) / 2);
        % Σημείο 5
        [x5, y5] = rotatePoint(x1 + L_t, 0, x0, y0, x1 + L_t, Wt * (pi/180) / 2);
     elseif bars == 1 || bars == 2
        R = (L_t - h_tp - beta_t)/2;
        [x5, y5, x4, y4] = findParallelLinePoints(x6, y6, x7, y7, x901, y901, R);
    end

    sequence = Motor.Configuration.Winding.Sequence;
    [List, OppositeList] = saveLetter(sequence, shift);

    function turns = getTurnsFromBlockName(blockName)
        if isstrprop(blockName, 'upper')
        turns = CoilTurns;
        else
        turns = -CoilTurns;
        end
    end

    function circuitName = getCircuitFromBlockName(blockName)
        circuitName = upper(blockName);
    end

    
    if q < 1 && layers == 2
        % Κανονικά το sequence
        for i = 1:length(List)/t
            blockNameNor = List{i}; % Extract το γράμμα από την αλληλουχία (Normal Sequence)
            blockNameSup = OppositeList{i}; % Εxtract το γράμμα από την ανάποδη αλληλουχία (Supplementary Sequence)
            
            % Συντεταγμένες των σημείων δεξιού και αριστερού μέρους του Slot αντίστοιχα
            [x_Nor, y_Nor] = rotatePoint(Rmag + l_g + (L_t + h_tp + beta_t)/2,  0, x0, y0, Rmag + l_g + (L_t + h_tp + beta_t)/2, (Wt + (Wavtip - Wt) / 2) * (pi/180) / 2 + Wavtip * (i-1) * (pi/180));
            [x_Sup, y_Sup] = rotatePoint(Rmag + l_g + (L_t + h_tp + beta_t)/2,  0, x0, y0, Rmag + l_g + (L_t + h_tp + beta_t)/2, -(Wt + (Wavtip - Wt) / 2) * (pi/180) / 2 + Wavtip * i * (pi/180));
        
            % Κάνω assign για το κανονικό sequence (Δεξί μέρος)
            createBlocklabel(x_Nor, y_Nor, 'Phase', 0, Motor.Solver.Mesh.Stator.Winding, getCircuitFromBlockName(blockNameNor), 0 , Motor.Solver.Groups.Stator.Winding, getTurnsFromBlockName(blockNameNor));
            
            % Κάνω assign για το συμπληρωματικό sequence (Αριστερό μέρος)
            createBlocklabel(x_Sup, y_Sup, 'Phase', 0, Motor.Solver.Mesh.Stator.Winding, getCircuitFromBlockName(blockNameSup), 0 , Motor.Solver.Groups.Stator.Winding, getTurnsFromBlockName(blockNameSup));
        end
    elseif q < 1 && layers == 1
        for i = 1:length(List)/t
            blockNameNor = List{i}; % Extract το γράμμα από την αλληλουχία (Normal Sequence)
            blockNameSup = OppositeList{i}; % Εxtract το γράμμα από την ανάποδη αλληλουχία (Supplementary Sequence)
    
            % Συντεταγμένες των σημείων δεξιού και αριστερού μέρους του Slot αντίστοιχα
            [x_Nor, y_Nor] = rotatePoint(Rmag + l_g + (L_t + h_tp + beta_t)/2,  0, x0, y0, Rmag + l_g + (L_t + h_tp + beta_t)/2, (Wavtip / 2) * (pi/180) + 2 * Wavtip * (i-1) * (pi/180));
            [x_Sup, y_Sup] = rotatePoint(Rmag + l_g + (L_t + h_tp + beta_t)/2,  0, x0, y0, Rmag + l_g + (L_t + h_tp + beta_t)/2, -(Wavtip / 2) * (pi/180) + 2 * Wavtip * i * (pi/180));
       
            % Κάνω assign για το κανονικό sequence (Δεξί μέρος)
            createBlocklabel(x_Nor, y_Nor, 'Phase', 0, Motor.Solver.Mesh.Stator.Winding, getCircuitFromBlockName(blockNameNor), 0 , Motor.Solver.Groups.Stator.Winding, getTurnsFromBlockName(blockNameNor));
            
            % Κάνω assign για το συμπληρωματικό sequence (Αριστερό μέρος)
            createBlocklabel(x_Sup, y_Sup, 'Phase', 0, Motor.Solver.Mesh.Stator.Winding, getCircuitFromBlockName(blockNameSup), 0 , Motor.Solver.Groups.Stator.Winding, getTurnsFromBlockName(blockNameSup));
        end
    elseif q >= 1 && layers == 1
        if bars == 0
            for i = 1:length(List)/t
                blockNameNor = List{i}; % Extract το γράμμα από την αλληλουχία (Normal Sequence)
        
                % Συντεταγμένες των σημείων του Slot
                [x_Nor, y_Nor] = rotatePoint(Rmag + l_g + (L_t + h_tp + beta_t)/2,  0, x0, y0, Rmag + l_g + (L_t + h_tp + beta_t)/2, (Wavtip / 2) * (pi/180) + Wavtip * (i-1) * (pi/180));
        
                % Κάνω assign για το sequence
                createBlocklabel(x_Nor, y_Nor, 'Phase', 0, Motor.Solver.Mesh.Stator.Winding, getCircuitFromBlockName(blockNameNor), 0 , Motor.Solver.Groups.Stator.Winding, getTurnsFromBlockName(blockNameNor));
            end
        elseif bars == 1 || bars == 2
            for i = 1:length(List)/t
                blockNameNor = List{i};
                addBlockLabelsBetweenPoints(x5, y5, x4, y4, x6, y6, x7, y7, numofbars, Motor.Solver.Mesh.Stator.Winding, getCircuitFromBlockName(blockNameNor), Motor.Solver.Groups.Stator.Winding, getTurnsFromBlockName(blockNameNor), Wavtip * (i-1) * (pi/180));
            end
        end
    elseif q >=1 && layers == 2
        for i = 1:length(List)/t
            blockNameUpper = List{i}; % Extract το γράμμα από την πάνω αλληλουχία
            blockNameLower = OppositeList{i}; % Extract το γράμμα από την κάτω αλληλουχία
    
            % Συντεταγμένες των σημείων δεξιού και αριστερού μέρους του Slot αντίστοιχα
            [x_Up, y_Up] = rotatePoint(Rmag + l_g + (3*L_t + h_tp + beta_t)/4,  0, x0, y0, Rmag + l_g + (3*L_t + h_tp + beta_t)/4, (Wavtip / 2) * (pi/180) + Wavtip * (i-1) * (pi/180));
            [x_Lo, y_Lo] = rotatePoint(Rmag + l_g + (L_t + 3*h_tp + 3*beta_t)/4,  0, x0, y0, Rmag + l_g + (L_t + 3*h_tp + 3*beta_t)/4, -(Wavtip / 2) * (pi/180) + Wavtip * i * (pi/180));
    
            createBlocklabel(x_Up, y_Up, 'Phase', 0, Motor.Solver.Mesh.Stator.Winding, getCircuitFromBlockName(blockNameUpper), 0, Motor.Solver.Groups.Stator.Winding, getTurnsFromBlockName(blockNameUpper));
            createBlocklabel(x_Lo, y_Lo, 'Phase', 0, Motor.Solver.Mesh.Stator.Winding, getCircuitFromBlockName(blockNameLower), 0, Motor.Solver.Groups.Stator.Winding, getTurnsFromBlockName(blockNameLower));
        end
    end
end