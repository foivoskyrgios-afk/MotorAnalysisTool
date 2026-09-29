function [Motor] = DrawStatorGeometry(Motor)
    % Design of the stator of an SPM
    % 11/2025 F.G.Kyrgios
    
    Poles = Motor.Configuration.Basic.Poles;
    Slots = Motor.Configuration.Basic.Slots;
    layers = Motor.Configuration.Basic.Layers;
    bars = Motor.Configuration.Basic.Bars;
    numofbars = Motor.Configuration.Basic.NumOfBars;
    Lx = Motor.Configuration.Basic.Lx;
    Ly = Motor.Configuration.Basic.Ly;
    radius = Motor.Configuration.Basic.radius;
    
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

    Wbi = Wt * WbiRatio;
    
    b_so = (1 - theta_tip) * (Rmag + l_g) * Wavtip; % Length of slot entrance
    
    h_tp = Motor.Configuration.Stator.h_tp; % Tooth pad width
    L_t = Rso - Wbi - l_g - Rmag; % Tooth length
    
    % Central point %
    x0 = 0;
    y0 = 0;
    if t == 1
        createBlocklabel(x0 ,y0, 'Air', 0, Motor.Solver.Mesh.Air, 0, 0, Motor.Solver.Groups.Air, 0); % Centre
    end
    
    % Κατασκευή της γεωμετρίας του στάτη (κβάντο στάτη και mirroring/rotation) %
    
    % To κβάντο του στάτη αποτελείται από 8 σημεία
    
    % Σημείο 1
    x1 = Rmag + l_g;
    y1 = 0;
    createPoint(x1, y1, 0, Motor.Solver.Groups.Stator.Geometry);
    
    % Σημείο 2
    [x2, y2] = rotatePoint(x1, 0, x0, y0, x1, Wtip * (pi/180) / 2);
    createPoint(x2, y2, 0, Motor.Solver.Groups.Stator.Geometry);
    
    % Σημείο 3
    [x3, y3] = rotatePoint(x1 + h_tp, 0, x0 , y0, x1 + h_tp, Wtip * (pi/180) /2);
    createPoint(x3, y3, 0, Motor.Solver.Groups.Stator.Geometry);
    
    % % Σημείο 4
    % [x4, y4] = rotatePoint(x1 + h_tp + beta_t, 0, x0, y0, x1 + h_tp + beta_t, Wt * (pi/180) / 2);
    % createPoint(x4, y4, 0, Motor.Solver.Groups.Stator.Geometry);
    % 
    % % Σημείο 5
    % [x5, y5] = rotatePoint(x1 + L_t, 0, x0, y0, x1 + L_t, Wt * (pi/180) / 2);
    % createPoint(x5, y5, 0, Motor.Solver.Groups.Stator.Geometry);
    
    % Σημείο 6
    [x6, y6] = rotatePoint(x1 + L_t, 0, x0, y0, x1 + L_t, Wavtip * (pi/180) / 2);
    createPoint(x6, y6, 0, Motor.Solver.Groups.Stator.Geometry);
    
    % Σημείο 7
    [x7, y7] = rotatePoint(x1 + L_t + Wbi, 0, x0, y0, x1 + L_t + Wbi, Wavtip * (pi/180) / 2);
    createPoint(x7, y7, 0, Motor.Solver.Groups.Stator.Geometry);
    
    % Σημείο 8
    x8 = x1 + L_t + Wbi;
    y8 = 0;
    createPoint(x8, y8, 0, Motor.Solver.Groups.Stator.Geometry);
    
    % Σημείο 9 (EXTRA: Για τον χωρισμό του αυλακιού στα δυο)
    if bars == 0
        [x9 ,y9] = rotatePoint(x1 + h_tp, 0, x0, y0, x1 + h_tp, Wavtip * (pi/180) / 2);
        createPoint(x9, y9, 0, Motor.Solver.Groups.Stator.Geometry);
    end
    % Extra σημεία για double layer distributed winding
    % Στο single layer distributed και το concentrated αρκεί το σημείο 9
    
    % Σημείο 9.1 ανάμεσα σε 4 και 5 (μπαίνει πάντα)
    [x901, y901] = rotatePoint(x1 + (L_t + h_tp + beta_t)/2, 0, x0, y0, x1 + (L_t + h_tp + beta_t)/2, Wt * (pi/180) / 2);
    if bars == 0
        createPoint(x901, y901, 0, Motor.Solver.Groups.Stator.Geometry);
    end

    if bars == 0
        % Σημείο 4
        [x4, y4] = rotatePoint(x1 + h_tp + beta_t, 0, x0, y0, x1 + h_tp + beta_t, Wt * (pi/180) / 2);
        createPoint(x4, y4, 0, Motor.Solver.Groups.Stator.Geometry);
        
        % Σημείο 5
        [x5, y5] = rotatePoint(x1 + L_t, 0, x0, y0, x1 + L_t, Wt * (pi/180) / 2);
        createPoint(x5, y5, 0, Motor.Solver.Groups.Stator.Geometry);
    elseif bars == 1 || bars == 2
        R = (L_t - h_tp - beta_t)/2;
        [x5, y5, x4, y4] = findParallelLinePoints(x6, y6, x7, y7, x901, y901, R);
        createPoint(x4, y4, 0, Motor.Solver.Groups.Stator.Geometry);
        createPoint(x5, y5, 0, Motor.Solver.Groups.Stator.Geometry);
    end
    
    % Σημείο 9.2
    if q >= 1 && layers == 2
        [x902, y902] = rotatePoint(x1 + (L_t + h_tp + beta_t)/2, 0, x0, y0, x1 + (L_t + h_tp + beta_t)/2, Wavtip * (pi/180) / 2);
        createPoint(x902, y902, 0, Motor.Solver.Groups.Stator.Geometry);
    end
    
    % Σημείο 10 (EXTRA: Για τον καθορισμό του σιδήρου στον στάτη)
    [x11, y11] = rotatePoint(x1 + (L_t + h_tp + beta_t)/2, 0, x0, y0, x1 + (L_t + h_tp + beta_t)/2, Wt/4 * (pi/180));
    createBlocklabel(x11, y11, 'Iron', 0, Motor.Solver.Mesh.Stator.Iron, 0, 0, Motor.Solver.Groups.Stator.Iron, 0);
    
    % Τμήματα %
    
    % Τόξο 1 - 2
    createArc(x1, y1, x2, y2, Wtip / 2, 5, 5, 'None', Motor.Solver.Groups.Stator.Geometry);
    
    % Τμήμα 2 - 3
    createSegment(x2, y2, x3, y3, 'None', Motor.Solver.Mesh.Stator.Iron, 0, 0, Motor.Solver.Groups.Stator.Geometry);
    
    % Τμήμα 3 - 4
    createSegment(x3, y3, x4, y4, 'None', Motor.Solver.Mesh.Stator.Iron, 0, 0, Motor.Solver.Groups.Stator.Geometry);
    
    % Τμήμα 4 - 9.1
    createSegment(x4, y4, x901, y901, 'None', Motor.Solver.Mesh.Stator.Iron, 0, 0, Motor.Solver.Groups.Stator.Geometry);
    
    % Τμήμα 9.1 - 5
    createSegment(x901, y901, x5, y5, 'None', Motor.Solver.Mesh.Stator.Iron, 0, 0, Motor.Solver.Groups.Stator.Geometry);
    
    % Τόξο 5 - 6
    createArc(x5, y5, x6, y6, (Wavtip - Wt) / 2, 5, 5, 'None', Motor.Solver.Groups.Stator.Geometry);
    
    % Τόξο 7 - 8
    createArc(x8, y8, x7, y7, Wavtip / 2, 5, 5, 'A=0', Motor.Solver.Groups.Stator.Geometry);
    
    % Τμήμα 8 - 1 (ΠΡΟΣΟΧΗ! Αυτό το τμήμα δεν κατασκευάζεται στο σχέδιο (για την πλήρη μηχανή))
    if t ~= 1
        createSegment(x8, y8, x1, y1, 'apbc4', Motor.Solver.Mesh.Stator.Iron, 0, 0, 0);
    end
    
    % Τμήμα 9 - 6 (EXTRA: Για τον χωρισμό του αυλακιού στα δυο)
    if q < 1 && layers == 2
        createSegment(x9, y9, x6, y6, 'None', Motor.Solver.Mesh.Stator.Iron, 0, 0, Motor.Solver.Groups.Stator.Geometry);
    elseif q >= 1 && layers == 2
        createSegment(x901, y901, x902, y902, 'None', Motor.Solver.Mesh.Stator.Iron, 0, 0, Motor.Solver.Groups.Stator.Geometry);
    end
    
    
    % Τμήμα 3 - 9 (EXTRA: Για το "κλείσιμο" του αυλακιού)
    if bars == 0
        createSegment(x3, y3, x9, y9, 'None', Motor.Solver.Mesh.Stator.Iron, 0, 0, Motor.Solver.Groups.Stator.Geometry);
    end

    
    mi_createradius(x5, y5, pi/16); % Δημιουργώ καμπύλη στις γωνίες κάθε αυλακιού
    
    % Παράλληλα τμήματα για την περίπτωση των μπαρών
    if bars == 1
        createPerpendicularSegments(x4, y4, x5, y5, x6, y6, x7, y7, numofbars, 0.1, Motor.Solver.Groups.Stator.Geometry, Motor.Solver.Mesh.Stator.Iron);
    end
    
    if bars == 2
        createSpecificSizeBlocks(x4, y4, x5, y5, x6, y6, x7, y7, Lx, Ly/2, numofbars, Motor.Solver.Groups.Stator.Geometry, Motor.Solver.Mesh.Stator.Iron, radius);
    end
    %{
    % Δοκιμή σημείων επιλογής των Τμημάτων και των Τόξων
    mi_addnode(x1_2, y1_2);
    mi_addnode(x2_3, y2_3);
    mi_addnode(x3_4, y3_4);
    mi_addnode(x4_5, y4_5);
    mi_addnode(x5_6, y5_6);
    mi_addnode(x6_7, y6_7);
    mi_addnode(x7_8, y7_8);
    mi_addnode(x8_1, y8_1);
    %}
    
    % Κατασκευή του στάτη με mirroring και rotation
    
    mi_selectgroup(Motor.Solver.Groups.Stator.Geometry); % Επιλογή του κβάντου της μηχανής
    mi_mirror(x6, y6, x7, y7);
    
    for i = 1:(Slots/t-1)
        mi_selectgroup(Motor.Solver.Groups.Stator.Geometry);
        mi_copyrotate(x0, y0, Wavtip, 1);
        mi_clearselected();
    end
    
    % Κατασεκυή border στην άλλη μεριά του στάτη για το κβάντο της μηχανής
    % που κατασκευάζεται για σύγχρονη περιστροφή
    if t ~= 1
        [x1_rotated, y1_rotated] = rotatePoint(x1, y1, x0, y0, x1, Wavtip * Slots/t * (pi/180));
        [x8_rotated, y8_rotated] = rotatePoint(x8, y8, x0, y0, x8, Wavtip * Slots/t * (pi/180));
        createSegment(x8_rotated, y8_rotated, x1_rotated, y1_rotated, 'apbc4', Motor.Solver.Mesh.Stator.Iron, 0, 0, 0);
    end
end