function [Motor] = DrawRotorGeometry(Motor)
    % Design of the rotor of an SPM
    % 01/2026 F.G.Kyrgios
    
    Poles = Motor.Configuration.Basic.Poles;
    Slots = Motor.Configuration.Basic.Slots;
    
    % SPP
    q = Motor.Configuration.Basic.q;

    %
    t = Motor.Configuration.Basic.t;
    
    % Rotor
    Rri = Motor.Configuration.Rotor.Rri; % Internal radius of rotor
    Rro = Motor.Configuration.Rotor.Rro; % Outer radius of rotor
    
    % Magnet
    Hmag = Motor.Configuration.Magnet.Hmag; % Magnet height
    theta_mag = Motor.Configuration.Magnet.theta_mag; % Magnet range
    Nodes = Motor.Configuration.Magnet.Nodes; % Number of nodes in the outer side of the magnet
    Hmage = Motor.Configuration.Magnet.Hmage; % Magnet height at the edge
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
    
    h_tp = 1; % Tooth pad width
    L_t = Rso - Wbi - l_g - Rmag; % Tooth length
    
    % Central point %
    x0 = 0;
    y0 = 0;
    
    %% Magnets
    
    % Σημεία κέντρου εσωτερικού τόξου
    x_magcenti = Rro;
    y_magcenti = 0;
    
    % Σημεία κέντρου εξωτερικού τόξου
    x_magcento = Rro + Hmage;
    y_magcento = 0;
    
    % Σημείο 1
    [x_mag1, y_mag1] = rotatePoint(x_magcenti, y_magcenti, x0, y0, x_magcenti, (-Wmag) * (pi/180) / 2);
    createPoint(x_mag1, y_mag1, 0, Motor.Solver.Groups.Magnet.Geometry);
    
    % Σημείο 2
    [x_mag2, y_mag2] = rotatePoint(x_magcenti, y_magcenti, x0, y0, x_magcenti, Wmag * (pi/180) / 2);
    createPoint(x_mag2, y_mag2, 0, Motor.Solver.Groups.Magnet.Geometry);
    
    % Σημείο 3
    [x_mag3, y_mag3] = rotatePoint(x_magcento, y_magcento, x0, y0, x_magcento, (-Wmag) * (pi/180) / 2);
    % createPoint(x_mag3, y_mag3, 0, Motor.Solver.Groups.Magnet.Geometry);
    
    % Σημείο 4
    [x_mag4, y_mag4] = rotatePoint(x_magcento, y_magcento, x0, y0, x_magcento, Wmag * (pi/180) / 2);
    % createPoint(x_mag4, y_mag4, 0, Motor.Solver.Groups.Magnet.Geometry);

    % Κατασκευή σημείων στο εξωτερικό τμήμα του μαγνήτη
    pop = linspace(y_mag3, y_mag4, Nodes); % Θυμίσου το 40 να το κάνεις nodes στο MSV
    
    % Preallocate arrays
    xarc = zeros(1, Nodes);
    yarc = zeros(1,Nodes);

    % Δημιουργία κόμβων
    for i = 1:Nodes
        yarc(i) = pop(i);
        xarc(i) = x_mag3 + (Hmag+Rro-x_mag3) * (1 - yarc(i)^2 / y_mag3^2);
        createPoint(xarc(i), yarc(i), 0, Motor.Solver.Groups.Magnet.Geometry);
    end

    % Ενωση των σημείων
    for i = 1:Nodes-1
        createSegment(xarc(i), yarc(i), xarc(i+1), yarc(i+1), 'None', Motor.Solver.Mesh.Magnet, 0, 0, Motor.Solver.Groups.Magnet.Geometry);  % Fixed: connect to next node
    end
    
    % Σημείο 5 (EXTRA:Για τον καθορισμό των σημείων για ορισμό blocks)
    x_mag5 = Rro + Hmag/2;
    y_mag5 = 0;
    
    % Τόξο 1 - 2 
    % Το τόξο 1 - 2 δεν κατασκευάζεται καθώς τον ρόλο αυτού του τόξου τον
    % καλύπτει ο εξωτερικός δακτύλιος του δρομέα
    
    % Τμήμα 1 - 3
    createSegment(x_mag1, y_mag1, x_mag3, y_mag3, 'None', Motor.Solver.Mesh.Magnet, 0, 0, Motor.Solver.Groups.Magnet.Geometry);
    
    % Τόξο 3 - 4 
    % createArc(x_mag3, y_mag3, x_mag4, y_mag4, Wmag, 5, 5, 'None', Motor.Solver.Groups.Magnet.Geometry);
    
    % Τμήμα 2 - 4
    createSegment(x_mag2, y_mag2, x_mag4, y_mag4, 'None', Motor.Solver.Mesh.Magnet, 0, 0, Motor.Solver.Groups.Magnet.Geometry);
    
    % Κατασκευή του μαγνήτη με rotation
    
    for i = 1:(Poles/t-1)
        mi_selectgroup(Motor.Solver.Groups.Magnet.Geometry);
        mi_copyrotate(x0, y0, Wavmag, 1);
        mi_clearselected();
    end
    
    
    % Assignment των πολικοτήτων στους μαγνήτες
    
    for i = 1:Poles/t
        % Συντεταγμένες των blocks των μαγνητών
        [x_magblock, y_magblock] = rotatePoint(x_mag5, y_mag5, x0, y0, x_mag5, (i-1) * Wavmag * (pi/180));
        % Assign της πολικότητας στον μαγνήτη
        magdirection = Wavmag * (i-1) - (mod(i,2) == 0) * 180; % Εναλλαγή μεταξύ ορθών και ανάποδων πόλων
        createBlocklabel(x_magblock, y_magblock, 'Magnet', 0, Motor.Solver.Mesh.Magnet, 0, magdirection, Motor.Solver.Groups.Magnet.Magnet+(i-1), 0);
    end
    
    angrotrot = (mod(1,t) ~= 0)*Wavmag/2; % Γωνία περιστροφής μαγνητών (Angle Rotor Rotation)
    % Επιλογή μαγνητών
    mi_selectgroup(Motor.Solver.Groups.Magnet.Geometry);
    for i = 1:Poles/t
        mi_selectgroup(Motor.Solver.Groups.Magnet.Magnet+(i-1));
    end
    mi_moverotate(0, 0, angrotrot);
    mi_clearselected();
    
    %% Rotor
    
    % Σημείο Α για τον εσωτερικό κύκλο
    x_riA = Rri;
    y_riA = 0;
    createPoint(x_riA, y_riA, 0, Motor.Solver.Groups.Rotor.Geometry);
    
    % Σημείο Β για τον εσωτερικό κύκλο
    [x_riB, y_riB] = rotatePoint(Rri, 0, x0, y0, Rri, 360/(((mod(1,t)==0)+1)*t) * (pi/180));
    createPoint(x_riB, y_riB, 0, Motor.Solver.Groups.Rotor.Geometry);
    
    % Σημείο Α για τον εξωτερικό κύκλο
    x_roA = Rro;
    y_roA = 0;
    createPoint(x_roA, y_roA, 0, Motor.Solver.Groups.Rotor.Geometry);
    
    % Σημείο Β για τον εξωτερικό κύκλο
    [x_roB, y_roB] = rotatePoint(Rro, 0, x0, y0, Rro, 360/(((mod(1,t)==0)+1)*t) * (pi/180));
    createPoint(x_roB, y_roB, 0, Motor.Solver.Groups.Rotor.Geometry);
    
    % Σημείο Γ (EXTRA: Για τον καθορισμό του σιδήρου στον δρομέα)
    [x_G, y_G] = rotatePoint(Rri + (Rro-Rri)/2, 0, x0, y0, Rri + (Rro-Rri)/2, 360/(2*t) * (pi/180));
    createBlocklabel(x_G, y_G, 'Iron', 0, Motor.Solver.Mesh.Rotor.Iron, 0, 0, Motor.Solver.Groups.Rotor.Iron, 0);
    
    % Κατασκευή τόξων
    
    % Κατασκευή τόξων για το πάνω τόξο
    createArc(x_riA, y_riA, x_riB, y_riB, 360/(((mod(1,t)==0)+1)*t), 5, 5, 'A=0', Motor.Solver.Groups.Rotor.Geometry); % Πάνω ημικύκλιο
    createArc(x_roA, y_roA, x_roB, y_roB, 360/(((mod(1,t)==0)+1)*t), 5, 5, 'None', Motor.Solver.Groups.Rotor.Geometry); % Πάνω ημικύκλιο
    
    % Κατασκευή τόξων για το κάτω τόξο
    if t == 1
        createArc(x_roB, y_roB, x_roA, y_roA, 360/(((mod(1,t)==0)+1)*t), 5, 5, 'None', Motor.Solver.Groups.Rotor.Geometry); % Κάτω ημικύκλιο
        createArc(x_riB, y_riB, x_riA, y_riA, 360/(((mod(1,t)==0)+1)*t), 5, 5, 'A=0',Motor.Solver.Groups.Rotor.Geometry); % Κάτω ημικύκλιο
    else
        createSegment(x_riA, y_riA, x_roA, y_roA, 'apbc1', Motor.Solver.Mesh.Rotor.Iron, 0, 0, Motor.Solver.Groups.Rotor.Geometry);
        createSegment(x_riB, y_riB, x_roB, y_roB, 'apbc1', Motor.Solver.Mesh.Rotor.Iron, 0, 0, Motor.Solver.Groups.Rotor.Geometry);
    end

    if t ~= 1
        numofcirc = 2;
    else
        numofcirc = 5;
    end

    % Sliding Band & apbc2/apbc3

    for i = 1:numofcirc

        if t == 1
            % Για τα θετικά σημεία
            x_circp = Rro + Hmag + i*l_g/(numofcirc+1);
            y_circp = 0;
            createPoint(x_circp, y_circp, 0, Motor.Solver.Groups.AirGap);
            
            % Για τα αρνητικά σημεία
            [x_circn, y_circn] = rotatePoint(Rro + Hmag + i*l_g/(numofcirc+1), 0, x0, y0, Rro + Hmag + i*l_g/(numofcirc+1), 360/(((mod(1,t)==0)+1)*t) * (pi/180));
            createPoint(x_circn, y_circn, 0, Motor.Solver.Groups.AirGap);
        %
        elseif t ~= 1
            % Για τα θετικά σημεία
            x_circp = Rro + Hmag + (i==1)*l_g/3 + (i==2)*(l_g - l_g/3);
            y_circp = 0;
            createPoint(x_circp, y_circp, 0, Motor.Solver.Groups.AirGap);
            % Για τα αρνητικά σημεία
            [x_circn, y_circn] = rotatePoint(x_circp, 0, x0, y0, x_circp, 360/(((mod(1,t)==0)+1)*t) * (pi/180));
            createPoint(x_circn, y_circn, 0, Motor.Solver.Groups.AirGap); 
        end
        
    
        % Κατασκευή των ημικυκλίων
        if t == 1
            createArc(x_circp, y_circp, x_circn, y_circn, 360/(((mod(1,t)==0)+1)*t), 1, 1, 'None', Motor.Solver.Groups.AirGap); % Πάνω ημικύκλιο
            createArc(x_circn, y_circn, x_circp, y_circp, 180, 5, 5, 'None', Motor.Solver.Groups.AirGap); % Κάτω ημικύκλιο
        elseif t ~= 1 && i == 1
            createArc(x_circp, y_circp, x_circn, y_circn, 360/(((mod(1,t)==0)+1)*t), 1, 1, 'SlidingBand', Motor.Solver.Groups.AirGap);
            createSegment(x_circp, y_circp, x_roA, y_roA, 'apbc2', Motor.Solver.Mesh.Air, 0, 0, Motor.Solver.Groups.AirGap);
            createSegment(x_circn, y_circn, x_roB, y_roB, 'apbc2', Motor.Solver.Mesh.Air, 0, 0, Motor.Solver.Groups.AirGap);
        elseif t ~= 1 && i == numofcirc
            [x_siB, y_siB] = rotatePoint(Rmag + l_g, 0, x0, y0, Rmag + l_g, 360/(((mod(1,t)==0)+1)*t) * (pi/180));
            createArc(x_circp, y_circp, x_circn, y_circn, 360/(((mod(1,t)==0)+1)*t), 1, 1, 'SlidingBand', Motor.Solver.Groups.AirGap);
            createSegment(x_circp, y_circp, Rmag + l_g, 0, 'apbc3', Motor.Solver.Mesh.Air, 0, 0, Motor.Solver.Groups.AirGap);
            createSegment(x_circn, y_circn, x_siB, y_siB, 'apbc3', Motor.Solver.Mesh.Air, 0, 0, Motor.Solver.Groups.AirGap);
        end
    end
    
    for i = 1:(numofcirc+1)
        if (mod(i,2) ~= 0) && t ~= 1
            [x_block, y_block] = rotatePoint(Rro + Hmag + (i==1)*(l_g/6) + (i==3)*(l_g - l_g/6), 0, x0, y0, Rro + Hmag + (i==1)*(l_g/6) + (i==3)*(l_g - l_g/6), 360/(2*t) * (pi/180));
            createBlocklabel(x_block, y_block, 'Air', 0, Motor.Solver.Mesh.Air, 0, 0, Motor.Solver.Groups.AirGap, 0);
        elseif t == 1
            [x_block, y_block] = rotatePoint(Rro + Hmag + l_g/(2*(numofcirc+1)) + (i-1)*l_g/(numofcirc+1), 0, x0, y0, Rro + Hmag + l_g/(2*(numofcirc+1)) + (i-1)*l_g/(numofcirc+1), 360/(2*t) * (pi/180));
            createBlocklabel(x_block, y_block, 'Air', 0, Motor.Solver.Mesh.Air, 0, 0, Motor.Solver.Groups.AirGap, 0);
        end
    end

end