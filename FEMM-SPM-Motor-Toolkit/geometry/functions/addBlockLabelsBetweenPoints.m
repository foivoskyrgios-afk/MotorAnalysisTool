function addBlockLabelsBetweenPoints(x4, y4, x5, y5, x6, y6, x7, y7, k, Mesh, BlockName, Group, Turns, rotation)
    % Βάζει block labels στα κέντρα των τμημάτων της ευθείας 6-7
    %
    % Είσοδοι:
    %   x4, y4, x5, y5: Σημεία της πρώτης ευθείας (Σ4-Σ5)
    %   x6, y6, x7, y7: Σημεία της δεύτερης ευθείας (Σ6-Σ7)  
    %   k: Αριθμός τμημάτων
    %   material: Υλικό για το block label
    %   group: Ομάδα για το block label
    % 10/2025 F.G.Kyrgios

    if k < 2
        error('Ο αριθμός τμημάτων k πρέπει να είναι >= 2 για να υπάρχουν κενά ανάμεσα');
    end

    % 1. Υπολογισμός k+1 σημείων στην ευθεία 4-5
    points_line45 = calculateLinePoints(x4, y4, x5, y5, k);
    
    % 2. Βρες το μοναδιαίο κάθετο διάνυσμα στις παράλληλες ευθείες
    [parallel_vec, perpendicular_vec] = calculateVectors(x4, y4, x5, y5, x6, y6, x7, y7);
    
    % 3. Βρες την απόσταση μεταξύ των παράλληλων ευθειών
    distance = calculateParallelDistance(x4, y4, x6, y6, x7, y7, perpendicular_vec);
    
    % 4. Για κάθε σημείο στην ευθεία 4-5, βρες το απέναντι σημείο στην 6-7
    points_line67 = zeros(size(points_line45));
    
    for i = 1:size(points_line45, 1)
        x_point = points_line45(i, 1);
        y_point = points_line45(i, 2);
        
        % Προσθήκη του καθέτου διανύσματος (με το σωστό πρόσημο)
        x_proj = x_point + distance * perpendicular_vec(1);
        y_proj = y_point + distance * perpendicular_vec(2);
        
        points_line67(i, :) = [x_proj, y_proj];
    end
    
    % 5. Βρες τα κέντρα ανάμεσα σε διαδοχικά σημεία της ευθείας 6-7
    for i = 1:k
        % Σημεία i και i+1 στην ευθεία 6-7
        x1 = points_line67(i, 1);
        y1 = points_line67(i, 2);
        x2 = points_line67(i+1, 1);
        y2 = points_line67(i+1, 2);
        
        % Κέντρο μεταξύ των δύο σημείων
        x_center = (x1 + x2) / 2;
        y_center = (y1 + y2) / 2;

        [x_fixed, y_fixed] = rotatePoint(x_center, y_center, 0, 0, sqrt(x_center^2+y_center^2), rotation);

        % Δημιουργία block label
        createBlocklabel(x_fixed, y_fixed, 'Phase', 0, Mesh, BlockName, 0, Group, Turns);
    end
end