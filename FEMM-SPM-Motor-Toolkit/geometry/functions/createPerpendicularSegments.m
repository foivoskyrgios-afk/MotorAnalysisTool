function createPerpendicularSegments(x4, y4, x5, y5, x6, y6, x7, y7, k, r, geometryGroup, ironMesh)
    % Δημιουργεί σημεία που ισαπέχουν r από ευθεία 4-5 και από ευθεία Χ-Χ'
    %
    % Είσοδοι:
    %   x4, y4, x5, y5: Σημεία της πρώτης ευθείας (Σ4-Σ5)
    %   x6, y6, x7, y7: Σημεία της δεύτερης ευθείας (Σ6-Σ7)  
    %   k: Αριθμός τμημάτων
    %   r: Απόσταση από την ευθεία 4-5 και από την ευθεία Χ-Χ'
    %   geometryGroup: Ομάδα για τα points
    %   ironGroup: Ομάδα για τα segments
    % 11/2025 F.G.Kyrgios

    if k < 1
        error('Ο αριθμός τμημάτων k πρέπει να είναι >= 1');
    end

    % 1. Υπολογισμός k+1 σημείων στην ευθεία 4-5
    points_line45 = calculateLinePoints(x4, y4, x5, y5, k);
    
    % 2. Βρες το μοναδιαίο κάθετο διάνυσμα στις παράλληλες ευθείες
    [~, perpendicular_vec] = calculateVectors(x4, y4, x5, y5, x6, y6, x7, y7);
    
    % 3. Βρες την απόσταση μεταξύ των παράλληλων ευθειών
    distance = calculateParallelDistance(x4, y4, x6, y6, x7, y7, perpendicular_vec);

    % d = distancelinetoline(x4, y4, x5, y5, x6, y6, x7, 7);

    % % 👉 Προσαρμογή r αν η απόσταση είναι μεγαλύτερη από 2 + r
    % if d >= (2 + r)
    %     r = d - 2;
    %     fprintf('⚙️ Απόσταση %.3f >= 2 + %.3f → νέο r = %.3f\n', abs(distance), r, r);
    % end

    % 4. Αποθήκευση σημείων για μελλοντικές συνδέσεις
    all_points_left = [];
    all_points_right = [];
    all_proj_points_left = [];
    all_proj_points_right = [];
    
    % 5. Για κάθε σημείο Χ στην ευθεία 4-5
    for i = 1:size(points_line45, 1)
        x_point = points_line45(i, 1);
        y_point = points_line45(i, 2);
        
        % Προβολή του σημείου Χ στην ευθεία 6-7
        x_proj = x_point + distance * perpendicular_vec(1);
        y_proj = y_point + distance * perpendicular_vec(2);
        
        % Βρες τα δύο σημεία που ισαπέχουν r από την ευθεία 4-5
        % και από την ευθεία Χ-Χ'
        [point1, point2] = findPointsAtDistanceFromTwoLines(...
            x4, y4, x5, y5, x_point, y_point, x_proj, y_proj, r);
        
        % Βρες τις προβολές των point1 και point2 στην ευθεία 6-7
        proj_point1 = projectPointToLine(point1(1), point1(2), x6, y6, x7, y7);
        proj_point2 = projectPointToLine(point2(1), point2(2), x6, y6, x7, y7);
        
        % Επιλογή σημείου ανάλογα με τη θέση
        if i == 1
            % Πρώτο σημείο: μόνο το αριστερό + προβολή του
            createPoint(point1(1), point1(2), 0, geometryGroup);
            createPoint(proj_point1(1), proj_point1(2), 0, geometryGroup);
            createSegment(point1(1), point1(2), proj_point1(1), proj_point1(2), 'None', ironMesh, 0, 0, geometryGroup);
            
            % Αποθήκευση μόνο του αριστερού
            all_points_left(i, :) = point1;
            all_proj_points_left(i, :) = proj_point1;
            
        elseif i == size(points_line45, 1)
            % Τελευταίο σημείο: μόνο το δεξί + προβολή του
            createPoint(point2(1), point2(2), 0, geometryGroup);
            createPoint(proj_point2(1), proj_point2(2), 0, geometryGroup);
            createSegment(point2(1), point2(2), proj_point2(1), proj_point2(2), 'None', ironMesh, 0, 0, geometryGroup);
            
            % Αποθήκευση μόνο του δεξιού
            all_points_right(i, :) = point2;
            all_proj_points_right(i, :) = proj_point2;
            
        else
            % Εσωτερικά σημεία: και τα δύο σημεία + προβολές + segments
            createPoint(point1(1), point1(2), 0, geometryGroup);
            createPoint(point2(1), point2(2), 0, geometryGroup);
            createPoint(proj_point1(1), proj_point1(2), 0, geometryGroup);
            createPoint(proj_point2(1), proj_point2(2), 0, geometryGroup);
            
            createSegment(point1(1), point1(2), proj_point1(1), proj_point1(2), 'None', ironMesh, 0, 0, geometryGroup);
            createSegment(point2(1), point2(2), proj_point2(1), proj_point2(2), 'None', ironMesh, 0, 0, geometryGroup);
            
            % Αποθήκευση και των δύο
            all_points_left(i, :) = point1;
            all_points_right(i, :) = point2;
            all_proj_points_left(i, :) = proj_point1;
            all_proj_points_right(i, :) = proj_point2;
        end
    end
    
    % 6. Σύνδεση: αριστερό του προηγούμενου με δεξί του επόμενου
    connection_points = []; % Αποθήκευση σημείων σύνδεσης για στρογγυλοποίηση
    
    for i = 1:(size(points_line45, 1)-1)
        % Βεβαιώσου ότι υπάρχουν και τα δύο σημεία
        if i == 1
            % Το πρώτο έχει μόνο αριστερό, το δεύτερο έχει και τα δύο
            if size(all_points_left, 1) >= 1 && size(all_points_right, 1) >= 2
                % Δημιουργία σημείων σύνδεσης
                x_left = all_points_left(1, 1);
                y_left = all_points_left(1, 2);
                x_right = all_points_right(2, 1);
                y_right = all_points_right(2, 2);
                
                createSegment(x_left, y_left, x_right, y_right, 'None', ironMesh, 0, 0, geometryGroup);
                
                % Αποθήκευση και των δύο σημείων για στρογγυλοποίηση
                connection_points = [connection_points; x_left, y_left];
                connection_points = [connection_points; x_right, y_right];
            end
        elseif i == (size(points_line45, 1)-1)
            % Το προτελευταίο έχει και τα δύο, το τελευταίο έχει μόνο δεξί
            if size(all_points_left, 1) >= i && size(all_points_right, 1) >= (i+1)
                x_left = all_points_left(i, 1);
                y_left = all_points_left(i, 2);
                x_right = all_points_right(i+1, 1);
                y_right = all_points_right(i+1, 2);
                
                createSegment(x_left, y_left, x_right, y_right, 'None', ironMesh, 0, 0, geometryGroup);
                
                connection_points = [connection_points; x_left, y_left];
                connection_points = [connection_points; x_right, y_right];
            end
        else
            % Εσωτερικά σημεία: και τα δύο έχουν και τα δύο σημεία
            if size(all_points_left, 1) >= i && size(all_points_right, 1) >= (i+1)
                x_left = all_points_left(i, 1);
                y_left = all_points_left(i, 2);
                x_right = all_points_right(i+1, 1);
                y_right = all_points_right(i+1, 2);
                
                createSegment(x_left, y_left, x_right, y_right, 'None', ironMesh, 0, 0, geometryGroup);
                
                connection_points = [connection_points; x_left, y_left];
                connection_points = [connection_points; x_right, y_right];
            end
        end
    end
    
    % 7. Στρογγυλοποίηση γωνιών σε όλα τα σημεία σύνδεσης
    for j = 1:size(connection_points, 1)
        x_conn = connection_points(j, 1);
        y_conn = connection_points(j, 2);
        mi_createradius(x_conn, y_conn, 0.1);
    end
end
