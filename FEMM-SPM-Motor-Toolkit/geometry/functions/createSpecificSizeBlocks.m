function createSpecificSizeBlocks(x4, y4, x5, y5, x6, y6, x7, y7, Lx, Ly, k, group, mesh, radius)
    % Δημιουργεί blocks συγκεκριμένου πλάτους πάνω στην προέκταση της ευθείας 6-7
    % με βάση προβολές σημείων από την ευθεία 4-5.
    % 11/2025 F.G.Kyrgios

    if k < 1
        error('Ο αριθμός τμημάτων k πρέπει να είναι >= 1');
    end

    % 1️⃣ Υπολογισμός σημείων κατά μήκος της 4–5
    points_line45 = calculateLinePoints(x4, y4, x5, y5, k);

    % 2️⃣ Μοναδιαίο διάνυσμα της 6–7
    v67 = [x7 - x6, y7 - y6];
    v67_unit = v67 / norm(v67);
    
    % 3️⃣ Μοναδιαίο κάθετο διάνυσμα (για μετατόπιση Ly μονάδων)
    v67_perp = [-v67_unit(2), v67_unit(1)];  % Περιστροφή 90 μοιρών

    % 4️⃣ Προβολή των σημείων της 4–5 στην προέκταση της 6–7
    projected_points = zeros(size(points_line45));
    for i = 1:size(points_line45, 1)
        projected_points(i, :) = projectPointToLine(points_line45(i,1), points_line45(i,2), x6, y6, x7, y7);
    end

    % 5️⃣ Υπολογισμός αποστάσεων μεταξύ διαδοχικών projected points
    num_proj = size(projected_points, 1);
    p_values = zeros(num_proj-1, 1);
    for i = 1:num_proj-1
        p_values(i) = sqrt((projected_points(i+1,1)-projected_points(i,1))^2 + ...
                           (projected_points(i+1,2)-projected_points(i,2))^2);
    end

    % 6️⃣ Αποθήκευση σημείων για σύνδεση
    left_points = [];
    right_points = [];
    lower_left_points = [];
    lower_right_points = [];

    % 7️⃣ Δημιουργία σημείων
    for i = 1:num_proj
        if i == 1
            p = p_values(1);
        elseif i == num_proj
            p = p_values(end);
        else
            p = mean([p_values(i-1), p_values(i)]);
        end

        d = (p - Lx)/2;
        proj_x = projected_points(i,1);
        proj_y = projected_points(i,2);

        % Αριστερό και δεξί σημείο
        if i ~= 1
            left_point = [proj_x - d * v67_unit(1), proj_y - d * v67_unit(2)];
            createPoint(left_point(1), left_point(2), 0, group);
            
            % 📍 Κάτω σημείο από αριστερό σημείο (απόσταση Ly)
            point_Ly_down = left_point - Ly * v67_perp;
            createPoint(point_Ly_down(1), point_Ly_down(2), 0, group);
            
            % Αποθήκευση για σύνδεση
            left_points = [left_points; left_point];
            lower_left_points = [lower_left_points; point_Ly_down];
        end

        if i ~= num_proj
            right_point = [proj_x + d * v67_unit(1), proj_y + d * v67_unit(2)];
            createPoint(right_point(1), right_point(2), 0, group);
            
            % 📍 Κάτω σημείο από δεξί σημείο (απόσταση Ly)
            point_Ly_down = right_point - Ly * v67_perp;
            createPoint(point_Ly_down(1), point_Ly_down(2), 0, group);
            
            % Αποθήκευση για σύνδεση
            right_points = [right_points; right_point];
            lower_right_points = [lower_right_points; point_Ly_down];
        end
    end

    % 8️⃣ 🔗 ΣΥΝΔΕΣΗ ΣΗΜΕΙΩΝ
    
    % Α) Σύνδεση ΟΡΙΖΟΝΤΙΩΝ - κάτω σημεία (αριστερό ↔ δεξί)
    for i = 1:min(size(lower_left_points, 1), size(lower_right_points, 1))
        createSegment(lower_left_points(i,1), lower_left_points(i,2), ...
                     lower_right_points(i,1), lower_right_points(i,2), 'None', mesh, 0, 0, group);
    end
    
    % Β) Σύνδεση ΚΑΘΕΤΩΝ - αριστερά σημεία (κεντρικό ↔ κάτω)
    for i = 1:min(size(left_points, 1), size(lower_left_points, 1))
        createSegment(left_points(i,1), left_points(i,2), ...
                     lower_left_points(i,1), lower_left_points(i,2), 'None', mesh, 0, 0, group);
    end
    
    % Γ) Σύνδεση ΚΑΘΕΤΩΝ - δεξιά σημεία (κεντρικό ↔ κάτω)
    for i = 1:min(size(right_points, 1), size(lower_right_points, 1))
        createSegment(right_points(i,1), right_points(i,2), ...
                     lower_right_points(i,1), lower_right_points(i,2), 'None', mesh, 0, 0, group);
    end

    % 9️⃣ 🔄 ΣΤΡΟΓΓΥΛΟΠΟΙΗΣΗ ΓΩΝΙΩΝ ΣΤΑ LOWER ΣΗΜΕΙΑ
    connection_points = [lower_left_points; lower_right_points];
    
    for j = 1:size(connection_points, 1)
        x_conn = connection_points(j, 1);
        y_conn = connection_points(j, 2);
        mi_createradius(x_conn, y_conn, radius);
    end
end