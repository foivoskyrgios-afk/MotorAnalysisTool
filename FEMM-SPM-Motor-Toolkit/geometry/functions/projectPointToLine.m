function projected_point = projectPointToLine(x_point, y_point, x1_line, y1_line, x2_line, y2_line)
    % Βρίσκει την προβολή του σημείου (x_point,y_point) στην άπειρη ευθεία 
    % που ορίζεται από (x1_line,y1_line) και (x2_line,y2_line)
    % 11/2025 F.G.Kyrgios
    
    % Διάνυσμα της ευθείας
    line_vec = [x2_line - x1_line, y2_line - y1_line];
    
    % Διάνυσμα από σημείο αναφοράς της ευθείας προς το σημείο
    point_vec = [x_point - x1_line, y_point - y1_line];
    
    % Μήκος της ευθείας στο τετράγωνο
    line_length_sq = line_vec(1)^2 + line_vec(2)^2;
    
    if line_length_sq == 0
        error('Τα σημεία της ευθείας συμπίπτουν');
    end
    
    % Παράμετρος προβολής (dot product) - ΧΩΡΙΣ περιορισμό στο [0,1]
    t = dot(point_vec, line_vec) / line_length_sq;
    
    % Υπολογισμός προβεβλημένου σημείου στην άπειρη ευθεία
    x_proj = x1_line + t * line_vec(1);
    y_proj = y1_line + t * line_vec(2);
    
    projected_point = [x_proj, y_proj];
end