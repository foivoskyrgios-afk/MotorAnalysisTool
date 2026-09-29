function distance = calculateParallelDistance(x4, y4, x6, y6, x7, y7, perpendicular_vec)
    % Υπολογίζει την απόσταση μεταξύ των παράλληλων ευθειών
    % 10/2025 F.G.Kyrgios
    
    % Βρες την προβολή του σημείου 4 στην ευθεία 6-7
    t = ((x6 - x4) * (x7 - x6) + (y6 - y4) * (y7 - y6)) / ((x7 - x6)^2 + (y7 - y6)^2);
    x_proj = x6 + t * (x7 - x6);
    y_proj = y6 + t * (y7 - y6);
    
    % Απόσταση μεταξύ σημείου 4 και της προβολής του
    distance = sqrt((x4 - x_proj)^2 + (y4 - y_proj)^2);
    
    % Έλεγχος προσανατολισμού
    test_point = [x4 + perpendicular_vec(1), y4 + perpendicular_vec(2)];
    test_distance = distanceToLine(test_point(1), test_point(2), x6, y6, x7, y7);
    
    if test_distance > distance
        % Το κάθετο διάνυσμα δείχνει προς τη λάθος κατεύθυνση
        distance = -distance;
    end
end