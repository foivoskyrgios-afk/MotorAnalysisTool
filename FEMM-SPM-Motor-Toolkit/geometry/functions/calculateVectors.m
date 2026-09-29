function [parallel_vec, perpendicular_vec] = calculateVectors(x4, y4, x5, y5, x6, y6, x7, y7)
    % Υπολογίζει τα παράλληλα και κάθετα διανύσματα
    % 10/2025 F.G.Kyrgios
    
    % Διάνυσμα παράλληλο στις ευθείες (από 4-5)
    parallel_vec = [x5 - x4, y5 - y4];
    
    % Κανονικοποίηση
    parallel_length = norm(parallel_vec);
    if parallel_length == 0
        error('Τα σημεία 4 και 5 συμπίπτουν');
    end
    parallel_vec = parallel_vec / parallel_length;
    
    % Διάνυσμα κάθετο (περιστροφή 90 μοιρών)
    perpendicular_vec = [-parallel_vec(2), parallel_vec(1)];
    
    % Έλεγχος προσανατολισμού: βεβαιώνουμε ότι το κάθετο διάνυσμα δείχνει προς την ευθεία 6-7
    test_point = [x4 + perpendicular_vec(1), y4 + perpendicular_vec(2)];
    if distanceToLine(test_point(1), test_point(2), x6, y6, x7, y7) > distanceToLine(x4, y4, x6, y6, x7, y7)
        % Το κάθετο διάνυσμα δείχνει προς τη σωστή κατεύθυνση
    else
        % Αντιστροφή του καθέτου διανύσματος
        perpendicular_vec = [parallel_vec(2), -parallel_vec(1)];
    end
end