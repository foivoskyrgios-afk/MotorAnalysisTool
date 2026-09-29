function [point1, point2] = findPointsAtDistanceFromTwoLines(x4, y4, x5, y5, xX, yX, xXp, yXp, r)
    % Βρίσκει δύο σημεία που ισαπέχουν r από ευθεία 4-5 και από ευθεία Χ-Χ'
    % 11/2025 F.G.Kyrgios
    
    % Διάνυσμα της ευθείας 4-5
    vec45 = [x5 - x4, y5 - y4];
    vec45_length = norm(vec45);
    if vec45_length == 0
        error('Τα σημεία 4 και 5 συμπίπτουν');
    end
    vec45_unit = vec45 / vec45_length;
    
    % Διάνυσμα κάθετο στην ευθεία 4-5
    perp45 = [-vec45_unit(2), vec45_unit(1)];
    
    % Διάνυσμα της ευθείας Χ-Χ' (κάθετη στις 4-5 και 6-7)
    vecXXp = [xXp - xX, yXp - yX];
    vecXXp_length = norm(vecXXp);
    if vecXXp_length == 0
        error('Τα σημεία Χ και Χ'' συμπίπτουν');
    end
    vecXXp_unit = vecXXp / vecXXp_length;
    
    % Διάνυσμα κάθετο στην ευθεία Χ-Χ'
    perpXXp = [-vecXXp_unit(2), vecXXp_unit(1)];
    
    % Το σημείο τομής των δύο ευθειών είναι το σημείο Χ
    intersection = [xX, yX];
    
    % Τα ζητούμενα σημεία βρίσκονται στις διχοτόμους των γωνιών
    % μεταξύ των δύο ευθειών
    
    % Διάνυσμα διχοτόμου 1 (άθροισμα των κάθετων διανυσμάτων)
    bisector1 = perp45 + perpXXp;
    bisector1_length = norm(bisector1);
    if bisector1_length > 0
        bisector1_unit = bisector1 / bisector1_length;
    else
        % Αν τα διανύσματα είναι αντίθετα, χρησιμοποιούμε ένα από τα δύο
        bisector1_unit = perp45;
    end
    
    % Διάνυσμα διχοτόμου 2 (διαφορά των κάθετων διανυσμάτων)
    bisector2 = perp45 - perpXXp;
    bisector2_length = norm(bisector2);
    if bisector2_length > 0
        bisector2_unit = bisector2 / bisector2_length;
    else
        % Αν τα διανύσματα είναι ίδια, χρησιμοποιούμε το κάθετο του ενός
        bisector2_unit = [perp45(2), -perp45(1)];
    end
    
    % Απόσταση από σημείο τομής σε σημείο που απέχει r και από τις δύο ευθείες
    % Σε ορθή γωνία, η απόσταση είναι r * sqrt(2)
    d = r * sqrt(2);
    
    % Τα δύο σημεία
    point1 = intersection + d * bisector1_unit;
    point2 = intersection + d * bisector2_unit;
end