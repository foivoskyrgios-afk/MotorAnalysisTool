function [x2, y2] = rotatePoint(x1, y1, x0, y0, r, delta_theta)
    % 04/2025 F.G.Kyrgios
    
    % x0, y0: Συντεταγμένες κέντρου κύκλου
    % x1, y1: Συντεταγμένες γνωστού σημείου
    % r: Ακτίνα κύκλου
    % delta_theta: Γωνία περιστροφής

    % Υπολογίζω την γωνία του γνωστού σημείου σε σχέση με το κέντρο του
    % κύκλου
    theta1 = atan2(y1 - y0, x1 - x0);
    
    % Νέα γωνία
    theta2 = theta1 + delta_theta;
    
    % Υπολογισμός του νέου σημείου σε μορφή μιγαδικού
    P2_complex = x0 + 1i*y0 + r * exp(1i*theta2);
 
    % Συντεταγμένες σημείου
    x2 = real(P2_complex);
    y2 = imag(P2_complex);
end