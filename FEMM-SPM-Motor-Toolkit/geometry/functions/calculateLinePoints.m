function points = calculateLinePoints(x1, y1, x2, y2, k)
    % Υπολογίζει k+1 σημεία σε μια ευθεία μεταξύ (x1,y1) και (x2,y2)
    % 10/2025 F.G.Kyrgios
    dx = x2 - x1;
    dy = y2 - y1;
    
    points = zeros(k + 1, 2);
    
    for i = 0:k
        t = i / k;
        x = x1 + t * dx;
        y = y1 + t * dy;
        points(i + 1, :) = [x, y];
    end
end