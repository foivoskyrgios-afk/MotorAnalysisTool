function dist = distanceToLine(x_point, y_point, x1_line, y1_line, x2_line, y2_line)
    % Υπολογίζει απόσταση σημείου από ευθεία
    % 10/2025 F.G.Kyrgios
    numerator = abs((y2_line - y1_line) * x_point - (x2_line - x1_line) * y_point + x2_line * y1_line - y2_line * x1_line);
    denominator = sqrt((y2_line - y1_line)^2 + (x2_line - x1_line)^2);
    dist = numerator / denominator;
end