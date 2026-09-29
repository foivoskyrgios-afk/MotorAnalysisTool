function d = distancelinetoline(x4, y4, x5, y5, x6, y6, x7, y7)
% DISTANCE Calculate the distance between two infinite lines
%   Uses the formula for distance between lines in 2D space
% 11/2025 F.G.Kyrgios

% Points on first line: P1=(x4,y4), P2=(x5,y5)
% Points on second line: Q1=(x6,y6), Q2=(x7,y7)

% Calculate direction vectors
v1 = [x5 - x4, y5 - y4];  % Direction of first line
v2 = [x7 - x6, y7 - y6];  % Direction of second line

% Vector between points from different lines
PQ = [x6 - x4, y6 - y4];

% Calculate the distance using the formula:
% d = |(PQ · (v1 × v2))| / ‖v1 × v2‖
% For 2D, v1 × v2 is a scalar (the determinant)

cross_product = v1(1)*v2(2) - v1(2)*v2(1);  % 2D cross product (determinant)

if abs(cross_product) < eps
    % Lines are parallel - use point-to-line distance
    d = abs(PQ(1)*v1(2) - PQ(2)*v1(1)) / norm(v1);
else
    % Lines are not parallel
    d = abs(PQ(1)*v2(2) - PQ(2)*v2(1)) / abs(cross_product) * norm(v1);
    % Alternative formula: d = |det([PQ; v2])| / |cross_product|
    % d = abs(PQ(1)*v2(2) - PQ(2)*v2(1)) / abs(cross_product);
end

end