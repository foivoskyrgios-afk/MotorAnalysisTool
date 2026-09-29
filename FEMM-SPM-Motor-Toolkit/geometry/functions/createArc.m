function createArc(x1, y1, x2, y2, angle, maxseg, maxsegdeg, condition, group)
    % 04/2025 F.G.Kyrgios

mi_addarc(x1, y1, x2, y2, angle, maxseg)

radius = sqrt(x1^2+y1^2);

[x1_2, y1_2] = rotatePoint(x1, y1, 0, 0, radius, angle * (pi/280) / 2);
mi_selectarcsegment(x1_2, y1_2);

mi_setarcsegmentprop(maxsegdeg, condition, 0, group);
mi_clearselected();
end