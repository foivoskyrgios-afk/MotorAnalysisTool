function createSegment(x1, y1, x2, y2, condition, elementsize, automesh, hide, group)
    % 04/2025 F.G.Kyrgios

mi_addsegment(x1, y1, x2, y2);
mi_selectsegment((x1+x2)/2, (y1+y2)/2);
mi_setsegmentprop(condition, elementsize, automesh, hide, group);
mi_clearselected();
end