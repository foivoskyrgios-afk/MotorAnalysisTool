function createPoint(x, y, prop, group)
    % 11/2025 F.G.Kyrgios
mi_addnode(x,y);
mi_selectnode(x,y);
mi_setnodeprop(prop, group);
mi_clearselected();
end