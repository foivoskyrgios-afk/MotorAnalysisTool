function createBlocklabel(x, y, material, automesh, meshsize, circuit, magdir, group, turn)
    % 04/2025 F.G.Kyrgios

mi_addblocklabel(x, y);
mi_selectlabel(x, y);
mi_setblockprop(material,automesh,meshsize,circuit,magdir,group,turn);
mi_clearselected;
end