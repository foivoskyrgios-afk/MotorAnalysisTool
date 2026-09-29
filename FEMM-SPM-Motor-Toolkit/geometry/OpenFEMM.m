function [Motor] = OpenFEMM(Motor)
% Opens FEMM and applies basic solver settings
% 09/2025 F.G.Kyrgios

openfemm;
newdocument(0);
mi_probdef(0, 'millimeters', 'planar', Motor.Solver.Precision, Motor.Configuration.Basic.ActiveLength, Motor.Solver.MinAngle);

end