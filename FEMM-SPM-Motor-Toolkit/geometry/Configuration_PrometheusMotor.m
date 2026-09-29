function [Motor] = Configuration_PrometheusMotor(Motor)
    % Defines basic solver settings, group numbering and mesh sizes
    % 03/2025 F.G.Kyrgios
    
    %% General parameters
    Motor.Solver.Precision                                             = 1e-8; % Solver precision, default: 1e-8
    Motor.Solver.MinAngle                                              = 15;   % Minimum mesh triangle angle, default: 15
    
    %% Group numbering
    % Air
    Motor.Solver.Groups.Air                                            = 901;
    Motor.Solver.Groups.AirGap                                         = 902;
    
    % Stator groups
    Motor.Solver.Groups.Stator.Iron                                    = 101;
    Motor.Solver.Groups.Stator.Geometry                                = 102;
    
    % Windings
    Motor.Solver.Groups.Stator.Winding                                 = 103;
    
    % Rotor
    Motor.Solver.Groups.Rotor.Iron                                     = 201;
    Motor.Solver.Groups.Rotor.Geometry                                 = 202;
    
    % Magnet
    Motor.Solver.Groups.Magnet.Geometry                                = 301;
    Motor.Solver.Groups.Magnet.Magnet                                  = 302;
    
    %% Mesh sizes
    % Air
    Motor.Solver.Mesh.Air                                              = 0.7;
    
    % Stator
    Motor.Solver.Mesh.Stator.Iron                                      = 0.7;
    Motor.Solver.Mesh.Stator.Winding                                   = 0.7;
    
    % Rotor
    Motor.Solver.Mesh.Rotor.Iron                                       = 0.7;
    
    % Magnet
    Motor.Solver.Mesh.Magnet                                           = 0.7;
end