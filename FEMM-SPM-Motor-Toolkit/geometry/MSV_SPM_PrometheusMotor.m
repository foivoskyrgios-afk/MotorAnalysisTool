% MSV (Machine State Vector) definition: Basic design parameter structure
% 03/2025 F.G.Kyrgios

%% Winding Sequence
Motor.Configuration.Winding.Sequence                       = 'A|A|b|b|C|C|a|a|B|B|c|c|A|A|b|b|C|C|a|a|B|B|c|c|A|A|b|b|C|C|a|a|B|B|c|c|A|A|b|b|C|C|a|a|B|B|c|c|';

%% Basic parameters
Motor.Configuration.Basic.Poles                            = 8;
Motor.Configuration.Basic.Slots                            = 48;
Motor.Configuration.Basic.Layers                           = 1;
Motor.Configuration.Basic.Shift                            = 0;
Motor.Configuration.Basic.Periodicity                      = 1; % 1 = Yes / 0 = No
Motor.Configuration.Basic.Phases                           = 3;
Motor.Configuration.Basic.AirGap                           = 1;
Motor.Configuration.Basic.ActiveLength                     = 1;
Motor.Configuration.Basic.FillFactor                       = 0.6; % Unused
Motor.Configuration.Basic.NominalCurrent                   = 0;
Motor.Configuration.Basic.Bars                             = 2; % 0 = No Bars / 1 = Maxed Size Bars / 2 = Fixed Size Bars
Motor.Configuration.Basic.NumOfBars                        = 13;
Motor.Configuration.Basic.Lx                               = 1;
Motor.Configuration.Basic.Ly                               = 4;
Motor.Configuration.Basic.radius                           = 0.4; % Fillet radius on edges of bars
% Periodicity
Motor.Configuration.Basic.t = gcd(Motor.Configuration.Basic.Poles, (Motor.Configuration.Basic.Slots)*(Motor.Configuration.Basic.Periodicity == 1) + 1*(Motor.Configuration.Basic.Periodicity == 0));
% SPP
Motor.Configuration.Basic.q = Motor.Configuration.Basic.Slots/(3*Motor.Configuration.Basic.Poles);

%% Stator Configuration
Motor.Configuration.Stator.WbiRatio                        = 2.5; % Percentage of Wbi/Wt (Yoke 2 Tooth)
Motor.Configuration.Stator.Rso                             = 78;
Motor.Configuration.Stator.WtRatio                         = 0.9;
Motor.Configuration.Stator.beta_t                          = 0.1;
Motor.Configuration.Stator.theta_tip                       = 0.5; % Percentage of whole available space to captured space of tip
Motor.Configuration.Stator.h_tp                            = 0.2;
Motor.Configuration.Stator.CoilTurns                       = 13;

%% Rotor Configuration
Motor.Configuration.Rotor.Rri                              = 31;
Motor.Configuration.Rotor.Rro                              = 46;

%% Magnet Configuration
Motor.Configuration.Magnet.Hmag                            = 8; % mm
Motor.Configuration.Magnet.Hmage                           = 5.5; % mm (edges)
Motor.Configuration.Magnet.Nodes                           = 40;
Motor.Configuration.Magnet.theta_mag                       = 0.93; % Percentage of whole availabe space to captured space of magnet