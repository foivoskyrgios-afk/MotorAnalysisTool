% * ***************************************** *
% * Motor Parametric Design and Analysis Tool * 
% *     Electrical Machine Design - NTUA      *
% *              Diploma Thesis               *
% *    Created: 31/03/2025 by F.G.Kyrgios     *
% * ***************************************** *

%% 1. Initialize workspace
clear;
clc;

% Repository root (= parent folder of this file) and all sub-folders on the path
dir_Database = fileparts(fileparts(mfilename('fullpath')));
addpath(genpath(dir_Database));

% Add femm m-files to Matlab path (change if FEMM is installed elsewhere)
addpath('C:\femm42\mfiles');

%% 2. Define a parametric vector for the geometry 

% Run MSV
MSV_SPM_PrometheusMotor;
% Configuration file
[Motor] = Configuration_PrometheusMotor(Motor);

%% 3. FEMM drawing

% a. Open FEMM problem and define material properties and BCs
[Motor] = OpenFEMM(Motor);
[Motor] = MaterialsBcs_PrometheusMotor(Motor);

% b. Create Stator
[Motor] = DrawStatorGeometry(Motor);

% c. Create Rotor
[Motor] = DrawRotorGeometry(Motor);

% d. Create Winding
[Motor] = CreateWinding(Motor);
