%---------------------------------------------------
%  NAME:      example_2D_bent_filaments.m
%  WHAT:      Calculation of the magnetic field of two bent filaments
%             on a plane (+ 2D plot).
%  REQUIRED:  BSmag Toolbox 20150407
%  AUTHOR:    20150407, L. Queval (loic.queval@gmail.com)
%  COPYRIGHT: 2015, Loic Quéval, BSD License (http://opensource.org/licenses/BSD-3-Clause).
%----------------------------------------------------

% Initialize
clear all, close all, clc
BSmag = BSmag_init(); % Initialize BSmag analysis

% Source points (where there is a current source)
Gamma1 = [1,-0.5,-1; % x,y,z [m,m,m]
          1,0.2,0.2;
          1,-0.3,1];
I1 = 2; % filament current [A]
dGamma1 = 1e-3; % filament max discretization step [m]  
[BSmag] = BSmag_add_filament(BSmag,Gamma1,I1,dGamma1);

Gamma2 = [-1,0,-1; % x,y,z [m,m,m]
          -1,0,1;
          -1,0.5,1;
          -1,0.5,-1
          -1,0,-1];
I2 = 5; % filament current [A]
dGamma2 = 1e-3; % filament max discretization step [m]  
[BSmag] = BSmag_add_filament(BSmag,Gamma2,I2,dGamma2);

% Field points (where we want to calculate the field)
x_M = linspace(-2, 2, 41); % x [m]
y_M = linspace(-1, 1, 21); % y [m]
[X_M,Y_M] = ndgrid(x_M,y_M);
Z_M = zeros(41,21); % z [m]
BSmag_plot_field_points(BSmag,X_M,Y_M,Z_M); % shows the field points plane

% Biot-Savart Integration
[BSmag,X,Y,Z,BX,BY,BZ] = BSmag_get_B(BSmag,X_M,Y_M,Z_M);

% Plot B/|B|
figure(1)
    normB=sqrt(BX.^2+BY.^2+BZ.^2);
    quiver3(X,Y,Z,BX./normB,BY./normB,BZ./normB,'b')

% Plot By on the plane
figure(2), hold on, box on, grid on
    contourf(X, Y, BY), colorbar
xlabel ('x [m]'), ylabel ('y [m]'), title ('By [T]')