%---------------------------------------------------
%  NAME:      validation_loop_filament_onAxis.m
%  WHAT:      Loop filament in xy-plane, centered in (0,0,0)
%             comparison BSmag vs. analytical for field points on z-axis.
%  REQUIRED:  BSmag Toolbox 20150407
%  AUTHOR:    20150407, L. Queval (loic.queval@gmail.com)
%  COPYRIGHT: 2015, Loic Quéval, BSD License (http://opensource.org/licenses/BSD-3-Clause).
%----------------------------------------------------

clear all, close all, clc

%% Parameters 
I0 = 1;  % loop current [A]
b = 5e-2; % loop radius [m]

%% BSmag

% Initialize
BSmag = BSmag_init(); % Initialize BSmag analysis

% Source points (where there is a current source)
theta = linspace(0,2*pi,1e4);
Gamma = [b*cos(theta'),b*sin(theta'),theta'*0]; % x,y,z [m,m,m]
I = I0; % filament current [A]
dGamma = 1e9; % filament max discretization step [m]
[BSmag] = BSmag_add_filament(BSmag,Gamma,I,dGamma);

% Field points (where we want to calculate the field)
x_M = zeros(51,1); % x [m]
y_M = zeros(51,1); % y [m]
z_M = linspace(0,0.1,51)'; % z [m]
BSmag_plot_field_points(BSmag,x_M,y_M,z_M); % shows the field points volume

% Biot-Savart Integration
[BSmag,X,Y,Z,BX,BY,BZ] = BSmag_get_B(BSmag,x_M,y_M,z_M);

% Plot B/|B|
figure(1)
    normB=sqrt(BX.^2+BY.^2+BZ.^2);
    quiver3(X,Y,Z,BX./normB,BY./normB,BZ./normB,'b')
%axis tight

% Plot Bz on the line
figure(2)
	plot(Z, BZ, 'x'), hold on
xlabel ('z [m]'), ylabel (' Bz [T]')

%% Analytical
% Source : D.K. Cheng , Fundamentals Engineering Electromagnetics, Eq.(5.37) p.186.
% Note : In cylindrical coordinates

mu0 = 4*pi*1e-7;                   % vaccum permeability [N/A^2]
z = z_M;                           % Field point z-coordinate [m]
 
Brho = 0;                                   % Field point magnetic flux density rho-component [T]
Bth = 0;                                    % Field point magnetic flux density theta-component [T]
Bz = (mu0*I0*b.^2)./(2*(z.^2+b.^2).^(3/2));  % Field point magnetic flux density z-component [T]
         
% Plot Bz along z-axis
figure(2)
    plot(z,Bz)

%% Postprocessing
box on, grid on
legend('BSmag','Analytical'), legend('boxoff')