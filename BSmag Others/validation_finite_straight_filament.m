%---------------------------------------------------
%  NAME:      validation_finite_straight_filament.m
%  WHAT:      Finite straight filament along z-axis, centered in (0,0,0)
%             comparison BSmag vs. analytical.
%  REQUIRED:  BSmag Toolbox 20150407
%  AUTHOR:    20150407, L. Queval (loic.queval@gmail.com)
%  COPYRIGHT: 2015, Loic Quéval, BSD License (http://opensource.org/licenses/BSD-3-Clause).
%----------------------------------------------------

clear all, close all, clc

%% Parameters 
I0 = 1;  % filament current [A]
L = 10e-2; % filament length [m]

%% BSmag

% Initialize
BSmag = BSmag_init(); % Initialize BSmag analysis

% Source points (where there is a current source)
Gamma   = [0,0,-L/2; % x,y,z [m,m,m]
           0,0,L/2];
I = I0; % filament current [A]
dGamma = 1e-7; % filament max discretization step [m]      
[BSmag] = BSmag_add_filament(BSmag,Gamma,I,dGamma);

% Field points (where we want to calculate the field)
x_M = logspace(-6,0,50); % x [m]
z_M = [0,5e-2,10e-2]; % z [m]
[X_M,Z_M] = ndgrid(x_M,z_M);
Y_M = zeros(50,3); % z [m]
BSmag_plot_field_points(BSmag,X_M,Y_M,Z_M); % -> shows the field point line

% Biot-Savart Integration
[BSmag,X,Y,Z,BX,BY,BZ] = BSmag_get_B(BSmag,X_M,Y_M,Z_M);      

% Plot B/|B|
figure(1)
    normB=sqrt(BX.^2+BY.^2+BZ.^2);
    quiver3(X,Y,Z,BX./normB,BY./normB,BZ./normB,'b')
axis tight

% Plot By on the line
figure(2)
	loglog(X, BY, 'x'), hold on
xlabel ('x [m]'), ylabel (' By [T]')

%% Analytical
% Source : P. Leuchtmann, Einführung in die elektromagnetische Feldtheorie, p.101
% Note : In cylindrical coordinates

mu0 = 4*pi*1e-7;                   % vaccum permeability [N/A^2]
rho = logspace(-6,0,100) ;        % Field point rho-coordinate [m]
z = [0,5e-2,10e-2];                % Field point z-coordinate [m]
[RHO,Z] = ndgrid(rho,z);

Brho = 0;                          % Field point magnetic flux density rho-component [T]                                                                                        
Bth = mu0*I0./(4*pi*RHO).*((Z+L/2)./sqrt((RHO.^2+(Z+L/2).^2))-(Z-L/2)./sqrt((RHO.^2+(Z-L/2).^2)));
                                   % Field point magnetic flux density th-component [T]
Bz = 0;                            % Field point magnetic flux density z-component [T]                                                                
                                   
% Plot By on the lines
figure(2)
	plot(RHO, Bth, '-')

%% Postprocessing
box on, grid on
legend('BSmag, z = 0 mm','BSmag, z = 50 mm','BSmag, z = 100 mm','Analytical, z = 0 mm','Analytical, z = 50 mm','Analytical, z = 100 mm'), legend('boxoff')