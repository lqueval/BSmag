function [] = BSmag_plot_field_points(BSmag,X,Y,Z)
%---------------------------------------------------
%  NAME:      BSmag_plot_field_points.m
%  WHAT:      Plots all the field points (where we want to calculate the field) in the default figure.
%  REQUIRED:  BSmag Toolbox 20150407
%  AUTHOR:    20150407, L. Queval (loic.queval@gmail.com)
%  COPYRIGHT: 2015, Loic Quéval, BSD License (http://opensource.org/licenses/BSD-3-Clause).
%
%  USE:
%    [] = BSmag_plot_field_points(BSmag,X,Y,Z)
%
%  INPUTS:
%    BSmag      = BSmag data structure
%    X          = Field points x-coordinate vector or matrix
%    Y          = Field points y-coordinate vector or matrix
%    Z          = Field points z-coordinate vector or matrix
%
%  OUTPUTS:
%---------------------------------------------------

%Plot M (where we want to calculate the field)
figure(1)
	plot3(X(:),Y(:),Z(:),'kx')
axis  tight