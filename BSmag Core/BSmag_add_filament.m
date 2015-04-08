function [BSmag] = BSmag_add_filament(BSmag,Gamma,I,dGamma)
%---------------------------------------------------
%  NAME:      BSmag_add_filament.m
%  WHAT:      Adds a filament to the BSmag analysis.
%  REQUIRED:  BSmag Toolbox 20150407
%  AUTHOR:    20150407, L. Queval (loic.queval@gmail.com)
%  COPYRIGHT: 2015, Loic Quéval, BSD License (http://opensource.org/licenses/BSD-3-Clause).
%
%  USE:
%  [BSmag] = BSmag_add_filament(BSmag,Gamma,I,dGamma)
%
%  INPUTS:
%    BSmag      = BSmag data structure
%    Gamma      = Filament points coordinates (x,y,z), one point per line [m,m,m]
%    I          = Filament current (flows from first point towards last point) [A]
%    dGamma     = Filament max discretization step [m]
%
%  OUTPUTS:
%    BSmag      = Updated BSmag data structure
%      BSmag.Nfilament              = Number of filaments
%      BSmag.filament(*).*          = Filament structure
%      BSmag.filament(*).Gamma      = Filament points coordinates (x,y,z), one point per line [m,m,m]
%      BSmag.filament(*).I          = Filament current (flows from first point towards last point) [A]
%      BSmag.filament(*).dGamma     = Filament max discretization step [m]
%----------------------------------------------------

n = BSmag.Nfilament+1;
BSmag.filament(n).Gamma = Gamma;
BSmag.filament(n).I = I;
BSmag.filament(n).dGamma = dGamma;
BSmag.Nfilament = n;

%Plot P (where there is a current source)
figure(1)
	plot3(Gamma(:,1),Gamma(:,2),Gamma(:,3),'.-r')
axis tight