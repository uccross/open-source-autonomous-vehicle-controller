                function ECEF = lla2ecef2(lla)
%----------------------------------------------------------------------
%               function ECEF = lla2ecef2(lla)
%
%   converts latitude, longtitude and altitude coordinates (given in
%   radians and meters) into ECEF coordinates (given in meters).
%   Handles vectors [lat(:) long(:) alt(:)]
%
%   Note: Alitude should be negative!!   
%
%   Modified for spherical earth - COLB 8/8/01
%
%   Demoz Gebre 8/18/98 
%---------------------------------------------------------------------

N = length(lla(:,1));

EarthRad = 6378137.0;

ecc = 0.0;
ecc2 = ecc*ecc;

sinlat = sin(lla(:,1));
coslat = cos(lla(:,1));
%Rn = EarthRad / sqrt(abs(1.0 - (ecc2 * sinlat * sinlat)));
Rn = EarthRad;
ECEF(:,1) = (Rn*ones(N,1) - lla(:,3)) .* coslat .* cos(lla(:,2));
ECEF(:,2) = (Rn*ones(N,1) - lla(:,3)) .* coslat .* sin(lla(:,2));
ECEF(:,3) = (Rn*ones(N,1) - lla(:,3)) .* sinlat;
