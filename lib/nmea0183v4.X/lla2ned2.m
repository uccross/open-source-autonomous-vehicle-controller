function ned = lla2ned2(lla, ref)
%----------------------------------------------------------------------
%               function ned = lla2ned(lla, ref)
%
%   converts latitude, longtitude and altitude coordinates (given in
%   degrees and meters) into ECEF coordinates (given in meters).
%   Handles vectors [lat(:) long(:) alt(:)]
%   with ref given as [lat(:) long(:) alt(:)]
%
%   Note: Alitude should be negative!!   
%
%   Modified for spherical earth - COLB 8/8/01
%
%
%---------------------------------------------------------------------

EarthRad = 6378137.0;
d2r = pi/180;

lla(:,1) = lla(:,1)*pi/180; % convert to radians
lla(:,2) = lla(:,2)*pi/180; % convert to radians
lla(:,3) = lla(:,3);  % convert to meters

lat = d2r*ref(1);
long = d2r*ref(2);
alt = ref(3);

% Form rotation matrix

Te2n = [-sin(lat)*cos(long) -sin(lat)*sin(long) cos(lat);
        -sin(long)          cos(long)           0;
        -cos(lat)*cos(long) -cos(lat)*sin(long) -sin(lat)];

N = length(lla(:,1));

ecef = lla2ecef2(lla);

ned = (Te2n*(ecef'))';

ned = ned + (EarthRad + alt)*[zeros(N,2) ones(N,1)];
