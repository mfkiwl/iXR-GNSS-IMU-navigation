function [gVector] = normalGravity(lat,height)
RE_WGS84  =  6378137.0;           %  earth semimajor axis (WGS84) (m)
FE_WGS84  =  (1.0/298.257223563); %  earth flattening (WGS84) */
BE_WGS84	=RE_WGS84*(1.0-FE_WGS84);	%/* earth semiminor axis (WGS84) (m) */
GAMMA_A=		9.7803267715;		%/* gravitational acceleration at semimajor axis (m/s^2) */
GAMMA_B=		9.8321863685;		%/* gravitational acceleration at semiminor axis (m/s^2) */
OMGE=        7.2921151467E-5;     %/* earth angular velocity (IS-GPS) (rad/s) */
GM	=		3.986005E14	;		%/* earth gravitational constant WGS84 (m^3/s^2) */

%// Calculate normal gravity at ellipsoidal surface according to Somigliana ([1], equations 2.139 and 2.140)
m = ((GAMMA_A*GAMMA_A) / GM)*OMGE*OMGE*BE_WGS84;
dGamma = (RE_WGS84*GAMMA_A*power(cos(lat), 2.0) + BE_WGS84 * GAMMA_B*power(sin(lat), 2.0)) /sqrt(power(RE_WGS84 * cos(lat), 2.0) + power(BE_WGS84 * sin(lat), 2.0));
%// Propagate gravity to current height and set g vector
gVector(1) = -8.08E-9 * height * sin(2.0 * lat);
gVector(2) = 0.0;
gVector(3) = dGamma * (1.0 - (2.0 / RE_WGS84) * (1.0 + FE_WGS84 + m - 2.0*FE_WGS84*power(sin(lat), 2.0))*height + 3.0 * power(height/ RE_WGS84, 2.0));
end

