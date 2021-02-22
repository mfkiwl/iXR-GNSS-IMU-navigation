function [w_en_n] = transportRates(velNorth,velEast,lat,height)
RE_WGS84  =  6378137.0;           %  earth semimajor axis (WGS84) (m)
FE_WGS84  =  (1.0/298.257223563); %  earth flattening (WGS84) */
ECC_WGS84	=sqrt(FE_WGS84 * (2.0- FE_WGS84)); %/* Inverse earth flattening */
%/* earth equatorial radius, [1], eq. 2.105 */
R_N = RE_WGS84 * (1.0 - power(ECC_WGS84, 2.0)) / power((1.0 - power(ECC_WGS84, 2.0)*power(sin(lat),2.0)), 3.0 / 2.0);
%/* Earth curvature in meridional direction, [1], eq. 2.106 */
R_E = RE_WGS84 / sqrt(1.0 - power(ECC_WGS84, 2.0)*power(sin(lat), 2.0));
%/* Transport rate */
w_en_n(1) = velEast / (R_E + height);
w_en_n(2) = -velNorth / (R_N + height);
w_en_n(3) = -(velEast * tan(lat)) / (R_E + height);
end

