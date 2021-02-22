function gnssins = buildTransitionMatrix(gnssins, dt)

RE_WGS84  =  6378137.0;           %  earth semimajor axis (WGS84) (m)
FE_WGS84  =  (1.0/298.257223563); %  earth flattening (WGS84) */
ECC_WGS84 = sqrt(FE_WGS84 * (2.0- FE_WGS84)); %/* Inverse earth flattening */
OMGE =        7.2921151467E-5;     %/* earth angular velocity (IS-GPS) (rad/s) */

gnssins.curMat3by3_1 = zeros(3, 3);
gnssins.curMat3by3_2 = zeros(3, 3);
gnssins.curMat15by15_1 = zeros(15,15);

%/* earth equatorial radius */
gnssins.R_N = RE_WGS84 * (1.0 - power(ECC_WGS84, 2.0)) / power((1.0 - power(ECC_WGS84, 2)*power(sin(gnssins.posVel(1)), 2)), 3.0 / 2.0);
%/* Earth curvature in meridional direction */
gnssins.R_E = RE_WGS84 / sqrt(1.0 - power(ECC_WGS84, 2.0)*power(sin(gnssins.posVel(1)), 2.0));

%/* Phi Matrix (Transition matrix) */
lat = gnssins.posVel(1);
lon = gnssins.posVel(2);
hgt = gnssins.posVel(3);
vN = gnssins.posVel(4);
vE = gnssins.posVel(5);
vD = gnssins.posVel(6);
%// F11 Pos Pos Error in meter
gnssins.F(2) = ((vE * tan(lat)) / (gnssins.R_N + hgt));
gnssins.F(31) = (vN / (gnssins.R_N + hgt));
gnssins.F(32) = (vE / (gnssins.R_E + hgt));
%// F12 (Pos Vel Error in meter)
gnssins.F(46) = 1.0;
gnssins.F(62) = 1.0;
gnssins.F(78) = 1.0;
%// F21 (Vel Pos Error in meter)
gnssins.F(4) = (((-power(1.0 / cos(lat), 2.0) * power(vE, 2)) / ((gnssins.R_E + hgt) * (gnssins.R_N + hgt))) - ((2.0 * vE * OMGE * cos(lat)) / (gnssins.R_N + hgt)));
gnssins.F(5) = (((power(1.0 / cos(lat), 2.0) * vN * vE) / ((gnssins.R_E + hgt) * (gnssins.R_N + hgt))) + ((2.0 * vN * OMGE * cos(lat)) / (gnssins.R_N + hgt)) - ((2.0 * vD * sin(lat)) / (gnssins.R_N + hgt)));
gnssins.F(6) = ((2.0 * vE * OMGE * sin(lat)) / (gnssins.R_N + hgt));
gnssins.F(34) = (-((power(vE, 2.0) * tan(lat)) / power(gnssins.R_E + hgt, 2.0)) + ((vN * vD) / power(gnssins.R_N + hgt, 2.0)));
gnssins.F(35) = ((vN * vE * tan(lat) + vE * vD) / power(gnssins.R_E + hgt, 2.0));
r_e_e_S = gnssins.R_E * sqrt(power(cos(lat), 2.0) + power(1.0 - power(ECC_WGS84, 2.0), 2.0) * power(sin(lat), 2.0));
gnssins.gVector = normalGravity(lat, hgt);
g0 = gnssins.gVector(3);
gnssins.F(36) = (-(power(vE, 2.0) / power(gnssins.R_E + hgt, 2.0)) - (power(vN, 2.0) / power(gnssins.R_N + hgt, 2.0)) + (2.0 * g0) / r_e_e_S);
%// F22
gnssins.F(49) = (vD / (gnssins.R_N + hgt));
gnssins.F(50) = (((vE * tan(lat)) / (gnssins.R_E + hgt)) + 2.0 * OMGE * sin(lat));
gnssins.F(51) = (-(2.0 * vN) / (gnssins.R_N + hgt));
gnssins.F(64) = ((-(2.0 * vE * tan(lat)) / (gnssins.R_E + hgt)) - 2.0 * OMGE * sin(lat));
gnssins.F(65) = ((vN * tan(lat) + vD) / (gnssins.R_E + hgt));
gnssins.F(66) = (((-(2.0 * vE)) / (gnssins.R_E + hgt)) - 2.0 * OMGE * cos(lat));
gnssins.F(79) = (vN / (gnssins.R_N + hgt));
gnssins.F(80) = ((vE / (gnssins.R_E + hgt)) + 2.0 * OMGE * cos(lat));
%// F23
gnssins.curVec3by1_1(1) = gnssins.f_ib_b(1);
gnssins.curVec3by1_1(2) = gnssins.f_ib_b(2);
gnssins.curVec3by1_1(3) = gnssins.f_ib_b(3);
gnssins.curVec3by1_2 = gnssins.C_b_n * gnssins.curVec3by1_1;
gnssins.curMat3by3_2 = skew(gnssins.curVec3by1_2,0);
gnssins.F(94) = -gnssins.curMat3by3_2(1);
gnssins.F(95) = -gnssins.curMat3by3_2(2);
gnssins.F(96) = -gnssins.curMat3by3_2(3);
gnssins.F(109) = -gnssins.curMat3by3_2(4);
gnssins.F(110) = -gnssins.curMat3by3_2(5);
gnssins.F(111) = -gnssins.curMat3by3_2(6);
gnssins.F(124) = -gnssins.curMat3by3_2(7);
gnssins.F(125) = -gnssins.curMat3by3_2(8);
gnssins.F(126) = -gnssins.curMat3by3_2(9);
%// F31 (Att Pos Error in meter)
gnssins.F(7) = ((OMGE * sin(lat)) / (gnssins.R_N + hgt));
gnssins.F(9) = (((OMGE * cos(lat)) / (gnssins.R_N + hgt)) + (vE / ((gnssins.R_N + hgt) * (gnssins.R_E + hgt) * power(cos(lat), 2.0))));
gnssins.F(37) = (-vE / power(gnssins.R_E + hgt, 2.0));
gnssins.F(38) = (vN / power(gnssins.R_N + hgt, 2.0));
gnssins.F(39) = ((vE * tan(lat)) / power(gnssins.R_E + hgt, 2));
%// F32
gnssins.F(53) = (1.0 / (gnssins.R_N + hgt));
gnssins.F(67) = (-1.0 / (gnssins.R_E + hgt));
gnssins.F(69) = (tan(lat) / (gnssins.R_E + hgt));
%// F33
gnssins.F(98) = -(-OMGE*sin(lat) - ((vE * tan(lat)) / (gnssins.R_E + hgt)));
gnssins.F(99) = -(vN / (gnssins.R_N + hgt));
gnssins.F(112) = -((OMGE*sin(lat)) + ((vE * tan(lat)) / (gnssins.R_E + hgt)));
gnssins.F(114) = -((OMGE*cos(lat)) + (vE / (gnssins.R_E + hgt)));
gnssins.F(127) = -(-vN / (gnssins.R_N + hgt));
gnssins.F(128) = -((-OMGE * cos(lat)) - (vE / (gnssins.R_E + hgt)));
%// F24 (= C_b_n)
gnssins.F(139) = gnssins.C_b_n(1);
gnssins.F(140) = gnssins.C_b_n(2);
gnssins.F(141) = gnssins.C_b_n(3);
gnssins.F(154) = gnssins.C_b_n(4);
gnssins.F(155) = gnssins.C_b_n(5);
gnssins.F(156) = gnssins.C_b_n(6);
gnssins.F(169) = gnssins.C_b_n(7);
gnssins.F(170) = gnssins.C_b_n(8);
gnssins.F(171) = gnssins.C_b_n(9);
%// F35 (= C_b_n)
gnssins.F(187) = gnssins.C_b_n(1);
gnssins.F(188) = gnssins.C_b_n(2);
gnssins.F(189) = gnssins.C_b_n(3);
gnssins.F(202) = gnssins.C_b_n(4);
gnssins.F(203) = gnssins.C_b_n(5);
gnssins.F(204) = gnssins.C_b_n(6);
gnssins.F(217) = gnssins.C_b_n(7);
gnssins.F(218) = gnssins.C_b_n(8);
gnssins.F(219) = gnssins.C_b_n(9);

gnssins.Phi = eye(15);
gnssins.curMat15by15_1 = gnssins.F * gnssins.F;
for i = 1:225
        gnssins.Phi(i) = gnssins.Phi(i) + gnssins.F(i) * dt + gnssins.curMat15by15_1(i) * dt * dt * 0.5;
end
end

