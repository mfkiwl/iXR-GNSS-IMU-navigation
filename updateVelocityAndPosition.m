function gnssins = updateVelocityAndPosition(gnssins,f_b,w_ie_n,w_en_n,dt)

RE_WGS84  =  6378137.0;           %  earth semimajor axis (WGS84) (m)
FE_WGS84  =  (1.0/298.257223563); %  earth flattening (WGS84) */
ECC_WGS84	=sqrt(FE_WGS84 * (2.0- FE_WGS84)); %/* Inverse earth flattening */
%/* earth equatorial radius, (1), eq. 2.105 */
gnssins.R_N = RE_WGS84 * (1.0 - ECC_WGS84^2) / ((1.0 - power(ECC_WGS84, 2.0)*(sin(gnssins.previousPos(1))^2.0))^(3.0 / 2.0));
%/* Earth curvature in meridional direction, (1), eq. 2.106 */
gnssins.R_E = RE_WGS84 / sqrt(1.0 - (ECC_WGS84^2)*(sin(gnssins.previousPos(1))^2.0));

gnssins.curVec3by1_1 = zeros( 3, 1);
gnssins.curVec3by1_2 = zeros( 3, 1);
gnssins.velIncrement = zeros( 3, 1);
gnssins.curMat3by3_1 = zeros( 3, 3);
gnssins.curMat3by3_2 = zeros( 3, 3);
gnssins.curMat3by3_3 = zeros( 3, 3);
%/* Determine earth rotation and transport rate effects */
for i = 1:3
    gnssins.curVec3by1_1(i) = 2.0 * w_ie_n(i);
end
gnssins.curMat3by3_1 = skew(gnssins.curVec3by1_1,  0);
gnssins.curMat3by3_2 = skew(w_en_n,  0);
for i = 1:9
        gnssins.curMat3by3_1(i) = gnssins.curMat3by3_1(i) + gnssins.curMat3by3_2(i);
end
gnssins.curVec3by1_1 =  gnssins.curMat3by3_1 * gnssins.previousVel;
%// Compute specific force (1), eq. 5.47
for i = 1:9
        gnssins.curMat3by3_3(i) = 0.5 * (gnssins.C_b_n(i) + gnssins.C_b_n_prev(i));
end
gnssins.specForce = gnssins.curMat3by3_3 * f_b;

%/* Compute velocity increment in n frame and remove earth rotation and transport rate effects (1), eq. 5.54*/
for i = 1:3
    gnssins.velIncrement(i) = gnssins.specForce(i) - gnssins.curVec3by1_1(i) + gnssins.gVector(i);
end
%/* Update velocity */
for i = 1 : 3
    gnssins.curVec3by1_2(i) = gnssins.previousVel(i) + gnssins.velIncrement(i) * dt;
end

%/* Compute updated position in n frame with trapez rule, (1), eq. 5.56 */
%/* Latitude, Longitude, Height */
gnssins.posVel(3) = gnssins.previousPos(3) - 0.5 * (gnssins.previousVel(3) + gnssins.curVec3by1_2(3)) * dt;
gnssins.posVel(1) = gnssins.previousPos(1) + ((0.5 * (gnssins.previousVel(1) + gnssins.curVec3by1_2(1))) / (gnssins.R_N + gnssins.posVel(3))) * dt;
gnssins.posVel(2) = gnssins.previousPos(2) + (0.5 * (gnssins.previousVel(2) + gnssins.curVec3by1_2(2)) / ((gnssins.R_E + gnssins.posVel(3)) * cos(gnssins.posVel(1)))) * dt;
%/* Velocity North, East, Down */
gnssins.posVel(4) = gnssins.curVec3by1_2(1);
gnssins.posVel(5) = gnssins.curVec3by1_2(2);
gnssins.posVel(6) = gnssins.curVec3by1_2(3);

gnssins.curVec3by1_1 = zeros(3,1);
gnssins.curVec3by1_2 = zeros(3,1);

end

