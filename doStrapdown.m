function gnssins = doStrapdown(imusingal,gnssins,dt)

%%/* Remove Biases from sensor measurements */
for i = 1:3
    gnssins.f_ib_b(i) = imusingal(gnssins.numacc + i - 1) + gnssins.accBias(i);
    gnssins.w_ib_b(i) = imusingal(gnssins.numgyro + i - 1) + gnssins.gyrBias(i);
end
%/* Compute earth rotation */
 gnssins.w_ie_n = earthRotationVector(gnssins.previousPos(1));

%/* Compute Transport rates */
gnssins.w_en_n = transportRates(gnssins.previousVel(1), gnssins.previousVel(2), gnssins.previousPos(1),gnssins.previousPos(3));

%/* Compute normal gravity at current position */
gnssins.gVector = normalGravity(gnssins.previousPos(1), gnssins.previousPos(3));
%/* Update Attitude */
gnssins = updateAttitude(gnssins, dt);
%/* Update velocity and position */
gnssins = updateVelocityAndPosition(gnssins, gnssins.f_ib_b, gnssins.w_ie_n, gnssins.w_en_n, dt);

%// Calculate Euler angles (mainly for debug output/plotting of results)
gnssins.attEuler(1) = atan2(gnssins.C_b_n(6), gnssins.C_b_n(9));
gnssins.attEuler(2) = asin(-gnssins.C_b_n(3));
gnssins.attEuler(3) = atan2(gnssins.C_b_n(2), gnssins.C_b_n(1));

for i = 1:3
    gnssins.previousPos(i) = gnssins.posVel(i);
    gnssins.previousVel(i) = gnssins.posVel(3 + i);
    gnssins.gnss_ins_sol.rr(i) = gnssins.posVel(i);
    gnssins.gnss_ins_sol.rr(3 + i) = gnssins.posVel(3 + i);
    gnssins.gnss_ins_sol.att(i) = gnssins.attEuler(i);
end
end

