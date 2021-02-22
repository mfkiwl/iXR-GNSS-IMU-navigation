function gnssins = doFeedback(gnssins)
ATTITUDE_UPDATE_EULER = 0;
ATTITUDE_UPDATE_QUATERNION = 1;
%// Position error feedback in meter
gnssins.previousPos(3) = gnssins.posVel(3) - gnssins.xaPosteriori(3) * (-1.0);
gnssins.posVel(3) = gnssins.previousPos(3);
gnssins.previousPos(1) = gnssins.posVel(1) - gnssins.xaPosteriori(1) / (gnssins.R_N + gnssins.posVel(3));
gnssins.posVel(1) = gnssins.previousPos(1);
gnssins.previousPos(2) = gnssins.posVel(2) - gnssins.xaPosteriori(2) / ((gnssins.R_E + gnssins.posVel(3)) * cos(gnssins.posVel(1)));
gnssins.posVel(2) = gnssins.previousPos(2);

gnssins.previousVel(1) =  gnssins.posVel(4)- gnssins.xaPosteriori(4);
gnssins.posVel(4) = gnssins.previousVel(1);
gnssins.previousVel(2) = gnssins.posVel(5) - gnssins.xaPosteriori(5);
gnssins.posVel(5) = gnssins.previousVel(2);
gnssins.previousVel(3) = gnssins.posVel(6) - gnssins.xaPosteriori(6);
gnssins.posVel(6) = gnssins.previousVel(3);

%// Feed back Attitude error, depending on representation type
if (gnssins.attitudeUpdateType == ATTITUDE_UPDATE_EULER)
    
    gnssins.curMat3by3_1 = zeros( 3, 3);
    gnssins.curMat3by3_2 = zeros( 3, 3);
    
    gnssins.curMat3by3_1 = body2navFrame(gnssins.xaPosteriori(7), gnssins.xaPosteriori(8), gnssins.xaPosteriori(9));
    gnssins.curMat3by3_2 = gnssins.C_b_n;
    gnssins.C_b_n = (gnssins.curMat3by3_1)' * gnssins.curMat3by3_2;
    
    gnssins.curMat3by3_1 = zeros( 3, 3);
    gnssins.curMat3by3_2 = zeros( 3, 3);
end

if (gnssins.attitudeUpdateType == ATTITUDE_UPDATE_QUATERNION)
    gnssins.curVec4by1_1(1) = 1.0;
    for i = 1:3
        gnssins.curVec4by1_1(i + 1) = (-0.5)*gnssins.xaPosteriori(i + 6);
    end
    gnssins.curMat4by4_1 = skew(gnssins.curVec4by1_1, 2);
    gnssins.curVec4by1_1 = gnssins.attQuat;
    gnssins.attQuat = gnssins.curMat4by4_1 * gnssins.curVec4by1_1;
    %//Normalize Quaternion
    qnorm = 0.0;
    for k = 1:4
        qnorm = qnorm +power(gnssins.attQuat(k), 2.0);
    end
    qnorm = sqrt(qnorm);
    for m = 1:4
        gnssins.attQuat(m) = gnssins.attQuat(m) / qnorm;
    end
    %//Set C_b_n from quaternion
    gnssins.C_b_n = quat2dcm(gnssins.attQuat);
end

gnssins.accBias(1) = gnssins.accBias(1) -gnssins.xaPosteriori(10);
gnssins.accBias(2) = gnssins.accBias(2) -gnssins.xaPosteriori(11);
gnssins.accBias(3) = gnssins.accBias(3) -gnssins.xaPosteriori(12);

gnssins.gyrBias(1) = gnssins.gyrBias(1) -gnssins.xaPosteriori(13);
gnssins.gyrBias(2) = gnssins.gyrBias(2) -gnssins.xaPosteriori(14);
gnssins.gyrBias(3) = gnssins.gyrBias(3) -gnssins.xaPosteriori(15);

gnssins.curVec3by1_1 = zeros( 3, 1);
gnssins.curVec4by1_1 = zeros( 4, 1);
gnssins.curVec4by1_2 = zeros( 4, 1);
gnssins.curMat3by3_1 = zeros( 3, 3);
gnssins.curMat4by4_1 = zeros( 4, 4);
gnssins.xaPosteriori = zeros( 15, 1);
gnssins.xaPriori = zeros( 15, 1);
end

