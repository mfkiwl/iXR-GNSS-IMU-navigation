function gnssins = updateAttitude(gnssins,dt)
%/* Both attitude update algorithms are implemented according to (1),
%see chapters 5.5.1 for the precision DCM attitude update and chapter
%E.6.2 in the appendix for the precision quaternion update */
ATTITUDE_UPDATE_EULER = 0;
ATTITUDE_UPDATE_QUATERNION = 1;

gnssins.curVec3by1_1 = zeros(3,1);
gnssins.curMat3by3_1 = zeros(3,3);
gnssins.curMat3by3_2 = zeros(3,3);
gnssins.curMat3by3_3 = zeros(3,3);
gnssins.curVec4by1_1 = zeros(4,1);
gnssins.curMat4by4_1 = zeros(4,4);
gnssins.curMat4by4_2 = zeros(4,4);
gnssins.C_b_n_prev = gnssins.C_b_n ;

%/* Precision Euler update */
if gnssins.attitudeUpdateType == ATTITUDE_UPDATE_EULER
    
    gnssins.C_b_b = eye(3);
    %/* Alpha_ib_b, aka attitude increment */
    for i = 1:3
        gnssins.alpha_ib_b(i) = gnssins.w_ib_b(i) * dt;
    end
    %/* Magnitude Alpha_ib_b */
    dMagAlpha = sqrt(power(gnssins.alpha_ib_b(1), 2.0) + power(gnssins.alpha_ib_b(2), 2.0) + power(gnssins.alpha_ib_b(3), 2.0));
    %/* skew alpha */
    gnssins.curMat3by3_1 = skew(gnssins.alpha_ib_b, 0);
    
    if dMagAlpha > 1.0E-8
        
        for i = 1:9
                gnssins.curMat3by3_2(i) = (sin(dMagAlpha) / dMagAlpha) * gnssins.curMat3by3_1(i);
        end
        dFactor = (1.0 - cos(dMagAlpha)) / power(dMagAlpha, 2.0);
        gnssins.curMat3by3_3 = (dFactor .* gnssins.curMat3by3_1) * gnssins.curMat3by3_1;
        for i = 1:9
                gnssins.C_b_b(i) = gnssins.C_b_b(i) + gnssins.curMat3by3_2(i) + gnssins.curMat3by3_3(i);
        end
        
    else
        for i = 1:3
            for j = 1:3
                gnssins.C_b_b(i,j) = gnssins.C_b_b(i,j) + gnssins.curMat3by3_1(i,j);
            end
        end
    end
    
    gnssins.curMat3by3_1 = zeros(3,3);
    gnssins.curMat3by3_2 =  eye(3);
    gnssins.curMat3by3_3 = zeros(3,3);
    
    %/* Transport rates and earth rotation */
    for i = 1:3
        gnssins.curVec3by1_1(i) = (gnssins.w_en_n(i) + gnssins.w_ie_n(i)) * dt;
    end
    gnssins.curMat3by3_1 = skew(gnssins.curVec3by1_1, 0);
    for i = 1:9
            gnssins.curMat3by3_2(i) = gnssins.curMat3by3_2(i) - gnssins.curMat3by3_1(i);
    end
    gnssins.curMat3by3_3 = gnssins.curMat3by3_2 * gnssins.C_b_n;
    %/* Set new attitude */
    gnssins.C_b_n = gnssins.curMat3by3_3 * gnssins.C_b_b;
end
%// End of DCM Implementation/ Precision Euler Update

%// Precision Quaternion Update
if gnssins.attitudeUpdateType == ATTITUDE_UPDATE_QUATERNION
    
    %/* Alpha_ib_b, aka attitude increment */
    for i = 1:3
        gnssins.alpha_ib_b(i) = gnssins.w_ib_b(i) * dt;
    end
    %/* Magnitude Alpha_ib_b */
    dMagAlpha = sqrt(power(gnssins.alpha_ib_b(1), 2.0) + power(gnssins.alpha_ib_b(2), 2.0) + power(gnssins.alpha_ib_b(3), 2.0));
    
    gnssins.q_b_b(1) = cos(dMagAlpha / 2.0);
    gnssins.q_b_b(2) = (sin(dMagAlpha / 2.0) / dMagAlpha) * gnssins.alpha_ib_b(1);
    gnssins.q_b_b(3) = (sin(dMagAlpha / 2.0) / dMagAlpha) * gnssins.alpha_ib_b(2);
    gnssins.q_b_b(4) = (sin(dMagAlpha / 2.0) / dMagAlpha) * gnssins.alpha_ib_b(3);
    
    for i = 1:3
        gnssins.curVec4by1_1(1+i) = 0.5*(gnssins.w_en_n(i) + gnssins.w_ie_n(i)) * dt;
    end
    
    gnssins.curMat4by4_1 = skew(gnssins.attQuat,  2);
    gnssins.curMat4by4_2 = skew(gnssins.curVec4by1_1,  2);
    
    %/* Transport rates and earth rotation */
    gnssins.curVec4by1_1 = -1.0 * gnssins.curMat4by4_2 * gnssins.attQuat;
    gnssins.attQuat = gnssins.curVec4by1_1;
    %/* Attitude increment */
    gnssins.attQuat = gnssins.curMat4by4_1 * gnssins.q_b_b + gnssins.attQuat;
    
    %//Normalize Quaternion
    qnorm = 0.0;
    for k = 1:4
        qnorm = qnorm + power(gnssins.attQuat(k), 2.0);
    end
    qnorm = sqrt(qnorm);
    %/* Set new attitude */
    for m = 1:4
        gnssins.attQuat(m) = gnssins.attQuat(m) / qnorm;
    end
    
    %//Set C_b_n from quaternion
    gnssins.C_b_n = quat2dcm(gnssins.attQuat);
       
    %// End of Quaternion Implementation / Precision Quaternion Update
    gnssins.curMat3by3_1 = zero(3,3);
    gnssins.curMat3by3_2 = zero(3,3);
    gnssins.curMat3by3_3 = zero(3,3);
    gnssins.curVec3by1_1 = zero(3,1);
    gnssins.curVec3by1_1 = zero(4,4);
    gnssins.curVec4by1_1 = zero(4,1);

end
end
