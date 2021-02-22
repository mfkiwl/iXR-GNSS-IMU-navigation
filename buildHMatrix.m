function gnssins = buildHMatrix(gnssins)
POSITION_FILTER_UPDATE = 0;
XYZVEL_FILTER_UPDATE = 1;
NEDVEL_FILTER_UPDATE = 2;
POSVEL_FILTER_UPDATE = 3;

if (gnssins.filterUpdateType == POSITION_FILTER_UPDATE)
    %// Calculate H_r3_n
    gnssins.curVec3by1_1 = gnssins.C_b_n * gnssins.leverArm;
    gnssins.H_r3_n = skew(gnssins.curVec3by1_1, 0);
    %// Set H Matrix for Position Update
    gnssins.H(1) = -1.0;
    gnssins.H(5) = -1.0;
    gnssins.H(9) = -1.0;
    gnssins.H(19) = gnssins.H_r3_n(1);
    gnssins.H(20) = gnssins.H_r3_n(2);
    gnssins.H(21) = gnssins.H_r3_n(3);
    gnssins.H(22) = gnssins.H_r3_n(4);
    gnssins.H(23) = gnssins.H_r3_n(5);
    gnssins.H(24) = gnssins.H_r3_n(6);
    gnssins.H(25) = gnssins.H_r3_n(7);
    gnssins.H(26) = gnssins.H_r3_n(8);
    gnssins.H(27) = gnssins.H_r3_n(9);   
end

if (gnssins.filterUpdateType == XYZVEL_FILTER_UPDATE || gnssins.filterUpdateType == NEDVEL_FILTER_UPDATE)
    
    %// Calculate H_v3_n
    gnssins.curMat3by3_1 = skew(gnssins.w_ib_b, 0);
    gnssins.curVec3by1_1 = (gnssins.curMat3by3_1 * gnssins.leverArm);
    gnssins.curVec3by1_2 = ( gnssins.C_b_n * gnssins.curVec3by1_1 );
    gnssins.curMat3by3_1 = skew(gnssins.w_ie_n,0);
    gnssins.curMat3by3_2 = (gnssins.curMat3by3_1 * gnssins.C_b_n);
    gnssins.curVec3by1_1 = (gnssins.curMat3by3_2 * gnssins.leverArm);
    for i = 1:3
        gnssins.curVec3by1_2(i) = gnssins.curVec3by1_2(i) - gnssins.curVec3by1_1(i);
    end
    gnssins.H_v3_n = skew(gnssins.curVec3by1_2, 0);
    %// Calculate H_v5_n
    gnssins.curMat3by3_1 = skew(gnssins.leverArm, 0);
    gnssins.H_v5_n = (gnssins.C_b_n * gnssins.curMat3by3_1);
    %// Set H Matrix for Velocity Update
    gnssins.H(10) = -1.0;
    gnssins.H(14) = -1.0;
    gnssins.H(15) = -1.0;
    gnssins.H(18) = gnssins.H_v3_n(1);
    gnssins.H(20) = gnssins.H_v3_n(2);
    gnssins.H(21) = gnssins.H_v3_n(3);
    gnssins.H(22) = gnssins.H_v3_n(4);
    gnssins.H(23) = gnssins.H_v3_n(5);
    gnssins.H(24) = gnssins.H_v3_n(6);
    gnssins.H(25) = gnssins.H_v3_n(7);
    gnssins.H(26) = gnssins.H_v3_n(8);
    gnssins.H(27) = gnssins.H_v3_n(9);
    gnssins.H(37) = gnssins.H_v5_n(1);
    gnssins.H(38) = gnssins.H_v5_n(2);
    gnssins.H(39) = gnssins.H_v5_n(3);
    gnssins.H(40) = gnssins.H_v5_n(4);
    gnssins.H(41) = gnssins.H_v5_n(5);
    gnssins.H(42) = gnssins.H_v5_n(6);
    gnssins.H(43) = gnssins.H_v5_n(7);
    gnssins.H(44) = gnssins.H_v5_n(8);
    gnssins.H(45) = gnssins.H_v5_n(9);   
end

if (gnssins.filterUpdateType == POSVEL_FILTER_UPDATE)
    
    %// Calculate H_r3_n
    gnssins.curVec3by1_1 = gnssins.C_b_n * gnssins.leverArm;
    gnssins.H_r3_n = skew(gnssins.curVec3by1_1,0);
    %// Calculate H_v3_n
    gnssins.curMat3by3_1 = skew(gnssins.w_ib_b, 0);
    gnssins.curVec3by1_1 = gnssins.curMat3by3_1 * gnssins.leverArm;
    gnssins.curVec3by1_2 = gnssins.C_b_n * gnssins.curVec3by1_1;
    gnssins.curMat3by3_1 = skew(gnssins.w_ie_n, 0);
    gnssins.curMat3by3_2 = gnssins.curMat3by3_1 * gnssins.C_b_n;
    gnssins.curVec3by1_1 = gnssins.curMat3by3_2 * gnssins.leverArm;
    for i = 1:3
        gnssins.curVec3by1_2(i) = gnssins.curVec3by1_2(i) - gnssins.curVec3by1_1(i);
    end
    gnssins.H_v3_n = skew(gnssins.curVec3by1_2,0);
    %// Calculate H_v5_n
    gnssins.curMat3by3_1 = skew(gnssins.leverArm, 0);
    gnssins.H_v5_n = gnssins.C_b_n * gnssins.curMat3by3_1;
    %// Set H Matrix for Position/Velocity Update
    gnssins.H(1) = -1.0;
    gnssins.H(8) = -1.0;
    gnssins.H(15) = -1.0;
    
    gnssins.H(22) = -1.0;
    gnssins.H(29) = -1.0;
    gnssins.H(36) = -1.0;
    
    gnssins.H(37) = gnssins.H_r3_n(1);
    gnssins.H(38) = gnssins.H_r3_n(2);
    gnssins.H(39) = gnssins.H_r3_n(3);
    gnssins.H(43) = gnssins.H_r3_n(4);
    gnssins.H(44) = gnssins.H_r3_n(5);
    gnssins.H(45) = gnssins.H_r3_n(6);
    gnssins.H(49) = gnssins.H_r3_n(7);
    gnssins.H(50) = gnssins.H_r3_n(8);
    gnssins.H(51) = gnssins.H_r3_n(9);
    
    gnssins.H(40) = gnssins.H_v3_n(1);
    gnssins.H(41) = gnssins.H_v3_n(2);
    gnssins.H(42) = gnssins.H_v3_n(3);
    gnssins.H(46) = gnssins.H_v3_n(4);
    gnssins.H(47) = gnssins.H_v3_n(5);
    gnssins.H(48) = gnssins.H_v3_n(6);
    gnssins.H(52) = gnssins.H_v3_n(7);
    gnssins.H(53) = gnssins.H_v3_n(8);
    gnssins.H(54) = gnssins.H_v3_n(9);
    
    gnssins.H(76) = gnssins.H_v5_n(1);
    gnssins.H(77) = gnssins.H_v5_n(2);
    gnssins.H(78) = gnssins.H_v5_n(3);
    gnssins.H(82) = gnssins.H_v5_n(4);
    gnssins.H(83) = gnssins.H_v5_n(5);
    gnssins.H(84) = gnssins.H_v5_n(6);
    gnssins.H(88) = gnssins.H_v5_n(7);
    gnssins.H(89) = gnssins.H_v5_n(8);
    gnssins.H(90) = gnssins.H_v5_n(9);
else
end

end

