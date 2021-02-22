function gnssins = updateFilter(gnssins,rtk)
SOLQ_NONE =  0 ;                  %/* solution status: no solution */
SOLQ_FIX =   1  ;                 %/* solution status: fix */
SOLQ_FLOAT = 2;                   %/* solution status: float */
POSITION_FILTER_UPDATE = 0;
XYZVEL_FILTER_UPDATE = 1;
NEDVEL_FILTER_UPDATE = 2;
POSVEL_FILTER_UPDATE = 3;
RE_WGS84  =  6378137.0;           %  earth semimajor axis (WGS84) (m)
FE_WGS84  =  (1.0/298.257223563); %  earth flattening (WGS84) */
ECC_WGS84	=sqrt(FE_WGS84 * (2.0- FE_WGS84)); %/* Inverse earth flattening */


gnssins = buildHMatrix(gnssins);
gnssins = buildRMatrix(gnssins, rtk);
gnssins = buildKMatrix(gnssins);

if (gnssins.filterUpdateType == POSITION_FILTER_UPDATE)
    %/* Prepare observation vector: Transform RTK position to LLH */
    for i = 1:3
        gnssins.curVec3by1_1(i) = rtk.sol.rr(i);
    end
    gnssins.curVec3by1_2 = ecef2pos(gnssins.curVec3by1_1);
    %/* earth equatorial radius */
    gnssins.R_N = RE_WGS84 * (1.0 - power(ECC_WGS84, 2)) / power((1.0 - power(ECC_WGS84, 2)*power(sin(gnssins.curVec3by1_2(1)), 2)), 3.0 / 2.0);
    %/* Earth curvature in meridional direction */
    gnssins.R_E = RE_WGS84 / sqrt(1.0 - power(ECC_WGS84, 2)*power(sin(gnssins.curVec3by1_2(1)), 2));
    %// z vector in meter
    gnssins.z(1) = ((-gnssins.posVel(1) + gnssins.curVec3by1_2(1)) * (gnssins.R_N + gnssins.posVel(3)));
    gnssins.z(2) = ((-gnssins.posVel(2) + gnssins.curVec3by1_2(2)) * ((gnssins.R_E + gnssins.posVel(3)) * cos(gnssins.posVel(1))));
    gnssins.z(3) = ((-gnssins.posVel(3) + gnssins.curVec3by1_2(2)) * (-1.0));
    %// Subtract lever arm
    gnssins.z =  -1.0 * gnssins.C_b_n * gnssins.leverArm + gnssins.z;
    %// Copy z vector for calculation of residuals
    gnssins.res = gnssins.z;
    %// Update error state vector
    gnssins.z = -1.0 * gnssins.H * gnssins.xaPriori + gnssins.z;
    gnssins.xaPosteriori = gnssins.xaPriori;
    gnssins.xaPosteriori = gnssins.K * gnssins.z + gnssins.xaPosteriori;
    %// Calculate and print residuals
    gnssins.res = -1.0 * gnssins.H * gnssins.xaPosteriori + gnssins.res;
    %// Set x aPriori to x aPosteriori values for next propagation step
    gnssins.xaPriori = gnssins.xaPosteriori;
    
    %// Prepare Matrices for covariance matrix update
    gnssins.curMat15by15_1= eye( 15);
    gnssins.curMat15by15_2 = zeros(15, 15);
    gnssins.curMat15by3_1 = zeros( 15, 3);
    
    %// Update covariance Matrix 3.25
    gnssins.curMat15by15_1 =  -1.0 * gnssins.K * gnssins.H + gnssins.curMat15by15_1;
    gnssins.PaPosteriori = gnssins.curMat15by15_1 * gnssins.PaPriori;
    gnssins.PaPriori = gnssins.PaPosteriori;
    
    for i = 1:15
        gnssins.curVec15by1_1(i) = gnssins.PaPosteriori(i * 16 - 15);
    end
    gnssins.curVec15by1_1 = zeros(15, 1);
    
elseif (gnssins.filterUpdateType == XYZVEL_FILTER_UPDATE || gnssins.filterUpdateType == NEDVEL_FILTER_UPDATE)
    
    %/* earth equatorial radius */
    gnssins.R_N = RE_WGS84 * (1.0 - power(ECC_WGS84, 2.0)) / power((1.0 - power(ECC_WGS84, 2.0)*power(sin(gnssins.posVel(1)), 2.0)), 3.0 / 2.0);
    %/* Earth curvature in meridional direction */
    gnssins.R_E = RE_WGS84 / sqrt(1.0 - power(ECC_WGS84, 2.0)*power(sin(gnssins.posVel(1)), 2.0));
    
    if (gnssins.filterUpdateType == XYZVEL_FILTER_UPDATE)
        
        %/* Prepare observation vector: Transform RTK position to LLH */
        gnssins.LL(1) = gnssins.posVel(1);
        gnssins.LL(2) = gnssins.posVel(2);
        for j = 1:3
            gnssins.velXYZ(j) = rtk.sol.rr(3 + j);
        end
        
        gnssins.velENU = ecef2enu(gnssins.LL, gnssins.velXYZ);
        
        gnssins.z(1) = -gnssins.posVel(4) + gnssins.velENU(2);
        gnssins.z(2) = -gnssins.posVel(5) + gnssins.velENU(1);
        gnssins.z(3) = -gnssins.posVel(6) - gnssins.velENU(3);
        
    elseif (gnssins.filterUpdateType == NEDVEL_FILTER_UPDATE)
        
        gnssins.z(1) = -gnssins.posVel(4) + rtk.sol.rr(4);
        gnssins.z(2) = -gnssins.posVel(5) + rtk.sol.rr(5);
        gnssins.z(3) = -gnssins.posVel(6) + rtk.sol.rr(6);
    end
    %// Subtract lever arm from velocity observation
    gnssins.curMat3by3_1 = skew(gnssins.w_ib_b, 0);
    gnssins.curVec3by1_1 = gnssins.curMat3by3_1 * gnssins.leverArm;
    gnssins.curVec3by1_2 = gnssins.C_b_n * gnssins.curVec3by1_1;
    gnssins.curMat3by3_1 = skew(gnssins.w_ie_n, 0);
    gnssins.curMat3by3_2 = gnssins.curMat3by3_1 * gnssins.C_b_n;
    gnssins.curVec3by1_2 = gnssins.curMat3by3_2 * gnssins.leverArm + gnssins.curVec3by1_2;
    for j = 1:3
        gnssins.z(j) = gnssins.z(j) - gnssins.curVec3by1_2(j);
    end
    gnssins.res = gnssins.z;
    %// Update error state vector
    gnssins.z = -1.0 * gnssins.H * gnssins.xaPriori + gnssins.z;
    
    gnssins.xaPosteriori = gnssins.xaPriori;
    gnssins.xaPosteriori = gnssins.K * gnssins.z + gnssins.xaPosteriori;
    
    %// Calculate Residuals
    gnssins.res =  -1.0 * gnssins.H * gnssins.xaPosteriori + gnssins.res;
    gnssins.xaPriori =  gnssins.xaPosteriori;
    
    %// Prepare Matrices for covariance matrix update
    gnssins.curMat15by15_1 = eye(15);
    gnssins.curMat15by15_2 = zeros( 15, 15);
    gnssins.curMat15by3_1 = zeros (15, 3);
    %// Update covariance Matrix regular form 3.25
    gnssins.curMat15by15_1 = -1.0 * gnssins.K * gnssins.H + gnssins.curMat15by15_1;
    gnssins.PaPosteriori = gnssins.curMat15by15_1 * gnssins.PaPriori;
    gnssins.PaPriori = gnssins.PaPosteriori;
    %// Print Variances
    for i = 1:15
        gnssins.curVec15by1_1(i) = gnssins.PaPosteriori(i * 16 - 15);
    end
    gnssins.curVec15by1_1 =zeros( 15, 1);
    %flag_updateFilter = 1;
    
elseif (gnssins.filterUpdateType == POSVEL_FILTER_UPDATE)
    
    %/* Prepare observation vector: Transform RTK position to LLH */
    for i = 1:3
        gnssins.curVec3by1_1(i) = rtk.sol.rr(i);
    end
    gnssins.curVec3by1_2 = ecef2pos(gnssins.curVec3by1_1);
    %/* earth equatorial radius */
    gnssins.R_N = RE_WGS84 * (1.0 - power(ECC_WGS84, 2)) / power((1.0 - power(ECC_WGS84, 2)*power(sin(gnssins.curVec3by1_2(1)), 2)), 3.0 / 2.0);
    %/* Earth curvature in meridional direction */
    gnssins.R_E = RE_WGS84 / sqrt(1.0 - power(ECC_WGS84, 2)*power(sin(gnssins.curVec3by1_2(1)), 2));
    %// z vector in meter
    gnssins.z(1) = ((-gnssins.posVel(1) + gnssins.curVec3by1_2(1)) * (gnssins.R_N + gnssins.posVel(3)));
    gnssins.z(2) = ((-gnssins.posVel(2) + gnssins.curVec3by1_2(2)) * ((gnssins.R_E + gnssins.posVel(3)) * cos(gnssins.posVel(1))));
    gnssins.z(3) = ((-gnssins.posVel(3) + gnssins.curVec3by1_2(3)) * (-1.0));
    %// Subtract lever arm from position observation
    gnssins.curVec3by1_1 = gnssins.C_b_n * gnssins.leverArm;
    for i =1:3
        gnssins.z(i) = gnssins.z(i) - gnssins.curVec3by1_1(i);
    end
    
    gnssins.LL(1) = gnssins.curVec3by1_2(1);
    gnssins.LL(2) = gnssins.curVec3by1_2(2);
    for j = 1:3
        gnssins.velXYZ(j) = rtk.sol.rr(3 + j);
    end
    
    gnssins.velENU = ecef2enu(gnssins.LL, gnssins.velXYZ);
    
    gnssins.z(4) = -gnssins.posVel(4) + gnssins.velENU(2);
    gnssins.z(5) = -gnssins.posVel(5) + gnssins.velENU(1);
    gnssins.z(6) = -gnssins.posVel(6) - gnssins.velENU(3);
    
    %// Subtract lever arm from velocity observation
    gnssins.curMat3by3_1 = skew(gnssins.w_ib_b, 0);
    gnssins.curVec3by1_1 = gnssins.curMat3by3_1 * gnssins.leverArm;
    gnssins.curVec3by1_2 = gnssins.C_b_n * gnssins.curVec3by1_1;
    gnssins.curMat3by3_1 = skew(gnssins.w_ie_n, 0);
    gnssins.curMat3by3_2 = gnssins.curMat3by3_1 * gnssins.C_b_n;
    gnssins.curVec3by1_2 = gnssins.curMat3by3_2 * gnssins.leverArm + gnssins.curVec3by1_2;
    for j = 1:3
        gnssins.z(j + 3) = gnssins.z(j + 3) - gnssins.curVec3by1_2(j);
    end
    %// Copy z vector for calculation of residuals
    gnssins.res = gnssins.z;
    %// Update error state vector
    gnssins.z = -1.0 * gnssins.H * gnssins.xaPriori + gnssins.z;
    gnssins.xaPosteriori = gnssins.xaPriori;
    gnssins.xaPosteriori = gnssins.K * gnssins.z + gnssins.xaPosteriori;
    
    %// Calculate and print residuals
    gnssins.res = -1.0 * gnssins.H * gnssins.xaPosteriori + gnssins.res;
    %// Set x aPriori to x aPosteriori values for next propagation step
    gnssins.xaPriori = gnssins.xaPosteriori;
    
    %// Prepare Matrices for covariance matrix update
    gnssins.curMat15by15_1 =  eye(15);
    gnssins.curMat15by15_2 = zeros( 15, 15);
    gnssins.curMat15by6_1 = zeros( 15, 6);
    
    %// Update covariance Matrix
    gnssins.curMat15by15_1 = -1.0 * gnssins.K * gnssins.H + gnssins.curMat15by15_1;
    gnssins.PaPosteriori = gnssins.curMat15by15_1 * gnssins.PaPriori;
    gnssins.PaPriori =  gnssins.PaPosteriori;
    %// Print Variances
    for i= 1:15
        gnssins.curVec15by1_1(i) = gnssins.PaPosteriori(i * 16 - 15);
    end
    gnssins.curVec15by1_1 = zeros( 15, 1);
    %flag_updateFilter = 1;
else
    %flag_updateFilter = 0;
end
end
