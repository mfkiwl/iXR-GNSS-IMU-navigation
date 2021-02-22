function gnssins = buildKMatrix(gnssins)
POSITION_FILTER_UPDATE = 0;
XYZVEL_FILTER_UPDATE = 1;
NEDVEL_FILTER_UPDATE = 2;
POSVEL_FILTER_UPDATE = 3;
if (gnssins.filterUpdateType == POSITION_FILTER_UPDATE || gnssins.filterUpdateType == XYZVEL_FILTER_UPDATE || gnssins.filterUpdateType == NEDVEL_FILTER_UPDATE)
    
    gnssins.curMat3by15_1 = zeros( 3, 15);
    gnssins.curMat15by3_1 = zeros( 15, 3);
    gnssins.curMat3by3_1 = zeros( 3, 3);
    gnssins.curMat3by3_1 = gnssins.H * gnssins.PaPriori * (gnssins.H)' + gnssins.R;
    gnssins.curMat3by3_1 = inv(gnssins.curMat3by3_1);
    gnssins.K = gnssins.PaPriori * (gnssins.H)' * gnssins.curMat3by3_1;
    %flag_K = 1;
end
if (gnssins.filterUpdateType == POSVEL_FILTER_UPDATE)
    
    gnssins.curMat6by15_1 = zeros( 6, 15);
    gnssins.curMat15by6_1 = zeros( 15, 6);
    gnssins.curMat6by6_1 = zeros( 6, 6);
    gnssins.curMat6by15_1 = gnssins.H * gnssins.PaPriori;
    gnssins.curMat6by6_1 = gnssins.R;
    gnssins.curMat6by6_1 = gnssins.curMat6by15_1 * (gnssins.H)' + gnssins.curMat6by6_1;
    gnssins.curMat6by6_1 = inv(gnssins.curMat6by6_1);
    gnssins.curMat15by6_1 = gnssins.PaPriori * (gnssins.H)';
    gnssins.K = gnssins.curMat15by6_1 * gnssins.curMat6by6_1;
    %flag_K = 1;  
else
    %flag_K = 0;
        
end
end

