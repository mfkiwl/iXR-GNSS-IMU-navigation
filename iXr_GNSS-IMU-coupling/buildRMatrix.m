function gnssins = buildRMatrix(gnssins,rtk)
SOLQ_NONE =  0 ;                  %/* solution status: no solution */
SOLQ_FIX =   1  ;                 %/* solution status: fix */
SOLQ_FLOAT = 2;                   %/* solution status: float */
POSITION_FILTER_UPDATE = 0;
XYZVEL_FILTER_UPDATE = 1;
NEDVEL_FILTER_UPDATE = 2;
POSVEL_FILTER_UPDATE = 3;

if (gnssins.filterUpdateType == POSITION_FILTER_UPDATE)
    gnssins.R = zeros(3,3);
    if (rtk.sol.stat == SOLQ_FIX)
        gnssins.R(1) = 0.0001;
        gnssins.R(5) = 0.0001;
        gnssins.R(9) = 0.0001;
    else
        gnssins.R(1) = 1;
        gnssins.R(5) = 1;
        gnssins.R(9) = 1;
    end

elseif (gnssins.filterUpdateType == XYZVEL_FILTER_UPDATE || gnssins.filterUpdateType == NEDVEL_FILTER_UPDATE)
    
    gnssins.R(1) = 0.00001;
    gnssins.R(5) = 0.00001;
    gnssins.R(9) = 0.00001;

elseif (gnssins.filterUpdateType == POSVEL_FILTER_UPDATE)
    
    gnssins.R = zeros( 6, 6);
    
    if (rtk.sol.stat == SOLQ_FIX)
        gnssins.R(1) = 0.0001;
        gnssins.R(8) = 0.0001;
        gnssins.R(15) = 0.0001;
        gnssins.R(22) = 0.0001;
        gnssins.R(29) = 0.0001;
        gnssins.R(36) = 0.0001;
    else
        gnssins.R(1) = 1;
        gnssins.R(8) = 1;
        gnssins.R(15) = 1;
        gnssins.R(22) = 1;
        gnssins.R(29) = 1;
        gnssins.R(36) = 1;
    end
else
end
end

