function [index] = findImuMeas(rtk,imudat,m,gnssins, imudatt)
imuMeasInterval = (1.0 / gnssins.imuMeasRate) /2.0;
index = 1;
gnssCompTime = rtk.sol.time;
for i = 1:m
    imuCompTime = imudat(imudatt + i -1,1);
    if (( (gnssCompTime - imuMeasInterval) <= imuCompTime) && ( (gnssCompTime + imuMeasInterval) >= imuCompTime))       
        index = i;      
    end
end
end

