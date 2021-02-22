clc,clear;
%% Constants initial
global gnssins;
deg_to_rad = pi/180;
rad_to_deg = 1/deg_to_rad;

POSITION_FILTER_UPDATE = 0;
XYZVEL_FILTER_UPDATE = 1;
NEDVEL_FILTER_UPDATE = 2;
POSVEL_FILTER_UPDATE = 3;
ATTITUDE_UPDATE_EULER = 0;		   %/* IMU Attitude update via Euler angles */
ATTITUDE_UPDATE_QUATERNION = 1;	   %/* IMU Attitude update via quaternions */;

SET_IMU_LEVELING = 0;				%/* Set initial Roll and Pitch to zero */
CALC_IMU_LEVELING = 1	;			%/* Calculate Roll and Pitch from accelerometers */
ALMODE_INS_STATIC = 0	;			%/* IMU Alignment mode: static (average over several epochs) */
ALMODE_INS_KINEMA = 1	;			%/* IMU Alignment mode: kinematic (only leveling done, heading acquired by GNSS)*/

SOLQ_NONE =  0 ;                  %/* solution status: no solution */
SOLQ_FIX =   1  ;                 %/* solution status: fix */
SOLQ_FLOAT = 2;                   %/* solution status: float */
SOLQ_SBAS =  3;                   %/* solution status: SBAS */
SOLQ_DGPS =  4;                   %/* solution status: DGPS/DGNSS */
SOLQ_SINGLE = 5;                   %/* solution status: single */
SOLQ_PPP =   6;                   %/* solution status: PPP */
SOLQ_DR  =   7;                  % /* solution status: dead reconing */
SOLQ_STRAP = 8;					%/* solution status: Strapdown */ // Added by Yize Zhang, for GNSS/INS
SOLQ_LOOSE = 9;					%/* solution status: GNSS/INS Kalman updated*/
MAXSOLQ =    9;                  % /* max number of solution status */

gnssins.imuIsAligned = 0;
gnssins.attitudeUpdateType = 0;
gnssins.filterUpdateType = 0;
gnssins.filterUpdateType = POSVEL_FILTER_UPDATE;
gnssins.imuLevelingType = SET_IMU_LEVELING;
%gnssins.imuLevelingType = CALC_IMU_LEVELING;
gnssins.alignmentType = ALMODE_INS_KINEMA;
gnssins.imuMeasRate = 0;
gnssins.gnss_ins_sol.stat = 1;
gnssins.accBias = zeros(3, 1);
gnssins.gyrBias = zeros(3, 1);
gnssins.posXYZ = zeros(3, 1);
gnssins.velCov = zeros(3, 1);
gnssins.LL = zeros(2, 1);
gnssins.velENU = zeros(3, 1);
gnssins.velXYZ = zeros(3, 1);
gnssins.R_N = 0.0;
gnssins.R_E = 0.0;
gnssins.leverArm = zeros(3, 1);
gnssins.lastFilterUpdateTime = 0.0;
gnssins.lastImuTime = 0.0;

%/* Allocate Strapdown matrices */
gnssins.previousPos = zeros(3, 1);
gnssins.previousVel = zeros(3, 1);
gnssins.attQuat = zeros(4, 1);
gnssins.attEuler = zeros(3, 1);
gnssins.w_ie_n = zeros(3, 1);
gnssins.C_b_n = zeros(3, 3);
gnssins.C_b_n_prev = zeros(3, 3);
gnssins.w_en_n = zeros(3, 1);
gnssins.alpha_ib_b = zeros(3, 1);
gnssins.w_ib_b = zeros(3, 1);
gnssins.gVector = zeros(3, 1);
gnssins.specForce = zeros(3, 1);
gnssins.velIncrement = zeros(3, 1);
gnssins.C_b_b = eye(3);
gnssins.q_b_b = zeros(4, 1);
gnssins.f_ib_b = zeros(3, 1);
gnssins.posVel = zeros(6, 1);

%/* Allocate filter matrices */
gnssins.xaPriori = zeros(15, 1);
gnssins.xaPosteriori = zeros(15, 1);
gnssins.F = zeros(15, 15);
gnssins.Phi = eye(15);
gnssins.PaPriori = eye(15);
gnssins.H_r3_n = zeros(3, 3);
gnssins.H_v3_n = zeros(3, 3);
gnssins.H_v5_n = zeros(3, 3);

gnssins.PaPriori(15,15) = 0.1;
gnssins.PaPriori(14,14) = 0.1;
gnssins.PaPriori(13,13) = 0.1;
gnssins.PaPriori(12,12) = 0.1;
gnssins.PaPriori(11,11) = 0.1;
gnssins.PaPriori(10,10) = 0.1;
gnssins.PaPriori(9,9) = 0.1;
gnssins.PaPriori(8,8) = 0.1;
gnssins.PaPriori(7,7) = 0.1;
gnssins.PaPriori(6,6) = 10.0;
gnssins.PaPriori(5,5) = 10.0;
gnssins.PaPriori(4,4) = 10.0;
gnssins.PaPriori(3,3) = 10.0;
gnssins.PaPriori(2,2) = 10.0;
gnssins.PaPriori(1,1) = 10.0;
gnssins.PaPosteriori = zeros(15, 15);
gnssins.Q = eye(15,15);

% Allocate temporary Matrices
gnssins.curMat15by15_1 = zeros(15, 15);
gnssins.curMat15by15_2 = zeros(15, 15);
gnssins.curMat6by15_1 = zeros(6, 15);
gnssins.curMat15by6_1 = zeros(15, 6);
gnssins.curMat6by6_1 = zeros(6, 6);
gnssins.curMat3by3_1 = zeros(3, 3);
gnssins.curMat3by3_2 = zeros(3, 3);
gnssins.curMat3by3_3 = zeros(3, 3);
gnssins.curMat4by4_1 = zeros(4, 4);
gnssins.curMat4by4_2 = zeros(4, 4);
gnssins.curVec3by1_1 = zeros(3, 1);
gnssins.curVec3by1_2 = zeros(3, 1);
gnssins.curVec4by1_1 = zeros(4, 1);
gnssins.curVec4by1_2 = zeros(4, 1);
gnssins.curVec6by1_1 = zeros(6, 1);
gnssins.curVec15by1_1 = zeros(15, 1);
gnssins.curMat15by3_1 = zeros(15, 3);
gnssins.curMat3by15_1 = zeros(3, 15);
%% Input
n  = 0; m = 0;
% Input format
% GNSS: Time1, State2, Pos3~5, Vel6~8
% IMU(ENU): Time1, Acc2~4, Gyro5~7
GNSSFile = "rtk0914.csv"; %File name
IMUFile = "imu0914.csv";
GNSSinputposformat = "pos"; % pos input format pos(BLH deg/deg/m) or ecef(m)
LCputposformat = "pos"; % pos output format pos or ecef
gnssins.leverArm = gnssins.leverArm; % Offset between IMU and GNSS antenna (in NED)
gnssins.filterUpdateType = POSITION_FILTER_UPDATE; % Update style POSITION_FILTER_UPDATE or POSVEL_FILTER_UPDATE

if exist(IMUFile,'file') ~= 0
    imudat = csvread(IMUFile);
    gnssins.imuMeasRate = 100;%IMU freq
    gnssins.numacc = 2;%Number of acc columns 2~4
    gnssins.numgyro = 5;%Number of gyro columns 5~7
    gnssins.Q = gnssins.Q .* 0.003;%MEMS noise
end
if exist(GNSSFile,'file') ~= 0
    Posdata = csvread(GNSSFile);
    ECEFdata = zeros(length(Posdata)-1,8);
    n = gnssins.imuMeasRate;
    GNSSfreq = 1; % GNSS freq
    if GNSSinputposformat == "pos"
        for j = 2:length(Posdata)
            ECEFdata(j,1) = Posdata(j,1);
            ECEFdata(j,2) = Posdata(j,2);
            ECEFdata(j,3:4) = Posdata(j,3:4) .* deg_to_rad;
            ECEFdata(j,5) = Posdata(j,5);
            ECEFdata(j,3:5) = pos2ecef(ECEFdata(j,3:5));
            %ECEFdata(j,6:8) = Posdata(j,6:8);
            ECEFdata(j,6:8) = (ECEFdata(j,3:5) - ECEFdata(j-1,3:5)) ./(ECEFdata(j,1) - ECEFdata(j-1,1)); %if you don;t have speed
        end
        ECEFdata = ECEFdata(3:end,:);
    else
        ECEFdata = Posdata(:,1:8);
    end
end
%% Progress bar
dots = '....................';
bars = '||||||||||||||||||||';
rewind = '\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b\b';
fprintf(strcat('Processing: ',dots));
progress_mark = 0;
progress_epoch = 0;
%% Main loop
Result = zeros(length(ECEFdata),8);
%Setting for Kalman filter
if (gnssins.filterUpdateType == POSITION_FILTER_UPDATE || gnssins.filterUpdateType == XYZVEL_FILTER_UPDATE || gnssins.filterUpdateType == NEDVEL_FILTER_UPDATE)
    gnssins.H = zeros(3, 15);
    gnssins.K = zeros(15, 3);
    gnssins.res = zeros(3, 1);
    gnssins.R = zeros(3, 3);
    gnssins.z = zeros(3, 1);
end
if (gnssins.filterUpdateType == POSVEL_FILTER_UPDATE)
    gnssins.H = zeros(6, 15);
    gnssins.K = zeros(15, 6);
    gnssins.res = zeros(6, 1);
    gnssins.R = zeros(6, 6);
    gnssins.z = zeros(6, 1);
end

for GNSSepoch = 1:length(ECEFdata)
    if (GNSSepoch - progress_epoch) > (length(ECEFdata)/20)
        progress_mark = progress_mark + 1;
        progress_epoch = GNSSepoch;
        fprintf(strcat(rewind,bars(1:progress_mark),dots(1:(20 - progress_mark))));
    end
    m = 0;
    indexOfFirstImuRec = 0;
    
    rtk.sol.time = ECEFdata(GNSSepoch,1);
    rtk.sol.stat = ECEFdata(GNSSepoch,2);
    rtk.sol.rr = ECEFdata(GNSSepoch,3:8);
    
    for IMUepoch = 1:length(imudat)
        if (abs(ECEFdata(GNSSepoch,1) - imudat(IMUepoch,1)) <= 0.1)
            if (m == 0)
                imudatt = IMUepoch;
            end
            m = m + 1;
        elseif (imudat(IMUepoch,1) - rtk.sol.time > 0.1)
            break;
        end
    end
    %% /* If GNSS measurements only are available, do rtkpos and return */
    if ((n > 0) && (m == 0))
        Result(GNSSepoch,1) = rtk.sol.time;
        Result(GNSSepoch,2) = rtk.sol.stat;
        if LCputposformat == "ecef"
            Result(GNSSepoch,3:8) = rtk.sol.rr;
        else
            Result(GNSSepoch,3:5) = ecef2pos(rtk.sol.rr);
            Result(GNSSepoch,6:8) = rtk.sol.rr(:,4:6);
        end
    end
    %% /* Since IMU measurements are available, check if the IMU is aligned; if not, do alignment on successful rtk fixed solution */
    if (gnssins.imuIsAligned < 1)
        % 		if (gnssins.alignmentType == ALMODE_INS_STATIC)
        % 			%// Not yet implemented
        % 		end;
        if (gnssins.alignmentType == ALMODE_INS_KINEMA)
            if ((n > 0) && (m > 0))
                if (rtk.sol.stat == SOLQ_FIX)
                    %/* Convert XYZ to LLH */
                    for i = 1:3
                        gnssins.posXYZ(i) = rtk.sol.rr(i);
                        gnssins.velXYZ(i) = rtk.sol.rr(i + 3);
                        %gnssins.velCov(i) = rtk.sol.qv(i); 
                    end
                    gnssins.previousPos = ecef2pos(gnssins.posXYZ);
                    gnssins.LL(1) = gnssins.previousPos(1);
                    gnssins.LL(2) = gnssins.previousPos(2);
                    gnssins.velENU = ecef2enu(gnssins.LL, gnssins.velXYZ);
                    totalVel = sqrt(power(gnssins.velENU(1), 2.0) + power(gnssins.velENU(2), 2.0) + power(gnssins.velENU(3), 2.0));
                    %totalVelAccuracy = sqrt(gnssins.velCov(1) + gnssins.velCov(2) + gnssins.velCov(3));%
                    %/* Determine if vehicle is stationary or moving. These thresholds are an empiric assumption. */
                    %if ((totalVel > 4.0 * totalVelAccuracy) && (totalVelAccuracy < 0.3) && (totalVel > 3.0))%
                    if ((totalVel > 3.0))
                        %indexOfFirstImuRec = findImuMeas(obs.time, imudat, m);
                        indexOfFirstImuRec = findImuMeas(rtk, imudat,m,gnssins, imudatt);
                        %/* Alignment Phase */
                        %/* Levelling */
                        gnssins.gVector =normalGravity(gnssins.previousPos(1), gnssins.previousPos(3));
                        gamma = gnssins.gVector(3);
                        if (gnssins.imuLevelingType == CALC_IMU_LEVELING)
                            gnssins.attEuler(2) = asin((imudat(indexOfFirstImuRec ,gnssins.numacc) - gnssins.accBias(1)) / gamma);
                            gnssins.attEuler(1) = atan2(imudat(indexOfFirstImuRec ,gnssins.numacc + 1) - gnssins.accBias(2), imudat(indexOfFirstImuRec ,gnssins.numacc + 2) - gnssins.accBias(3));
                        elseif (gnssins.imuLevelingType == SET_IMU_LEVELING)
                            gnssins.attEuler(1) = 0.0;
                            gnssins.attEuler(2) = 0.0;
                        end
                        %/* Calculate Initial Heading from GNSS velocities (procedure for MEMS IMUs), according to (2) */
                        gnssins.attEuler(3) = atan2(gnssins.velENU(1), gnssins.velENU(2));
                        %/* Set initial position and velocity */
                        gnssins.previousVel(1) = gnssins.velENU(2);
                        gnssins.previousVel(2) = gnssins.velENU(1);
                        gnssins.previousVel(3) = -gnssins.velENU(3);
                        gnssins.imuIsAligned = 2;
                        %/* Initialize C_b_n from Euler angles */
                        gnssins.C_b_n = body2navFrame(gnssins.attEuler(1), gnssins.attEuler(2), gnssins.attEuler(3));
                        %/* Initialize quaternion from euler angles */
                        gnssins.attQuat = eul2quat(gnssins.attEuler);
                        %/* Initial position and attitude is determined, filter is initialized, propagate filter and states to next epoch */
                        for i = (indexOfFirstImuRec:m)
                            gnssins = doStrapdown(imudat(i,:), gnssins, 1.0/gnssins.imuMeasRate);
                            gnssins = propagateFilter(gnssins, 1.0 / gnssins.imuMeasRate);
                        end
                    end
                end
            end
        end
    end
    %% /* If no GNSS measurements are available, perform only strapdown computation and propagate filter only */
    % 	if ((n == 0) && (m > 0) && gnssins.imuIsAligned > 0)
    % 	{
    % 		double printImuTime;
    %
    % 		for (int i = 0; i < m; i++)
    % 		{
    % 			double imuTime = imudat(i).time.time + imudat(i).time.sec;
    % 			doStrapdown(&imudat(i), gnssins, imuTime - gnssins.lastImuTime);
    % 			propagateFilter(gnssins, imuTime - gnssins.lastImuTime);
    % 			gnssins.lastImuTime = imudat(i).time.time + imudat(i).time.sec;
    % 			%/* Store results */
    % 			for (int j = 0; j < 3; j++)
    % 			{
    % 				gnssins.gnss_ins_sol.att(j) = gnssins.attEuler(j);
    % 				gnssins.gnss_ins_sol.imuBias(j) = gnssins.accBias(j);
    % 				gnssins.gnss_ins_sol.imuBias(j + 3) = gnssins.gyrBias(j);
    % 				gnssins.gnss_ins_sol.rr(j) = gnssins.posVel(j);
    % 				gnssins.gnss_ins_sol.rr(j + 3) = gnssins.posVel(j + 3);
    % 				gnssins.gnss_ins_sol.qr(j) = gnssins.PaPosteriori(j * 16);
    % 				gnssins.gnss_ins_sol.qv(j) = gnssins.PaPosteriori((j + 3) * 16);
    % 			}
    % 			gnssins.gnss_ins_sol.qr(3) = gnssins.PaPosteriori(15);
    % 			gnssins.gnss_ins_sol.qr(4) = gnssins.PaPosteriori(31);
    % 			gnssins.gnss_ins_sol.qr(5) = gnssins.PaPosteriori(30);
    % 			gnssins.gnss_ins_sol.qv(3) = gnssins.PaPosteriori(49);
    % 			gnssins.gnss_ins_sol.qv(4) = gnssins.PaPosteriori(65);
    % 			gnssins.gnss_ins_sol.qv(5) = gnssins.PaPosteriori(50);
    % 			gnssins.gnss_ins_sol.time.time = imudat.time.time;
    % 			gnssins.gnss_ins_sol.time.sec = imudat.time.sec;
    % 			gnssins.gnss_ins_sol.stat = SOLQ_STRAP;
    %
    % 			if (i == m / 2)
    % 			{
    % 				printImuTime = imuTime;
    % 			}
    % 		}
    % 		for (int j = 0; j < 3; j++)
    % 		{
    % 			gnssins.curVec15by1_1(j) = gnssins.posVel(j);
    % 			gnssins.curVec15by1_1(3 + j) = gnssins.posVel(j + 3);
    % 			gnssins.curVec15by1_1(6 + j) = gnssins.attEuler(j) / PI * 180.0;
    % 			gnssins.curVec15by1_1(9 + j) = gnssins.accBias(j);
    % 			gnssins.curVec15by1_1(12 + j) = gnssins.gyrBias(j);
    % 		}
    % 		fprintf(gnssins.filterTotalStates, "%*.*f ", 10, 7, printImuTime);
    % 		matfprint(gnssins.curVec15by1_1, 1, 15, 12, 8, gnssins.filterTotalStates);
    %
    % 		return 2;
    % 	}
    %% /* If GNSS and IMU measurements are available, try to compute RTK fixed position and update filter */
    if ((n > 0) && (m > 0) && gnssins.imuIsAligned > 0)
        
        for i =1:m
            imuTime = imudat(imudatt + i - 1, 1);
            obsTime = rtk.sol.time;
            %/* Strapdown computation */
            if (gnssins.lastImuTime == 0.0)
                gnssins = doStrapdown(imudat(imudatt + i - 1, :), gnssins, 1.0 / gnssins.imuMeasRate);
                gnssins = propagateFilter(gnssins, 1.0 / gnssins.imuMeasRate);
            end
            if (gnssins.lastImuTime ~= 0.0)
                gnssins = doStrapdown(imudat(imudatt + i - 1, :), gnssins, imuTime - gnssins.lastImuTime);
                gnssins = propagateFilter(gnssins, imuTime - gnssins.lastImuTime);
            end
            
            if (imuTime - gnssins.lastImuTime < 0.0)
                x = 1; %// Error message necessary
            end
            gnssins.lastImuTime = imuTime;
            
            %/* Check if filter update is available */
            if ((imuTime < obsTime + (1.0 / gnssins.imuMeasRate) /2.0) && (imuTime > obsTime - (1.0 / gnssins.imuMeasRate) /2.0))
                updateCheck = 0;
                if (rtk.sol.stat == SOLQ_FIX || rtk.sol.stat == SOLQ_FLOAT)
                    if (gnssins.lastFilterUpdateTime == 0.0)
                        gnssins.lastFilterUpdateTime = obsTime;
                        gnssins = updateFilter(gnssins, rtk);
                        gnssins = doFeedback(gnssins);
                    else
                        %/* On successful update, do feedback */
                        gnssins = updateFilter(gnssins, rtk);
                        gnssins = doFeedback(gnssins);
                        gnssins.lastFilterUpdateTime = obsTime;
                        %/* Store results */
                        for j = 1:3
                            gnssins.gnss_ins_sol.att(j) = gnssins.attEuler(j);
                            gnssins.gnss_ins_sol.imuBias(j) = gnssins.accBias(j);
                            gnssins.gnss_ins_sol.imuBias(j + 3) = gnssins.gyrBias(j);
                            gnssins.gnss_ins_sol.rr(j) = gnssins.posVel(j);
                            gnssins.gnss_ins_sol.rr(j + 3) = gnssins.posVel(j + 3);
                            gnssins.gnss_ins_sol.qr(j) = gnssins.PaPosteriori(j * 16 - 15);
                            gnssins.gnss_ins_sol.qv(j) = gnssins.PaPosteriori((j+2) * 16 + 1);
                        end
                        gnssins.gnss_ins_sol.qr(3) = gnssins.PaPosteriori(15);
                        gnssins.gnss_ins_sol.qr(4) = gnssins.PaPosteriori(31);
                        gnssins.gnss_ins_sol.qr(5) = gnssins.PaPosteriori(30);
                        gnssins.gnss_ins_sol.qv(3) = gnssins.PaPosteriori(49);
                        gnssins.gnss_ins_sol.qv(4) = gnssins.PaPosteriori(65);
                        gnssins.gnss_ins_sol.qv(5) = gnssins.PaPosteriori(50);
                        gnssins.gnss_ins_sol.time  = imuTime;
                        gnssins.gnss_ins_sol.stat = SOLQ_LOOSE;
                    end
                end
                for j = 1:3
                    gnssins.curVec15by1_1(j) = gnssins.posVel(j);
                    gnssins.curVec15by1_1(3 + j) = gnssins.posVel(j + 3);
                    gnssins.curVec15by1_1(6 + j) = gnssins.attEuler(j) / pi * 180.0;
                    gnssins.curVec15by1_1(9 + j) = gnssins.accBias(j);
                    gnssins.curVec15by1_1(12 + j) = gnssins.gyrBias(j);
                end
                rtk.sol.rr(:,1:3) = pos2ecef(gnssins.gnss_ins_sol.rr);
                rtk.sol.stat = gnssins.gnss_ins_sol.stat;
            end
        end
        Result(GNSSepoch,1) = rtk.sol.time;
        Result(GNSSepoch,2) = rtk.sol.stat;
        if LCputposformat == "ecef"
            Result(GNSSepoch,3:8) = rtk.sol.rr;
        else
            Result(GNSSepoch,3:5) = ecef2pos(rtk.sol.rr);
            Result(GNSSepoch,6:8) = rtk.sol.rr(:,4:6);
        end
    else
        Result(GNSSepoch,1) = rtk.sol.time;
        Result(GNSSepoch,2) = rtk.sol.stat;
        if LCputposformat == "ecef"
            Result(GNSSepoch,3:8) = rtk.sol.rr;
        else
            Result(GNSSepoch,3:5) = ecef2pos(rtk.sol.rr(:,1:3));
            Result(GNSSepoch,6:8) = rtk.sol.rr(:,4:6);
        end
    end
end
%% Plot
geoplot(Posdata(:,3),Posdata(:,4))
hold on
geoplot(Result(:,3) .* rad_to_deg,Result(:,4) .* rad_to_deg);