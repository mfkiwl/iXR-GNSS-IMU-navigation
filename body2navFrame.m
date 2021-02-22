function [C_b_n] = body2navFrame(roll,pitch,yaw)
C_b_n = zeros(3,3);
C_b_n(1) = cos(yaw)*cos(pitch);
C_b_n(2) = sin(yaw)*cos(pitch);
C_b_n(3) = -sin(pitch);
C_b_n(4) = -sin(yaw)*cos(roll) + cos(yaw)*sin(pitch)*sin(roll);
C_b_n(5) = cos(yaw)*cos(roll) + sin(yaw)*sin(pitch)*sin(roll);
C_b_n(6) = cos(pitch)*sin(roll);
C_b_n(7) = sin(yaw)*sin(roll) + cos(yaw)*sin(pitch)*cos(roll);
C_b_n(8) = -cos(yaw)*sin(roll) + sin(yaw)*sin(pitch)*cos(roll);
C_b_n(9) = cos(pitch)*cos(roll);
end

