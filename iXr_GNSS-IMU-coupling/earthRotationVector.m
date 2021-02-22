function [w_ie_n] = earthRotationVector(lat)
OMGE=        7.2921151467E-5;     %/* earth angular velocity (IS-GPS) (rad/s) */
w_ie_n(1) = OMGE * cos(lat);
w_ie_n(3) = -OMGE * sin(lat);
end

