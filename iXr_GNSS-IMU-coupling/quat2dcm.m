function [C_b_n] = quat2dcm(quat)
	q0 = quat(1);
	q1 = quat(2);
	q2 = quat(3);
	q3 = quat(4);
	C_b_n(1) = power(q0, 2) + power(q1, 2) - power(q2, 2) - power(q3, 2);
	C_b_n(2)= 2.0 * (q1 * q2 + q0 * q3);
	C_b_n(3) = 2.0 * (q1 * q3 - q0 * q2);
	C_b_n(4) = 2.0 * (q1 * q2 - q0 * q3);
	C_b_n(5) = power(q0, 2) - power(q1, 2) + power(q2, 2) - power(q3, 2);
	C_b_n(6) = 2.0 * (q2 * q3 + q0 * q1);
	C_b_n(7) = 2.0 * (q1 * q3 + q0 * q2);
	C_b_n(8) = 2.0 * (q2 * q3 - q0 * q1);
	C_b_n(9) = power(q0, 2) - power(q1, 2) - power(q2, 2) + power(q3, 2);
end

