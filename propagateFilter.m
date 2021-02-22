function gnssins = propagateFilter(gnssins, dt)
%// Build transition matrix Phi
gnssins = buildTransitionMatrix(gnssins, dt);
%/* Propagate state vector */
gnssins.xaPosteriori = gnssins.Phi * gnssins.xaPriori;
%/* Propagate state covariance */
gnssins.curMat15by15_1 = gnssins.Phi * gnssins.PaPriori;
gnssins.PaPosteriori = gnssins.Q;
gnssins.PaPosteriori = gnssins.PaPosteriori + gnssins.curMat15by15_1 * (gnssins.Phi)';
gnssins.PaPriori = gnssins.PaPosteriori;
gnssins.xaPriori = gnssins.xaPosteriori;
end

