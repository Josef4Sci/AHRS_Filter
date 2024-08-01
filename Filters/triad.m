function [q,Q] = triad(rN,rB)
% TRIAD Algorithm for Static Attitude Estimation
% Note: The rotation is from the body frame to the inertial frame.
%
% Inputs:
%       rN - 3xN Array of Inertial-Frame Position Vectors
%       rB - 3xN Array of Body-Frame Measured Position Vectors
%       stddev - Standard Deviation Error of Measurements
%
% Outputs:
%       Q - 3x3 Rotation Matrix
%       q - 4x1 Scalar-Last Quaternion

rN1 = rN(:,1); rN2 = rN(:,2); 
rB1 = rB(:,1); rB2 = rB(:,2); 
tN1 = rN1;
tN2 = cross(rN1,rN2)/norm(cross(rN1,rN2));

Tn = [tN1 tN2 cross(tN1,tN2)/norm(cross(tN1,tN2))];

tB1 = rB1;
tB2 = cross(rB1,rB2)/norm(cross(rB1,rB2));

Tb = [tB1 tB2 cross(tB1,tB2)/norm(cross(tB1,tB2))];

% Compute Q and q
Q = Tn*Tb';
q = quaternConj(rotm2quat(Q));