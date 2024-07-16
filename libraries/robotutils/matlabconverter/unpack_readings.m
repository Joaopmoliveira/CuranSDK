function measurments = unpack_readings(filename)
%UNPACK_READINGS The function takes a json encoded list of points recorded
%with the robotutils api and returns a structure with all the readings of
%the robot
%   Detailed explanation goes here

text = fileread(filename);
decoded_information = jsondecode(text);

measurments.cmd_q = str2num(decoded_information.cmd_q); 
n_readings = size(measurments.cmd_q,2);

measurments.cmd_tau = str2num(decoded_information.cmd_tau); 
measurments.ddq = str2num(decoded_information.ddq); 
measurments.dq = str2num(decoded_information.dq); 


jacobians = reshape(str2num(decoded_information.jacobians),[7 6 n_readings]); 
measurments.massmatrix = reshape(str2num(decoded_information.massmatrix),[7 7 n_readings]); 
measurments.rotation = reshape(str2num(decoded_information.rotation),[3 3 n_readings]); 

measurments.jacobians = zeros(6,7,n_readings);

% Matlab and cpp differ in colum and row major format
% this makes it so they agree

for i = 1 : n_readings
    measurments.rotation(:,:,i) = measurments.rotation(:,:,i)';
    measurments.massmatrix(:,:,i) = measurments.massmatrix(:,:,i)';
    measurments.jacobians(:,:,i) = jacobians(:,:,i)';
end

measurments.q = str2num(decoded_information.q); 
measurments.tau = str2num(decoded_information.tau); 
measurments.tau_ext = str2num(decoded_information.tau_ext); 
measurments.translation = str2num(decoded_information.translation); 

measurments.userdef = str2num(decoded_information.userdef); 
measurments.userdef2 = str2num(decoded_information.userdef2); 
measurments.userdef3 = str2num(decoded_information.userdef3); 
measurments.userdef4 = str2num(decoded_information.userdef4); 
end