clc
clear all


rotationMatrix1 = [1 0 0; 0 1 0; 0 0 1];
quat1 = rotm2quat(rotationMatrix1)
quat11 = quaternion(quat1)
rot1 = quat2rotm(quat1)
rot11 = quat2rotm(quat11)

rotationMatrix2 = [0 -1 0; 1 0 0; 0 0 1];
quat2 = rotm2quat(rotationMatrix2)
quat22 = quaternion(quat2)
rot2 = quat2rotm(quat2)
rot22 = quat2rotm(quat22)

rotationMatrix3 = [1 0 0; 0 0 1; 0 -1 0];
quat3 = rotm2quat(rotationMatrix3)
quat33 = quaternion(quat3)
rot3 = quat2rotm(quat3)
rot33 = quat2rotm(quat33)


quat_array = [quat1; quat1; quat1];
quat_array1 = [quat11; quat11; quat11];
quat_avg = meanrot(quat_array1)

