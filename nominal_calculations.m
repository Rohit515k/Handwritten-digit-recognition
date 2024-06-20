clc;
clear;

% Read joint angles from the CSV file
dataTable = readtable('f.csv');
data = dataTable{2:end, :}; % Exclude the header row

% Initialize matrix to store results
results = zeros(size(data, 1), 17); % 17 columns for combination_number, theta_1 to theta_6, x, y, z, euler_1 to euler_3, quaternion_1 to quaternion_4

for i = 1:size(data, 1)
    % Combination number
    combination_number = dataTable{i, 1};
    
    % Joint angles for the current configuration
    q_R1 = data(i, 2:end); % Exclude the first column (Combination Number)
    th1_R1 = q_R1(1); th2_R1 = (q_R1(2) - 90); th3_R1 = (q_R1(3)); th4_R1 = q_R1(4); th5_R1 = (q_R1(5) + 180); th6_R1 = (q_R1(6));

    % DH TABLE R1 NOMINAL
    
    %Link length  %Joint offset      %Twist angle
    a1_R1=410;    d1_R1=780;         al1_R1=-90;
    a2_R1=1075;   d2_R1=0;           al2_R1=0;
    a3_R1=165;    d3_R1=0;           al3_R1=-90;
    a4_R1=0;      d4_R1=1056;        al4_R1=90;
    a5_R1=0;      d5_R1=0;           al5_R1=90;
    a6_R1=0;      d6_R1=250;         al6_R1=0;  

    % Base offsets
    XB_R1 = 0; YB_R1 = 0; ZB_R1 = 0;
    XRB_R1 = 0; YRB_R1 = 0; ZRB_R1 = 0;

    % Tool offsets
    XT_R1 = 0; YT_R1 = 0; ZT_R1 = 0;
    XRT_R1 = 0; YRT_R1 = 0; ZRT_R1 = 0;

    % Base defined 
    Tbase_R1 = Tx(XB_R1) * Ty(YB_R1) * Tz(ZB_R1) * Rx(XRB_R1) * Ry(YRB_R1) * Rz(ZRB_R1);

    % Tool defined 
    Ttool_R1 = Tx(XT_R1) * Ty(YT_R1) * Tz(ZT_R1) * Rx(XRT_R1) * Ry(YRT_R1) * Rz(ZRT_R1);

    % Conventional DH Transformation Matrix
    sal1 = sind(al1_R1); cal1 = cosd(al1_R1); sal2 = sind(al2_R1); cal2 = cosd(al2_R1); 
    sal3 = sind(al3_R1); cal3 = cosd(al3_R1); sal4 = sind(al4_R1); cal4 = cosd(al4_R1);
    sal5 = sind(al5_R1); cal5 = cosd(al5_R1); sal6 = sind(al6_R1); cal6 = cosd(al6_R1);

    sth1 = sind(th1_R1); cth1 = cosd(th1_R1); sth2 = sind(th2_R1); cth2 = cosd(th2_R1); 
    sth3 = sind(th3_R1); cth3 = cosd(th3_R1); cth4 = cosd(th4_R1); sth5 = sind(th5_R1); 
    cth5 = cosd(th5_R1); sth6 = sind(th6_R1); cth6 = cosd(th6_R1); sth4 = sind(th4_R1);

    t1m = [cth1, -sth1*cal1, sth1*sal1, a1_R1*cth1; sth1, cth1*cal1, -cth1*sal1, a1_R1*sth1; 0, sal1, cal1, d1_R1; 0, 0, 0, 1];
    t2m = [cth2, -sth2*cal2, sth2*sal2, a2_R1*cth2; sth2, cth2*cal2, -cth2*sal2, a2_R1*sth2; 0, sal2, cal2, d2_R1; 0, 0, 0, 1];
    t3m = [cth3, -sth3*cal3, sth3*sal3, a3_R1*cth3; sth3, cth3*cal3, -cth3*sal3, a3_R1*sth3; 0, sal3, cal3, d3_R1; 0, 0, 0, 1];
    t4m = [cth4, -sth4*cal4, sth4*sal4, a4_R1*cth4; sth4, cth4*cal4, -cth4*sal4, a4_R1*sth4; 0, sal4, cal4, d4_R1; 0, 0, 0, 1];
    t5m = [cth5, -sth5*cal5, sth5*sal5, a5_R1*cth5; sth5, cth5*cal5, -cth5*sal5, a5_R1*sth5; 0, sal5, cal5, d5_R1; 0, 0, 0, 1];
    t6m = [cth6, -sth6*cal6, sth6*sal6, a6_R1*cth6; sth6, cth6*cal6, -cth6*sal6, a6_R1*sth6; 0, sal6, cal6, d6_R1; 0, 0, 0, 1];

    T_DH_R1 = t1m * t2m * t3m * t4m * t5m * t6m;
    T_DH_R1_Final = Tbase_R1 * T_DH_R1 * Ttool_R1;    % Transformation matrix from Conventional DH

    T_DH_R1_base_or_world = T_DH_R1_Final;

    % Position
    x = T_DH_R1_base_or_world(1, 4);
    y = T_DH_R1_base_or_world(2, 4);
    z = T_DH_R1_base_or_world(3, 4);

    % Rotation matrix orientation of end effector with respect to base
    rotationMatrix_R1 = T_DH_R1_Final(1:3, 1:3);
    eul_R1_deg = rotm2eul_custom(rotationMatrix_R1);  % Compute Euler angles manually

    % Quaternion
    quaternion_R1 = rotm2quat_custom(rotationMatrix_R1);  % Compute quaternion manually

    % Store the results in the matrix
    results(i, :) = [combination_number, th1_R1, th2_R1, th3_R1, th4_R1, th5_R1, th6_R1, x, y, z, eul_R1_deg, quaternion_R1];
end

% Write the results to a new CSV file
headers = {'combination_number', 'theta_1', 'theta_2', 'theta_3', 'theta_4', 'theta_5', 'theta_6', 'x', 'y', 'z', 'euler_1', 'euler_2', 'euler_3', 'quaternion_1', 'quaternion_2', 'quaternion_3', 'quaternion_4'};
outputTable = array2table(results, 'VariableNames', headers);
writetable(outputTable, 'output2_nominal.csv');

% Function definitions for translation and rotation matrices
function T = Tx(x)
    T = [1 0 0 x; 
         0 1 0 0; 
         0 0 1 0; 
         0 0 0 1];
end

function T = Ty(y)
    T = [1 0 0 0; 
         0 1 0 y; 
         0 0 1 0; 
         0 0 0 1];
end

function T = Tz(z)
    T = [1 0 0 0; 
         0 1 0 0; 
         0 0 1 z; 
         0 0 0 1];
end

function R = Rx(angle)
    R = [1 0 0 0; 
         0 cosd(angle) -sind(angle) 0; 
         0 sind(angle) cosd(angle) 0; 
         0 0 0 1];
end

function R = Ry(angle)
    R = [cosd(angle) 0 sind(angle) 0; 
         0 1 0 0; 
         -sind(angle) 0 cosd(angle) 0; 
         0 0 0 1];
end

function R = Rz(angle)
    R = [cosd(angle) -sind(angle) 0 0; 
         sind(angle) cosd(angle) 0 0; 
         0 0 1 0; 
         0 0 0 1];
end

function eul = rotm2eul_custom(R)
    % Assuming R is a 3x3 rotation matrix
    sy = sqrt(R(1,1) * R(1,1) +  R(2,1) * R(2,1));
    singular = sy < 1e-6;
    
    if ~singular
        x = atan2(R(3,2), R(3,3));
        y = atan2(-R(3,1), sy);
        z = atan2(R(2,1), R(1,1));
    else
        x = atan2(-R(2,3), R(2,2));
        y = atan2(-R(3,1), sy);
        z = 0;
    end
    
    eul = [rad2deg(z), rad2deg(y), rad2deg(x)];
end

function quat = rotm2quat_custom(R)
    % Assuming R is a 3x3 rotation matrix
    K = [R(1,1)-R(2,2)-R(3,3), 0, 0, 0;
         R(1,2)+R(2,1), R(2,2)-R(1,1)-R(3,3), 0, 0;
         R(1,3)+R(3,1), R(2,3)+R(3,2), R(3,3)-R(1,1)-R(2,2), 0;
         R(2,3)-R(3,2), R(3,1)-R(1,3), R(1,2)-R(2,1), R(1,1)+R(2,2)+R(3,3)];
    K = K / 3.0;
    [V, D] = eig(K);
    [~, idx] = max(diag(D));
    quat = V(:, idx)';
    quat = quat([4, 1, 2, 3]);
end