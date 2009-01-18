% EKF for UAV Nav System
% Author: Kedar Kulkarni [Poly ID 0356228] [kkulka01@students.poly.edu]

clear all;
% Declaring all constants
n = 6; % Number of laps
g = 9.8; % Gravity constant
r = 35; % Radius
h = 10; % Height
toutput = 1; % Time period for availability of Output Sample
estrate = 10; % Number of estimates before Output is available
w = 1/25; % Angular velocity
snr = 75; % Signal to noise ratio
% Accelorometer Gains
Kax=1;
Kay=1;
Kaz=1;
% Gyro Gains
Kgpsi=1;
Kgtheta=1;
Kgphi=1;
% Screen size for graph
scrsz = get(0,'ScreenSize');
% Declaring vars to be used for sensor inputs
Yx = 0;
Yy = 0;
Yz = 0;
% Calculating in-program constants
ts = toutput/estrate; % Sampling Rate => GPS reading rate/number of estimates before GPS reading is available
tt = round( ( ( 2 * pi ) / w ) * n ); % Total time => Total angle of circle / Angular Velocity * number of laps
t = 0:ts:tt; % Discrete time instant
ns = size(t,2); % Number of Samples
% Initialising States
X = zeros(15,1);
I = ones(1,ns); % for filling in X_expected
X_expected = [r*cos(w*t);r*sin(w*t);h.*I;0.*I;0.*I;0.*I;0.*I;0.*I; w*t;
    0.02.*I;0.02.*I;0.02.*I;0.00005.*I;0.00005.*I;0.00005.*I]; % Expected values for plotting [x,y,z,Vx,Vy,Vz,psi,theta,phi,]
% Initailising Input
U = zeros(6,1);
% Initialising Output
Y = zeros(3,1);
% Initialise C
C = zeros(6,12);
% Initialise P, Q, R, K, A, B
P = zeros(15);
Pnext = zeros(15);
Q = eye(15);
R = eye(6);
K = zeros(15,3);
A = zeros(15,15);
B = zeros(15,6);
Ax = zeros(1,1);
Ay = zeros(1,1);
Az = zeros(1,1);
for i = 1:ns-1
    % Generate values for sensor inputs
    % Gyro has bais of the magnitudee 20/3600/180*pi == 9.6963e-005
    U = [0+0.00005;0+0.00005;w+0.00005];
    % Adding Noise to sensor inputs
    U = awgn(U(:,1),snr);
    % Retrieve State Variables for this iteration
    x = X(1,i);
    y = X(2,i);
    z = X(3,i); %3
    Vx = X(4,i);
    Vy = X(5,i);
    Vz = X(6,i); %6
    psi = X(7,i);
    theta = X(8,i);
    phi = X(9,i); %9
    Bax = X(10,i);
    Bay = X(11,i);
    Baz = X(12,i); %12
    Bgpsi = X(13,i);
    Bgtheta = X(14,i);
    Bgphi = X(15,i); %15
    % Input Values for Calculations
    wpsi = U(1,1);
    wtheta = U(2,1);
    wphi = U(3,1);
    % Compute A, B matrices for initial state(first iteration) and when outputs are available
    % Compute Value for A
    A = [ 0, 0, 0, cos(theta)*cos(phi), sin(psi)*sin(theta)*cos(phi)- cos(psi)*sin(phi), -cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi), (cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi))*Vy+(sin(psi)*sin(theta)*cos(phi) +cos(psi)*sin(phi))*Vz, -sin(theta)*cos(phi)*Vx+sin(psi)*cos(theta)*cos(phi)*Vy-cos( psi)*cos(theta)*cos(phi)*Vz, -cos(theta)*sin(phi)*Vx+ (sin(psi)*sin(theta)*sin(phi)-cos(psi)*cos(phi))*Vy+(cos(psi)*sin(theta)*sin(phi) +sin(psi)*cos(phi))*Vz,0,0,0,0,0,0;
        0, 0, 0, cos(theta)*sin(phi), sin(psi)*sin(theta)*sin(phi) + cos(psi)*cos(phi), cos(psi)*sin(theta)*sin(phi)- sin(psi)*cos(phi),(cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi))*Vy+(-sin(psi)*sin(theta)*sin(phi)- cos(psi)*cos(phi))*Vz, -sin(theta)*sin(phi)*Vx+sin(psi)*cos(theta)*sin(phi)*Vy+cos(psi)*cos(theta)*sin(phi )*Vz, cos(theta)*cos(phi)*Vx+(sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi))*Vy+ (cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi))*Vz, 0, 0 ,0, 0,0,0;
        0, 0, 0, -sin(theta), sin(psi)*cos(theta), cos(psi)*cos(theta), cos(psi)*cos(theta)*Vy - sin(psi)*cos(theta)*Vz, -cos(theta)*Vx - sin(psi)*sin(theta)*Vy-cos(psi)*sin(theta)*Vz,0,0,0,0,0,0,0;
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
        0,0,0,0,0,0, cos(psi)*tan(theta)*(Kgtheta*wtheta+Bay)- sin(psi)*tan(theta)*(Kgphi*wphi+Baz), sin(psi)*(1+tan(theta)^2)*(Kgtheta*wtheta+Bay) + cos(psi)*(1+tan(theta)^2)*(Kgphi*wphi+Baz), 0,0,0,0, 1,sin(psi)*tan(theta),cos(psi)*tan(theta);
        0,0,0,0,0,0, -sin(psi)*(Kgtheta*wtheta+Bay)- cos(psi)*(Kgphi*wphi+Baz),0,0,0,0,0,0,cos(psi),-sin(psi);
        0,0,0,0,0,0, cos(psi)/cos(theta)*(Kgtheta*wtheta+Bay)- sin(psi)/cos(theta)*(Kgphi*wphi+Baz),sin(psi)/cos(theta)^2*(Kgtheta*wtheta+Bay)*sin(theta)+cos(psi)/cos(theta)^2*(Kgphi*wphi+Baz)*sin(theta),0,0,0,0,0,sin(psi)/cos(theta),cos(psi)/cos(theta);
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
        0,0,0,0,0,0,0,0,0,0,0,0,0,0,0];
    % Compute Value for B
    B = [ 0,0,0;
        0,0,0;
        0,0,0;
        0,0,0;
        0,0,0;
        0,0,0;
        Kgpsi,sin(psi)*tan(theta)*Kgtheta,cos(psi)*tan(theta)*Kgphi;
        0,cos(psi)*Kgtheta,-sin(psi)*Kgphi;
        0,sin(psi)/cos(theta)*Kgtheta,cos(psi)/cos(theta)*Kgphi;
        0,0,0;
        0,0,0;
        0,0,0;
        0,0,0;
        0,0,0;
        0,0,0];
    % Correct this state when output is available
    if i ~= 1 % Except case of i = 1 from last if
        % Generate value for output
        act_theta = 0;
        act_psi = 0;
        act_Vx = r*w;
        act_Vy = 0;
        act_Vz = 0;
        act_wpsi = 0;
        act_wphi = w;
        act_wtheta = 0;
        act_Bax = 0;
        act_Bay = 0;
        act_Baz = 0;
        Ax(1,i) = ( (-act_Vy*(act_wphi)/g + act_Vz*(act_wtheta))/g +sin(act_theta) )*Kax + act_Bax;
        Ay(1,i) = ( (act_Vx*(act_wphi)/g - act_Vz*(act_wpsi))/g -sin(act_psi)*cos(act_theta))*Kay + act_Bay;
        Az(1,i) = ( (-act_Vx*(act_wtheta)/g + act_Vy*(act_wpsi))/g -cos(act_theta)*cos(act_psi))*Kaz + act_Baz;
        % If GPS reading not available plugin the estimated value
        if rem( i, estrate ) == 0
            Yx = r*cos(w*t(1,i));
            Yy = r*sin(w*t(1,i));
            Yz = h;
        else
            Yx = X(1,i);
            Yy = X(2,i);
            Yz = X(3,i);
        end
        % Generate Output
        Y = [ Yx; Yy ; Yz; Ax(1,i)+0.02;Ay(1,i)+0.02;Az(1,i)+0.02];
        % Adding Noise to measured output
        Y = awgn(Y(:,1),snr);
        C = [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
            0,1,0,0,0,0,0,0,0,0,0,0,0,0,0;
            0,0,1,0,0,0,0,0,0,0,0,0,0,0,0;
            0,0,0,0,-(wphi+Bgphi)/g,(wtheta+Bgtheta)/g,0,cos(theta),0,1,0,0,0,Vz/g,-Vy/g;
            0,0,0,(wphi+Bgphi)/g,0,-(wpsi+Bgpsi)/g,cos(psi)*cos(theta),sin(psi)*sin(theta),0,0,1,0,-Vz/g,0,Vx/g;
            0,0,0,-(wtheta+Bgtheta)/g,(wpsi+Bgpsi)/g,0,sin(psi)*cos(theta),cos(psi)*sin(theta),0,0,0,1,Vy/g,-Vx/g,0];
        % Calculate Gain
        K = P * C' * inv( (C * P * C') + R);
        % Correct State Estimate
        X(:,i) = X(:,i) + K * ( Y - C*X(:,i) );
        % Pnext for next set of estimates
        Pnext = (eye(15) - K*C) * P;
    end % ends if for this state correction
    % Compute value of f(x,u) for these states
    func = [ cos(theta)*cos(phi)*Vx+(sin(psi)*sin(theta)*cos(phi)- cos(psi)*sin(phi))*Vy+(-cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi))*Vz;
        cos(theta)*sin(phi)*Vx+(sin(psi)*sin(theta)*sin(phi) + cos(psi)*cos(phi))*Vy+(cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi))*Vz;
        -sin(theta)*Vx+sin(psi)*cos(theta)*Vy+cos(psi)*cos(theta)*Vz;
        0;
        0;
        0;
        Kgpsi*wpsi+Bgpsi+sin(psi)*tan(theta)*(Kgtheta*wtheta+Bgtheta)+ cos(psi)*tan(theta)*(Kgphi*wphi+Bgphi);
        cos(psi)*(Kgtheta*wtheta+Bgtheta)+sin(psi)*(Kgphi*wphi+Bgphi);
        sin(psi)/cos(theta)*(Kgtheta*wtheta+Bgtheta) + cos(psi)/cos(theta)*(Kgphi*wphi+Bgphi);
        0;
        0;
        0;
        0;
        0;
        0];
    % Update P for next estimate
    P = (A * Pnext * A') + Q ;
    % Estimate next states
    X(:,i+1) = X(:,i) + ( ts * func );
end % ends sampling for Loop
% Plotting relevant Graphs
figure('Position',[1,8, 0.90*scrsz(3), 0.90*scrsz(4)])
subplot(1,3,1),plot3(X(1,:),X(2,:),X(3,:));grid on;title('Position, Inertial Frame');xlabel('x');ylabel('y');zlabel('z');
subplot(1,3,2),plot(X(1,:),X(2,:));grid on;title('Position, Inertial frame XY');
xlabel('x');ylabel('y');
subplot(1,3,3),plot(X(1,:),X(3,:));grid on;title('Position, Inertial frame XZ');
xlabel('x');ylabel('z');
figure('Position',[1,8, 0.80*scrsz(3), 0.80*scrsz(4)])
subplot(2,2,1),plot(t,X(4,:),'r',t,X(5,:),'b',t,X(6,:),'g');grid on;title('Linear Velocities vs Time');legend('Vx','Vy','Vz',1);
subplot(2,2,2),plot(t,X(7,:),'r',t,X(8,:),'b',t,X(9,:),'g');grid on;title('Pitch - Roll-Yaw vs Time');legend('\psi','\theta','\phi',1);
subplot(2,2,3),plot(t,X(1,:),'r',t,X(2,:),'b',t,X(3,:),'g');grid on;title('Position in Inertial frame vs Time');legend('x','y','z',1);
figure('Position',[1,8, 0.80*scrsz(3), 0.80*scrsz(4)])
subplot(1,2,1),plot(t,X(10,:),t,X(11,:),t,X(12,:));grid on;title('Accelerometer Bias vs Time');legend('Bax','Bay','Baz',1);
subplot(1,2,2),plot(t,X(13,:),t,X(14,:),t,X(15,:));grid on;title('Gyro Bias vs Time');legend('Bg\psi','Bg\theta','Bg\phi',1);
figure('Position',[1,8, 0.80*scrsz(3), 0.80*scrsz(4)])
subplot(3,3,1),plot(t,X(1,:),t,X_expected(1,:));grid on;title('X - Actual vs Expected');legend('Actual','Expected',1);
subplot(3,3,2),plot(t,X(2,:),t,X_expected(2,:));grid on;title('Y - Actual vs Expected');legend('Actual','Expected',1);
subplot(3,3,3),plot(t,X(3,:),t,X_expected(3,:));grid on;title('Z - Actual vs Expected');legend('Actual','Expected',1);
subplot(3,3,4),plot(t,X(4,:),t,X_expected(4,:));grid on;title('Vx - Actual vs Expected');legend('Actual','Expected',1);
subplot(3,3,5),plot(t,X(5,:),t,X_expected(5,:));grid on;title('Vy - Actual vs Expected');legend('Actual','Expected',1);
subplot(3,3,6),plot(t,X(6,:),t,X_expected(6,:));grid on;title('Vz - Actual vs Expected');legend('Actual','Expected',1);
subplot(3,3,7),plot(t,X(7,:),t,X_expected(7,:));grid on;title('\psi - Actual vs Expected');legend('Actual','Expected',1);
subplot(3,3,8),plot(t,X(8,:),t,X_expected(8,:));grid on;title('\theta - Actual vs Expected');legend('Actual','Expected',1);
subplot(3,3,9),plot(t,X(9,:),t,X_expected(9,:));grid on;title('\phi - Actual vs Expected');legend('Actual','Expected',1);
figure('Position',[1,8, 0.80*scrsz(3), 0.80*scrsz(4)])
subplot(3,3,1),plot(t,X(10,:),t,X_expected(10,:));grid on;title('Accelerometer Bias Bax - Actual vs Expected');legend('Actual','Expected',1);
subplot(3,3,2),plot(t,X(11,:),t,X_expected(11,:));grid on;title('Accelerometer Bias Bay - Actual vs Expected');legend('Actual','Expected',1);
subplot(3,3,3),plot(t,X(12,:),t,X_expected(12,:));grid on;title('Accelerometer Bias Baz - Actual vs Expected');legend('Actual','Expected',1);
subplot(3,3,4),plot(t,X(13,:),t,X_expected(13,:));grid on;title('Gyro Bias Bg\psi - Actual vs Expected');legend('Actual','Expected',1);
subplot(3,3,5),plot(t,X(14,:),t,X_expected(14,:));grid on;title('Gyro Bias Bg\theta - Actual vs Expected');legend('Actual','Expected',1);
subplot(3,3,6),plot(t,X(15,:),t,X_expected(15,:));grid on;title('Gyro Bias Bg\phi - Actual vs Expected');legend('Actual','Expected',1);