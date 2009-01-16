% Linear Model for UAV with State Bias Decoupling

clear all;
close all;

% Declaring all constants
n = 10;       % Number of laps
g = 9.8;      % Gravity constant
r = 35;       % Radius
h = 10;       % Height
toutput = 1;  % Time period before Output Sample becomes available
estrate = 10;
% estrate = 40; % Number of estimates before Output is available. Effectively this is the input signal sampling
w = 1/25;     % Angular velocity

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

% Calculating in-program constants
ts = toutput/estrate;     % Sampling Rate => GPS reading rate/number of estimates before GPS reading is available
tt = round( ( ( 2 * pi ) / w ) * n ); % Total time => Total angle of circle / Angular Velocity * number of laps
t = 0:ts:tt; % Discrete time instant
ns = size(t,2); % Number of Samples

% Initialising States Variables
X = zeros(9,1);
X_corrected = zeros(9,1);
Yc = zeros(6,1); % Calculated Output.
res = zeros(6,1); % residual => Ya-Yc
A = zeros(9,9);
C = zeros(9,6);   % Corresponds to H in text

% Covariance Matrix
P = eye(9,9);
% Posteri Covariance Matrix
T = zeros(9,9); % Corresponds to Pnext

% The Gain Matrix 
K = zeros(9,6);
% The bias free gain matrices
K_x = zeros(9,6); % From eqn 72 for state dimesions equal to K
K_b = zeros(6,6); % Dimensions from eqn 73 for bias

% Creating the input,output measurements
% Creating random number matrix
randMat = rand(1,ns);
I = ones(1,ns); % for filling in X_expected, U ,Y

% Setup the standard deviations for the R matricies later
PosSD = 0.8;  % Position
VelSD = 0.2;  % Velocity

% Initiating all variances
% Accelorometer Variance
varAx = 0.002;
varAy = 0.002;
varAz = 0.002;
% Gyro Variance
varGpsi = 0.000005;
varGtheta = 0.000005;
varGphi = 0.000005;
% Position Variance (from GPS)
varGPSx = 0.3;
varGPSy = 0.3;
varGPSz = 0.3;

% Initialising actual biases for modelling
% Gyro biases
act_Bgpsi = 0.00005;
act_Bgtheta = 0.00005;
act_Bgphi = 0.00005;
act_Bax = 0.02;
act_Bay = 0.02;
act_Baz = 0.02;

% Creating Sensor Input
% Gyro has bias of the magnitude 20/3600/180*pi ==   9.6963e-005
% Initialisng the Inputs
Ux = 0 + act_Bgpsi;
Uy = 0 + act_Bgtheta;
Uz = w + act_Bgphi;

% Adding noise based on variance
Ux = Ux + (sqrt(varGpsi) .* randMat);
Uy = Uy + (sqrt(varGtheta) .* randMat);
Uz = Uz + (sqrt(varGphi) .* randMat);
% Create a single output array of input readings
U = [ Ux; Uy; Uz ];

% Creating the Outputs
act_psi = 0;
act_theta = 0;
act_phi = w.*t;
act_Vx = r*w*cos(w.*t);
act_Vy = r*w*sin(w.*t);
act_Vz = 0;
act_wpsi = 0;
act_wphi = w;
act_wtheta = 0;

% Accelerometer Readings - Linear Acceleration - Unit m/s^2
Ax = ( (-act_Vy.*(act_wphi)/g + act_Vz.*(act_wtheta))/g + sin(act_theta) )*Kax + act_Bax;
Ay = ( (act_Vx.*(act_wphi)/g - act_Vz.*(act_wpsi))/g - sin(act_psi)*cos(act_theta))*Kay + act_Bay;
Az = ( (-act_Vx.*(act_wtheta)/g + act_Vy.*(act_wpsi))/g - cos(act_theta)*cos(act_psi))*Kaz + act_Baz;
% GPS Readings - Gives x,y,z cordinates(Absolute Values) - Unit m
Yx = r*cos(w*t);
Yy = r*sin(w*t);
Yz = h.*I;

% Generate Output
% Adding noise based on variances
Ax = Ax + ( sqrt(varAx) .* randMat );
Ay = Ay + ( sqrt(varAy) .* randMat );
Az = Az + ( sqrt(varAz) .* randMat );
Yx = Yx + ( sqrt(varGPSx) .* randMat );
Yy = Yy + ( sqrt(varGPSy) .* randMat );
Yz = Yz + ( sqrt(varGPSz) .* randMat );
% Create a single output array of output readings
Ya = [ Yx; Yy ; Yz; Ax ;Ay ;Az ];


%Expected Values
X_expected = [r*cos(w*t);r*sin(w*t);h.*I;act_Vx;act_Vy;0.*I;0.*I;0.*I; w*t; act_Bax.*I;act_Bay.*I;act_Baz.*I;act_Bgpsi.*I;act_Bgtheta.*I;act_Bgphi.*I]; % Expected values for plotting [x,y,z,Vx,Vy,Vz,psi,theta,phi,]

% Create the Q, R matrices corresponding to variances of measured
% parameters. Note Variance is the square of the standard deviation for
% some parameters initialise SD which we convert to variance before
% initialization
Q = diag([PosSD^2,PosSD^2,PosSD^2,varAx,varAy,varAz,varGpsi,varGtheta,varGphi]);
R = diag([PosSD^2,PosSD^2,PosSD^2,VelSD^2,VelSD^2,VelSD^2]);

% Initialising Bias Variables
U_x_bias = zeros(9,6);
U_b_bias = eye(6,6); % eqn 59 and 61.a
V_x_bias = zeros(9,6);
V_b_bias = U_b_bias; % eqn 59 and 61.a
C_bias = zeros(6,6); % Corresponds to C in text
S_bias = zeros(6,6); % Corresponds to 6 outputs X 6 bias parameters eqn 63
B_bias = zeros(9,6); % Corresponds to B in text
M_bias = eye(6,6); % Dimensions from eqn 16 initial condn from 23 equated to Pb
b_bias = zeros(6,ns); % Bias vector in terms of absolute value

% The variable represnting measured bias
delta = zeros(9,1);

for i = 1:ns-1 % Start the iteration
    % System states for this iteration
    x = X(1,i);      % Linear Position Unit m
    y = X(2,i);      % Linear Position Unit m
    z = X(3,i);      % Linear Position Unit m
    Vx = X(4,i);     % Linear Velocity Unit m/s
    Vy = X(5,i);     % Linear Velocity Unit m/s
    Vz = X(6,i);     % Linear Velocity Unit m/s
    psi = X(7,i);    % Angular Position Unit rad
    theta = X(8,i);  % Angular Position Unit rad
    phi = X(9,i);    % Angular Position Unit rad

    % Input Values for Calculations. Gyro Readings Angular Velocity
    wpsi = U(1,i);     % Unit rad/s
    wtheta = U(2,i);   % Unit rad/s
    wphi = U(3,i);     % Unit rad/s

    % Corresponds to H in text
    C = [1,0,0,0,0,0,0,0,0;
        0,1,0,0,0,0,0,0,0;
        0,0,1,0,0,0,0,0,0;
        0,0,0,0,-(wphi)/g,(wtheta)/g,0,cos(theta),0;
        0,0,0,(wphi)/g,0,-(wpsi)/g,cos(psi)*cos(theta),sin(psi)*sin(theta),0;
        0,0,0,-(wtheta)/g,(wpsi)/g,0,sin(psi)*cos(theta),cos(psi)*sin(theta),0];

    % Calculate Gain
    K = P * C' * inv( (C * P * C') + R); % eqn 74
    % Calculate the output vector
    Yc = C * X(:,i);

    if ( rem( i, estrate ) == 0 )
        % Corresponds to C in text
        C_bias = [
            0,0,0,0,0,0;
            0,0,0,0,0,0;
            0,0,0,0,0,0;
            1,0,0,0,Vz/g,-Vy/g;
            0,1,0,-Vz/g,0,Vx/g;
            0,0,1,Vy/g,-Vx/g,0
            ];
        % Calculate A matrix
        A = [
            0, 0, 0, cos(theta)*cos(phi), sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi), -cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi), (cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi))*Vy+(sin(psi)*sin(theta)*cos(phi)+cos(psi)*sin(phi))*Vz, -sin(theta)*cos(phi)*Vx+sin(psi)*cos(theta)*cos(phi)*Vy-cos(psi)*cos(theta)*cos(phi)*Vz, -cos(theta)*sin(phi)*Vx+(sin(psi)*sin(theta)*sin(phi)-cos(psi)*cos(phi))*Vy+(cos(psi)*sin(theta)*sin(phi)+sin(psi)*cos(phi))*Vz;
            0, 0, 0, cos(theta)*sin(phi), sin(psi)*sin(theta)*sin(phi)+cos(psi)*cos(phi), cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi), (cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi))*Vy+(-sin(psi)*sin(theta)*sin(phi)-cos(psi)*cos(phi))*Vz, -sin(theta)*sin(phi)*Vx+sin(psi)*cos(theta)*sin(phi)*Vy+cos(psi)*cos(theta)*sin(phi)*Vz, cos(theta)*cos(phi)*Vx+(sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi))*Vy+(cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi))*Vz;
            0, 0, 0, -sin(theta), sin(psi)*cos(theta), cos(psi)*cos(theta), cos(psi)*cos(theta)*Vy-sin(psi)*cos(theta)*Vz, -cos(theta)*Vx-sin(psi)*sin(theta)*Vy-cos(psi)*sin(theta)*Vz,0;
            0,0,0,0,0,0,0,0,0;
            0,0,0,0,0,0,0,0,0;
            0,0,0,0,0,0,0,0,0;
            0,0,0,0,0,0, cos(psi)*tan(theta)*(Kgtheta*wtheta)-sin(psi)*tan(theta)*(Kgphi*wphi), sin(psi)*(1+tan(theta)^2)*(Kgtheta*wtheta)+cos(psi)*(1+tan(theta)^2)*(Kgphi*wphi), 0;
            0,0,0,0,0,0, -sin(psi)*(Kgtheta*wtheta)-cos(psi)*(Kgphi*wphi),0,0;
            0,0,0,0,0,0, cos(psi)/cos(theta)*(Kgtheta*wtheta)-sin(psi)/cos(theta)*(Kgphi*wphi),sin(psi)/cos(theta)^2*(Kgtheta*wtheta)*sin(theta)+cos(psi)/cos(theta)^2*(Kgphi*wphi)*sin(theta),0;
            ];
        
        % B_bias => B in text
        B_bias = [
            0,0,0,0,0,0;
            0,0,0,0,0,0;
            0,0,0,0,0,0;
            0,0,0,0,0,0;
            0,0,0,0,0,0;
            0,0,0,0,0,0;
            0,0,0,1,sin(psi)*tan(theta),cos(psi)*tan(theta);
            0,0,0,0,cos(psi),-sin(psi);
            0,0,0,0,sin(psi)/cos(theta),cos(psi)/cos(theta);
            ];

        % Calculate Output Residue as difference between calculated and
        % actual output
        res = Ya(:,i) - Yc;
        
        % Bias Calculation
        S_bias = ( C * U_x_bias ) + C_bias; % eqn 63
        V_x_bias = U_x_bias - (K * S_bias);  % eqn 62
        M_bias = M_bias - ( (M_bias * S_bias')* inv( ( C * P * C') + R + (S_bias * M_bias * S_bias') )  * (S_bias * M_bias) );
        K_b = M_bias * ( ( V_x_bias' * C') + C_bias' ) * inv(R);

        % Correct the estimates
        K_x = K +( V_x_bias * K_b);
        X(:,i) = X(:,i) + K_x * res;
        
        % Calculate the bias
        if (i ~= 1)
            b_bias(:,i) = ( (eye(6) - ( K_b * S_bias )) * b_bias(:,i-1)) + (K_b*res);
        end  % State has been initialsed to zero. b_bias(:,i-1) will access Out Of Range

        % bias correction eqn pg 365 diagram
        delta(:,i) = V_x_bias * b_bias(:,i); 

        % Adding bias correction to state. Look at fig 2 on pg 365
        X_corrected(:,i) = X(:,i) + delta(:,i);
        
        %Update U_x_bias for next iteration
        U_x_bias = ( A * V_x_bias) + B_bias; % refer eqn 58
        
    end  % end the observation correction if
    
    % Assign local variables for biases for ease of calculation
    Bax = b_bias(1,i);
    Bay = b_bias(2,i);
    Baz = b_bias(3,i);
    Bgpsi = b_bias(4,i);
    Bgtheta = b_bias(5,i);
    Bgphi = b_bias(6,i);

    % For next loop
    % Compute value of f(x,u) for these states
    func = [ cos(theta)*cos(phi)*Vx+(sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi))*Vy+(-cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi))*Vz;
        cos(theta)*sin(phi)*Vx+(sin(psi)*sin(theta)*sin(phi)+cos(psi)*cos(phi))*Vy+(cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi))*Vz;
        -sin(theta)*Vx+sin(psi)*cos(theta)*Vy+cos(psi)*cos(theta)*Vz;
        0;
        0;
        0;
        Kgpsi*wpsi+Bgpsi+sin(psi)*tan(theta)*(Kgtheta*wtheta+Bgtheta)+cos(psi)*tan(theta)*(Kgphi*wphi+Bgphi);
        cos(psi)*(Kgtheta*wtheta+Bgtheta)+sin(psi)*(Kgphi*wphi+Bgphi);
        sin(psi)/cos(theta)*(Kgtheta*wtheta+Bgtheta)+cos(psi)/cos(theta)*(Kgphi*wphi+Bgphi);
        ];


    % Approximate next states of the system
    X(:,i+1) = X_corrected(:,i) + ts * func;
    X_corrected(:,i+1) = X(:,i+1);

    % Calculate Aposteri Covariance Matrix 
    T = (eye(9) - K*C) * P;

    % Update P for next estimate
    P = (A * T * A') + Q  ;

end % ends loop => for i = 1:ns-1

% Plotting relevant Graphs

figure('Position',[1,8, 0.90*scrsz(3), 0.90*scrsz(4)])
subplot(1,3,1),plot3(X_corrected(1,:),X_corrected(2,:),X_corrected(3,:));grid on;title('Corrected: Position, Inertial Frame');xlabel('x');ylabel('y');zlabel('z');
subplot(1,3,2),plot(X_corrected(1,:),X_corrected(2,:));grid on;title('Corrected: Position, Inertial frame X-Y');xlabel('x');ylabel('y');
subplot(1,3,3),plot(X_corrected(1,:),X_corrected(3,:));grid on;title('Corrected: Position, Inertial frame X-Z');xlabel('x');ylabel('z');

figure('Position',[1,8, 0.80*scrsz(3), 0.80*scrsz(4)])
subplot(2,2,1),plot(t,X_corrected(4,:),'r',t,X_corrected(5,:),'b',t,X_corrected(6,:),'g');grid on;title('Corrected: Linear Velocities vs Time');legend('Vx','Vy','Vz',1);
subplot(2,2,2),plot(t,X_corrected(7,:),'r',t,X_corrected(8,:),'b',t,X_corrected(9,:),'g');grid on;title('Corrected: Pitch-Roll-Yaw vs Time');legend('\psi','\theta','\phi',1);
subplot(2,2,3),plot(t,X_corrected(1,:),'r',t,X_corrected(2,:),'b',t,X_corrected(3,:),'g');grid on;title('Corrected: Position in Inertial frame vs Time');legend('x','y','z',1);

b_bias(:,i+1) = b_bias(:,i);

figure('Position',[1,8, 0.80*scrsz(3), 0.80*scrsz(4)])
subplot(1,2,1),plot(t,b_bias(1,:),t,b_bias(2,:),t,b_bias(3,:));grid on;title('Accelerometer Bias vs Time');legend('Bax','Bay','Baz',1);
subplot(1,2,2),plot(t,b_bias(4,:),t,b_bias(5,:),t,b_bias(6,:));grid on;title('Gyro Bias vs Time');legend('Bg\psi','Bg\theta','Bg\phi',1);

figure('Position',[1,8, 0.80*scrsz(3), 0.80*scrsz(4)])
subplot(3,3,1),plot(t,X_corrected(1,:),t,X_expected(1,:));grid on;title('X - Actual vs Expected');legend('Actual','Expected',1);
subplot(3,3,2),plot(t,X_corrected(2,:),t,X_expected(2,:));grid on;title('Y - Actual vs Expected');legend('Actual','Expected',1);
subplot(3,3,3),plot(t,X_corrected(3,:),t,X_expected(3,:));grid on;title('Z - Actual vs Expected');legend('Actual','Expected',1);
subplot(3,3,4),plot(t,X_corrected(4,:),t,X_expected(4,:));grid on;title('Vx - Actual vs Expected');legend('Actual','Expected',1);
subplot(3,3,5),plot(t,X_corrected(5,:),t,X_expected(5,:));grid on;title('Vy - Actual vs Expected');legend('Actual','Expected',1);
subplot(3,3,6),plot(t,X_corrected(6,:),t,X_expected(6,:));grid on;title('Vz - Actual vs Expected');legend('Actual','Expected',1);
subplot(3,3,7),plot(t,X_corrected(7,:),t,X_expected(7,:));grid on;title('\psi - Actual vs Expected');legend('Actual','Expected',1);
subplot(3,3,8),plot(t,X_corrected(8,:),t,X_expected(8,:));grid on;title('\theta - Actual vs Expected');legend('Actual','Expected',1);
subplot(3,3,9),plot(t,X_corrected(9,:),t,X_expected(9,:));grid on;title('\phi - Actual vs Expected');legend('Actual','Expected',1);

figure('Position',[1,8, 0.80*scrsz(3), 0.80*scrsz(4)])
subplot(2,2,1),plot(t,(X_corrected(1,:)-X_expected(1,:)));grid on;title('Position X - Error');legend('X',1);
subplot(2,2,2),plot(t,(X_corrected(2,:)-X_expected(2,:)));grid on;title('Position Y - Error');legend('Y',1);
subplot(2,2,3),plot(t,(X_corrected(3,:)-X_expected(3,:)));grid on;title('Position Y - Error');legend('Z',1);

figure('Position',[1,8, 0.80*scrsz(3), 0.80*scrsz(4)])
subplot(2,2,1),plot(t,(X_corrected(4,:)-X_expected(4,:)));grid on;title('Linear Velocity X - Error');legend('Vx',1);
subplot(2,2,2),plot(t,(X_corrected(5,:)-X_expected(5,:)));grid on;title('Linear Velocity Y - Error');legend('Vy',1);
subplot(2,2,3),plot(t,(X_corrected(6,:)-X_expected(6,:)));grid on;title('Linear Velocity Z - Error');legend('Vz',1);

figure('Position',[1,8, 0.80*scrsz(3), 0.80*scrsz(4)])
subplot(2,2,1),plot(t,(X_corrected(4,:)-X_expected(4,:)));grid on;title('Linear Velocity X - Error');legend('Vx',1);
subplot(2,2,2),plot(t,(X_corrected(5,:)-X_expected(5,:)));grid on;title('Linear Velocity Y - Error');legend('Vy',1);
subplot(2,2,3),plot(t,(X_corrected(6,:)-X_expected(6,:)));grid on;title('Linear Velocity Z - Error');legend('Vz',1);

figure('Position',[1,8, 0.80*scrsz(3), 0.80*scrsz(4)])
subplot(2,2,1),plot(t,(X_corrected(7,:)-X_expected(7,:)));grid on;title('Angular Posiltion \psi  - Error');legend('\psi ',1);
subplot(2,2,2),plot(t,(X_corrected(8,:)-X_expected(8,:)));grid on;title('Angular Posiltion \theta - Error');legend('\theta',1);
subplot(2,2,3),plot(t,(X_corrected(9,:)-X_expected(9,:)));grid on;title('Angular Posiltion \phi - Error');legend('\phi',1);

figure('Position',[1,8, 0.80*scrsz(3), 0.80*scrsz(4)])
subplot(2,2,1),plot(t,(b_bias(1,:)-X_expected(10,:)));grid on;title('Accelerometer Bias Bax  - Error');legend('Bax ',1);
subplot(2,2,2),plot(t,(b_bias(2,:)-X_expected(11,:)));grid on;title('Accelerometer Bias Bay - Error');legend('Bay',1);
subplot(2,2,3),plot(t,(b_bias(3,:)-X_expected(12,:)));grid on;title('Accelerometer Bias Baz - Error');legend('Baz',1);

figure('Position',[1,8, 0.80*scrsz(3), 0.80*scrsz(4)])
subplot(2,2,1),plot(t,(b_bias(4,:)-X_expected(13,:)));grid on;title('Gyro Bias Bg\psi  - Error');legend('Bg\psi ',1);
subplot(2,2,2),plot(t,(b_bias(5,:)-X_expected(14,:)));grid on;title('Gyro Bias Bg\theta - Error');legend('Bg\theta',1);
subplot(2,2,3),plot(t,(b_bias(6,:)-X_expected(15,:)));grid on;title('Gyro Bias Bg\phi - Error');legend('Bg\phi',1);

figure('Position',[1,8, 0.80*scrsz(3), 0.80*scrsz(4)])
subplot(3,3,1),plot(t,b_bias(1,:),t,X_expected(10,:));grid on;title('Accelerometer Bias Bax - Actual vs Expected');legend('Actual','Expected',1);
subplot(3,3,2),plot(t,b_bias(2,:),t,X_expected(11,:));grid on;title('Accelerometer Bias Bay - Actual vs Expected');legend('Actual','Expected',1);
subplot(3,3,3),plot(t,b_bias(3,:),t,X_expected(12,:));grid on;title('Accelerometer Bias Baz - Actual vs Expected');legend('Actual','Expected',1);
subplot(3,3,4),plot(t,b_bias(4,:),t,X_expected(13,:));grid on;title('Gyro Bias Bg\psi - Actual vs Expected');legend('Actual','Expected',1);
subplot(3,3,5),plot(t,b_bias(5,:),t,X_expected(14,:));grid on;title('Gyro Bias Bg\theta - Actual vs Expected');legend('Actual','Expected',1);
subplot(3,3,6),plot(t,b_bias(6,:),t,X_expected(15,:));grid on;title('Gyro Bias Bg\phi - Actual vs Expected');legend('Actual','Expected',1);