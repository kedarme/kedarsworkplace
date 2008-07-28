% Linear Model for uav
% Modifying for Friedlands paper

clear all;

% Declaring all constants
n = 6;       % Number of laps
g = 9.8;      % Gravity constant
r = 35;       % Radius
h = 10;       % Height
toutput = 1;  % Time period for availability of Output Sample
estrate = 10; % Number of estimates before Output is available
w = 1/25;     % Angular velocity
snr = 75;     % Signal to noise ratio
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
Yc = zeros(6,1); % Calculated Output.
Ya = zeros(6,1); % Actual Output.
res = zeros(6,1); % residual => Ya-Yc
A = zeros(9,9);
C = zeros(9,6);   % Corresponds to H in text
P = zeros(9);
T = zeros(9); % Async to Pnext
Q = eye(9);
R = eye(6);
K = zeros(9,6);
K_x = K; % From eqn 72 for state
K_b = zeros(6,6); % Dimensions from eqn 73 for bias

% Initialising Bias Variables
U_x_bias = zeros(9,6);
U_b_bias = eye(6,6); % eqn 59 and 61.a
V_x_bias = zeros(9,6);
V_b_bias = U_b_bias; % eqn 59 and 61.a

C_bias = zeros(6,6); % Corresponds to C in text
S_bias = zeros(6,6); % Corresponds to 6 outputs X 6 bias parameters eqn 63

B_bias = zeros(9,6); % Corresponds to B in text
M_bias = zeros(6,6); % Dimensions from eqn 16

b_bias = zeros(6,1); % Bias vector in terms of absolute value
delta = zeros(9,1);

% Variables for Outputs
Ax = 0;
Ay = 0;
Az = 0;
Yx = 0;
Yy = 0;
Yz = 0;

% Initailising Input
%U = [0;0;w];
% Generate values for sensor inputs
% Gyro has bais of the magnitudee 20/3600/180*pi ==   9.6963e-005
U = [0+0.00005;0+0.00005;w+0.00005];
% Adding Noise to sensor inputs
U = awgn(U(:,1),snr);


for i = 1:ns-1 % Start the loop

    % System states for this iteration
    x = X(1,i);
    y = X(2,i);
    z = X(3,i);    %3
    Vx = X(4,i);
    Vy = X(5,i);
    Vz = X(6,i);   %6
    psi = X(7,i);
    theta = X(8,i);
    phi = X(9,i);  %9

    % Input Values for Calculations
    wpsi = U(1,1);
    wtheta = U(2,1);
    wphi = U(3,1);



    C = [1,0,0,0,0,0,0,0,0;
        0,1,0,0,0,0,0,0,0;
        0,0,1,0,0,0,0,0,0;
        0,0,0,0,-(wphi)/g,(wtheta)/g,0,cos(theta),0;
        0,0,0,(wphi)/g,0,-(wpsi)/g,cos(psi)*cos(theta),sin(psi)*sin(theta),0;
        0,0,0,-(wtheta)/g,(wpsi)/g,0,sin(psi)*cos(theta),cos(psi)*sin(theta),0];

    C_bias = [
        0,0,0,0,0,0;
        0,0,0,0,0,0;
        0,0,0,0,0,0;
        1,0,0,0,Vz/g,-Vy/g;
        0,1,0,-Vz/g,0,Vx/g;
        0,0,1,Vy/g,-Vx/g,0
        ];

    Yc = C * X(:,i);

    % Calculate Gain
    K = P * C' * inv( (C * P * C') + R); % eqn 74


    if ( rem( i, estrate ) == 0 || i == 1 )
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


        % Actual Observations
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
        Ax = ( (-act_Vy*(act_wphi)/g + act_Vz*(act_wtheta))/g + sin(act_theta) )*Kax + act_Bax;
        Ay = ( (act_Vx*(act_wphi)/g - act_Vz*(act_wpsi))/g - sin(act_psi)*cos(act_theta))*Kay + act_Bay;
        Az = ( (-act_Vx*(act_wtheta)/g + act_Vy*(act_wpsi))/g - cos(act_theta)*cos(act_psi))*Kaz + act_Baz;
        Yx = r*cos(w*t(1,i));
        Yy = r*sin(w*t(1,i));
        Yz = h;

        % Pure Output
        %Ya = [ r*cos(w*t(1,i)); r*sin(w*t(1,i));h ; Ax;Ay;Az];

        % Generate Output
        Ya = [ Yx; Yy ; Yz; Ax+0.02;Ay+0.02;Az+0.02];
        % Adding Noise to measured output
        Ya = awgn(Ya(:,1),snr);

        % Calculate Residue
        res = Ya - Yc;

        % Bias Calculation
        S_bias = ( C * U_x_bias ) + C_bias; % eqn 63
        V_x_bias = U_x_bias - (K * S_bias);  % eqn 62
        M_bias = M_bias - ( (M_bias * S_bias')* inv( ( C * P * C') + R + (S_bias * M_bias * S_bias') )  * (S_bias * M_bias) );
        K_b = M_bias * ( ( V_x_bias' * C') + C_bias' ) * inv(R);





        % Correct the estimates
        %         X(:,i) = X(:,i) + K * (Ya - C * X(:,i));
        K_x = K +( V_x_bias * K_b);
        X(:,i) = X(:,i) + K * res;

    end  % end the observation correction if
    if (i ~= 1)
        b_bias(:,i) = ( (eye(6) - ( K_b * S_bias )) * b_bias(:,i-1)) + (K_b*res);
    end  % State has been initialsed to zero. b_bias(:,i-1) will access OOR
    delta = V_x_bias * b_bias(:,i); % eqn pg 365 diagram

    % Adding bias to state. Look at fig 2 on pg 365
    X(:,i)  = X(:,i) + delta;


    % For next loop
    % Compute value of f(x,u) for these states
    func = [
        cos(theta)*cos(phi)*Vx+(sin(psi)*sin(theta)*cos(phi)-cos(psi)*sin(phi))*Vy+(-cos(psi)*sin(theta)*cos(phi)+sin(psi)*sin(phi))*Vz;
        cos(theta)*sin(phi)*Vx+(sin(psi)*sin(theta)*sin(phi)+cos(psi)*cos(phi))*Vy+(cos(psi)*sin(theta)*sin(phi)-sin(psi)*cos(phi))*Vz;
        -sin(theta)*Vx+sin(psi)*cos(theta)*Vy+cos(psi)*cos(theta)*Vz;
        0;
        0;
        0;
        Kgpsi*wpsi+sin(psi)*tan(theta)*(Kgtheta*wtheta)+cos(psi)*tan(theta)*(Kgphi*wphi);
        cos(psi)*(Kgtheta*wtheta)+sin(psi)*(Kgphi*wphi);
        sin(psi)/cos(theta)*(Kgtheta*wtheta)+cos(psi)/cos(theta)*(Kgphi*wphi);
        ];
    % Approximate next states of the system
    %     X(:,i+1) = A * X(:,i);
    X(:,i+1) = X(:,i) + ts * func;

    %Aposteri
    T = (eye(9) - K*C) * P;
    % Update P for next estimate
    P = (A * T * A') + Q  ;

    %Update U_x_bias for next iteration
    U_x_bias = ( A * V_x_bias) + B_bias; % refer eqn 58

end % ends loop => for i = 1:ns-1

% Plotting relevant Graphs
figure('Position',[1,8, 0.90*scrsz(3), 0.90*scrsz(4)])
subplot(1,3,1),plot3(X(1,:),X(2,:),X(3,:));grid on;title('Position, Inertial Frame');xlabel('x');ylabel('y');zlabel('z');
subplot(1,3,2),plot(X(1,:),X(2,:));grid on;title('Position, Inertial frame X-Y');xlabel('x');ylabel('y');
subplot(1,3,3),plot(X(1,:),X(3,:));grid on;title('Position, Inertial frame X-Z');xlabel('x');ylabel('z');

figure('Position',[1,8, 0.80*scrsz(3), 0.80*scrsz(4)])
subplot(2,2,1),plot(t,X(4,:),'r',t,X(5,:),'b',t,X(6,:),'g');grid on;title('Linear Velocities vs Time');legend('Vx','Vy','Vz',1);
subplot(2,2,2),plot(t,X(7,:),'r',t,X(8,:),'b',t,X(9,:),'g');grid on;title('Pitch-Roll-Yaw vs Time');legend('\psi','\theta','\phi',1);
subplot(2,2,3),plot(t,X(1,:),'r',t,X(2,:),'b',t,X(3,:),'g');grid on;title('Position in Inertial frame vs Time');legend('x','y','z',1);