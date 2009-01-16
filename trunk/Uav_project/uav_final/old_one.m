% OLD one
clear all;

% Declaring all constants
n = 5;       % Number of laps
g = 9.8;      % Gravity constant
r = 50;       % Radius
h = 25;       % Height
toutput = 1;  % Time period for availability of Output Sample
estrate = 10; % Number of estimates before Output is available
w = 1/25;     % Angular velocity
snr = 65;     % Signal to noise ratio
% Accelorometer Gains
Kax=1;
Kay=1;
Kaz=1;
% Gyro Gains
Krx=1;
Kry=1;
Krz=1;

% Screen size for graph
scrsz = get(0,'ScreenSize');

% Calculating in-program constants
ts = toutput/estrate;     % Sampling Rate => GPS reading rate/number of estimates before GPS reading is available
tt = round( ( ( 2 * pi ) / w ) * n ); % Total time => Total angle of circle / Angular Velocity * number of laps
t = 0:ts:tt; % Discrete time instant
ns = size(t,2); % Number of Samples

R = 0.001 * eye(3);
Q = 0.001 * eye(6);
Q2 = 0.001 * eye(12);

% Initialise states
X = [0;0;0;0;0;0;0;0;0;0;0;0];

% sensor inputs
u = [-r*(w^2)*(t>=0);(t<0);g*(t>=0);(t<0);(t<0);w*(t>=0)];

% Initialise P
P = 0*eye(12);
P1 = P;

C = [ 1 0 0 0 0 0 0 0 0 0 0 0;
    0 0 1 0 0 0 0 0 0 0 0 0;
    0 0 0 1 0 0 0 0 0 0 0 0];

% Initialise Gain
L = zeros(12,3);

%T => t
%NN => ns
ONE = ones(1,ns);
Y = [r*cos(w*t);r*sin(w*t);h.*ONE];
N_temp = 1;

for i=1:ns
    x = X(1,i);
    y = X(2,i);
    z = X(3,i);
    Vx = X(4,i);
    Vy = X(5,i);
    Vz = X(6,i);
    rx = X(7,i);
    ry = X(8,i);
    rz = X(9,i);
    Bx = X(10,i);
    By = X(11,i);
    Bz = X(12,i);
    Ax = u(1,i);
    Ay = u(2,i);
    Az = u(3,i);
    wx = u(4,i);
    wy = u(5,i);
    wz = u(6,i);

    % Compute A
    A = [0,0,0,cos(ry)*cos(rz),sin(rx)*sin(ry)*cos(rz)-cos(rx)*sin(rz),cos(rx)*sin(ry)*cos(rz)+sin(rx)*sin(rz),(cos(rx)*sin(ry)*cos(rz)+sin(rx)*sin(rz))*Vy+(-sin(rx)*sin(ry)*cos(rz)+cos(rx)*sin(rz))*Vz,-sin(ry)*cos(rz)*Vx+sin(rx)*cos(ry)*cos(rz)*Vy+cos(rx)*cos(ry)*cos(rz)*Vz,-cos(ry)*sin(rz)*Vx+(-sin(rx)*sin(ry)*sin(rz)-cos(rx)*cos(rz))*Vy+(-cos(rx)*sin(ry)*sin(rz)+sin(rx)*cos(rz))*Vz,0,0,0;
        0,0,0,cos(ry)*sin(rz),sin(rx)*sin(ry)*sin(rz)+cos(rx)*cos(rz),cos(rx)*sin(ry)*sin(rz)-sin(rx)*cos(rz),(cos(rx)*sin(ry)*sin(rz)-sin(rx)*cos(rz))*Vy+(-sin(rx)*sin(ry)*sin(rz)-cos(rx)*cos(rz))*Vz,-sin(ry)*sin(rz)*Vx+sin(rx)*cos(ry)*sin(rz)*Vy+cos(rx)*cos(ry)*sin(rz)*Vz,cos(ry)*cos(rz)*Vx+(sin(rx)*sin(ry)*cos(rz)-cos(rx)*sin(rz))*Vy+(cos(rx)*sin(ry)*cos(rz)+sin(rx)*sin(rz))*Vz,0,0,0;
        0,0,0,-sin(ry),sin(rx)*cos(ry),cos(rx)*cos(ry),cos(rx)*cos(ry)*Vy-sin(rx)*cos(ry)*Vz,-cos(ry)*Vx-sin(rx)*sin(ry)*Vy-cos(rx)*sin(ry)*Vz,0,0,0,0;
        0,0,0,0,wz*Kax,-wy*Kax,0,g*cos(ry)*Kax,0,0,0,0;
        0,0,0,-wz*Kay,0,wx*Kay,-g*cos(ry)*cos(rx)*Kay,g*sin(ry)*sin(rx)*Kay,0,0,0,0;
        0,0,0,wy*Kaz,-wx*Kaz,0,g*cos(ry)*sin(rx)*Kaz,g*sin(ry)*cos(rx)*Kaz,0,0,0,0;
        0,0,0,0,0,0,cos(rx)*tan(ry)*(Kry*wy+By)-sin(rx)*tan(ry)*(Krz*wz+Bz),sin(rx)*(1+tan(ry)^2)*(Kry*wy+By)+cos(rx)*(1+tan(ry)^2)*(Krz*wz+Bz),0,1,sin(rx)*tan(ry),cos(rx)*tan(ry);
        0,0,0,0,0,0,-sin(rx)*(Kry*wy+By)-cos(rx)*(Krz*wz+Bz),0,0,0,cos(rx),-sin(rx);
        0,0,0,0,0,0,cos(rx)/cos(ry)*(Kry*wy+By)-sin(rx)/cos(ry)*(Krz*wz+Bz),sin(rx)/cos(ry)^2*(Kry*wy+By)*sin(ry)+cos(rx)/cos(ry)^2*(Krz*wz+Bz)*sin(ry),0,0,sin(rx)/cos(ry),cos(rx)/cos(ry);
        0,0,0,0,0,0,0,0,0,0,0,0;
        0,0,0,0,0,0,0,0,0,0,0,0;
        0,0,0,0,0,0,0,0,0,0,0,0];

    % Compute B
    B= [ 0,0,0,0,0,0;
        0,0,0,0,0,0;
        0,0,0,0,0,0;
        Kax,0,0,0,-Vz*Kax,Vy*Kax;
        0,Kay,0,Vz*Kay,0,-Vx*Kay;
        0,0,Kaz,-Vy*Kaz,Vx*Kaz,0;
        0,0,0,Krz,sin(rx)*tan(ry)*Kry,cos(rx)*tan(ry)*Krz;
        0,0,0,0,cos(rx)*Kry,-sin(rx)*Krz;
        0,0,0,0,sin(rx)/cos(ry)*Kry,cos(rx)/cos(ry)*Krz;
        0,0,0,0,0,0;
        0,0,0,0,0,0;
        0,0,0,0,0,0];

    % Propagate equations between measurements

    for j = 0:(ns-1)
        x = X(1,i);
        y = X(2,i+j);
        z = X(3,i+j);
        Vx = X(4,i+j);
        Vy = X(5,i+j);
        Vz = X(6,i+j);
        rx = X(7,i+j);
        ry = X(8,i+j);
        rz = X(9,i+j);
        Bx = X(10,i+j);
        By = X(11,i+j);
        Bz = X(12,i+j);
        Ax = u(1,i+j);
        Ay = u(2,i+j);
        Az = u(3,i+j);
        wx = u(4,i+j);
        wy = u(5,i+j);
        wz = u(6,i+j);

        f = [ cos(ry)*cos(rz)*Vx+(sin(rx)*sin(ry)*cos(rz)-cos(rx)*sin(rz))*Vy+(cos(rx)*sin(ry)*cos(rz)+sin(rx)*sin(rz))*Vz;
            cos(ry)*sin(rz)*Vx+(sin(rx)*sin(ry)*sin(rz)+cos(rx)*cos(rz))*Vy+(cos(rx)*sin(ry)*sin(rz)-sin(rx)*cos(rz))*Vz;
            -sin(ry)*Vx+sin(rx)*cos(ry)*Vy+cos(rx)*cos(ry)*Vz;
            (Ax+Vy*wz-Vz*wy+g*sin(ry))*Kax;
            (Ay-Vx*wz+Vz*wx-g*cos(ry)*sin(rx))*Kay;
            (Az+Vx*wy-Vy*wx-g*cos(ry)*cos(rx))*Kaz;
            Krx*wx+Bx+sin(rx)*tan(ry)*(Kry*wy+By)+cos(rx)*tan(ry)*(Krz*wz+Bz);
            cos(rx)*(Kry*wy+By)-sin(rx)*(Krz*wz+Bz);
            sin(rx)/cos(ry)*(Kry*wy+By)+cos(rx)/cos(ry)*(Krz*wz+Bz);
            0;
            0;
            0];
        % Compute predicted estimate
        X(:,N_temp+1)  = X(:,N_temp) + (ts)*(f);
        N_temp = N_temp+1;

        % Solve Ricati
        P =A * P1 * A' + B*Q*B';

    end % inner
    % Compute Kalman gain
    L = P*C' * inv(R+C*P*C');
    P1 = (eye(12)-L*C)*P;
    % correct states
    X(:,N_temp) = X(:,N_temp) + L * (Y(:,i)-C*X(:,N_temp));
end % outer

figure('Position',[1,8, 0.90*scrsz(3), 0.90*scrsz(4)])
subplot(1,3,1),plot3(X(1,:),X(2,:),X(3,:));grid on;title('Position, Inertial Frame');xlabel('x');ylabel('y');zlabel('z');
subplot(1,3,2),plot(X(1,:),X(2,:));grid on;title('Position, Inertial frame X-Y');xlabel('x');ylabel('y');
subplot(1,3,3),plot(X(1,:),X(3,:));grid on;title('Position, Inertial frame X-Z');xlabel('x');ylabel('z');

figure('Position',[1,8, 0.80*scrsz(3), 0.80*scrsz(4)])
subplot(2,2,1),plot(t,X(4,:),'r',t,X(5,:),'b',t,X(6,:),'g');grid on;title('Linear Velocities vs Time');legend('Vx','Vy','Vz',1);
subplot(2,2,2),plot(t,X(7,:),'r',t,X(8,:),'b',t,X(9,:),'g');grid on;title('Pitch-Roll-Yaw vs Time');legend('\psi','\theta','\phi',1);
subplot(2,2,3),plot(t,X(1,:),'r',t,X(2,:),'b',t,X(3,:),'g');grid on;title('Position in Inertial frame vs Time');legend('x','y','z',1);



