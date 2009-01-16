% No bias 9 state

clear all;

% Declaring all constants
n = 10;       % Number of laps
g = 9.8;      % Gravity constant
rad = 35;       % Radius
h = 10;       % Height
toutput = 1;  % Time period for availability of Output Sample
estrate = 10; % Number of estimates before Output is available
ang_vel = 1/25;     % Angular velocity

% Screen size for graph
scrsz = get(0,'ScreenSize');

% Calculating in-program constants
ts = toutput/estrate;     % Sampling Rate => GPS reading rate/number of estimates before GPS reading is available
tt = round( ( ( 2 * pi ) / ang_vel ) * n ); % Total time => Total angle of circle / Angular Velocity * number of laps
t = 0:ts:tt; % Discrete time instant
ns = size(t,2); % Number of Samples

% Initialising States
X = zeros(9,1);
% Initailising Input
U = zeros(3,1);
% Initialising Output
Y = zeros(6,1);
% Initialise C
C = zeros(6,9);

% Initialise P, Q, R, K, A, B
P = zeros(9);
Pnext = zeros(9);
Q = eye(9);
R = eye(6);
K = zeros(12,6);
A = zeros(9,9);
B = zeros(9,3);

for i = 1:ns-1
    % Create U
    % U = [ax,ay,az,p,q,r]
    %     U = [-((rad*ang_vel)^2/rad)*cos(ang_vel*i*ts);-((rad*ang_vel)^2/rad)*cos(ang_vel*i*ts);-g;0;0;ang_vel];
    U = [0;0;ang_vel];
    % Retrieve State Variables for this iteration
    x = X(1,i);
    y = X(2,i);
    z = X(3,i);
    u = X(4,i);
    v = X(5,i);
    w = X(6,i);
    phi = X(7,i);
    theta = X(8,i);
    psi = X(9,i);

    % Input Values for Calculations
%     ax = U(1,1);
%     ay = U(2,1);
%     az = U(3,1);
    p = U(1,1);
    q = U(2,1);
    r = U(3,1);

    % Compute Value for A
    A = [
        0,0,0,cos(theta)*cos(psi),(sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi)),(cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi)),((cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi))*v+(-sin(phi)*sin(theta)*cos(psi)+cos(phi)*sin(psi))*w),((-sin(theta)*cos(psi))*u+(sin(phi)*cos(theta)*cos(psi))*v+(cos(phi)*cos(theta)*cos(psi))*w),((-cos(theta)*sin(psi))*u+(-sin(phi)*sin(theta)*sin(psi)-cos(phi)*cos(psi))*v+(-cos(phi)*sin(theta)*sin(psi)+sin(phi)*cos(psi))*w);
        0,0,0,cos(theta)*sin(psi),(sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi)),(cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi)),((cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi))*v+(-sin(phi)*sin(theta)*sin(psi)-cos(phi)*cos(psi))*w),((-sin(theta)*sin(psi))*u+(sin(phi)*cos(theta)*sin(psi))*v+(cos(phi)*cos(theta)*sin(psi))*w),((cos(theta)*cos(psi))*u+(sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi))*v+(cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi))*w);
        0,0,0,-sin(theta),sin(phi)*cos(theta),cos(phi)*cos(theta),((cos(phi)*cos(theta))*v-(sin(phi)*cos(theta))*w),(-cos(theta)*u-sin(phi)*sin(theta)*v-cos(phi)*sin(theta)*w),0;
        %         0,0,0,0,r,-q,0,-g*cos(theta),0;
        %         0,0,0,-r,0,p,g*cos(theta)*cos(phi),-g*sin(theta)*sin(phi),0;
        %         0,0,0,q,-p,0,-g*cos(theta)*sin(phi),-g*sin(theta)*cos(phi),0;
        0,0,0,0,0,0,0,0,0;
        0,0,0,0,0,0,0,0,0;
        0,0,0,0,0,0,0,0,0;
        0,0,0,0,0,0,(q*cos(phi)*tan(theta)-r*sin(phi)*tan(theta)),(q*sin(phi)*(sec(theta)^2)+r*cos(phi)*(sec(theta)^2)),0;
        %0,0,0,0,0,0,(-q*sin(phi)+r*cos(phi)),0,0;
        0,0,0,0,0,0,(-q*sin(phi)-r*cos(phi)),0,0;
       % 0,0,0,0,0,0,((q*cos(phi)/cos(theta))-(r*sin(phi)/cos(theta))),((q*sin(phi)*(-1/(1-theta^2)^0.5))+(r*cos(phi)*(-1/(1-theta^2)^0.5))),0;
        0,0,0,0,0,0,(q*cos(phi)*sec(theta)-r*sin(phi)*sec(theta)),(q*sin(phi)*sec(theta)*tan(theta)+r*cos(phi)*sec(theta)*tan(theta)),0;
        ];
 
    % Compute Value for B
    %     B = [
    %         0,0,0,0,0,0;
    %         0,0,0,0,0,0;
    %         0,0,0,0,0,0;
    %         g,0,0,0,-w,r;
    %         0,g,0,w,0,-u;
    %         0,0,g,-v,u,0;
    %                 0,0,0,0,0,0;
    %                 0,0,0,0,0,0;
    %                 0,0,0,0,0,0;
    %         0,0,0,1,sin(phi)*tan(theta),cos(phi)*tan(theta);
    %         0,0,0,0,cos(phi),sin(phi);
    %         0,0,0,0,sin(phi)/cos(theta),cos(phi)/cos(theta);
    %         ];

    B = [
        0,0,0;
        0,0,0;
        0,0,0;
        0,0,0;
        0,0,0;
        0,0,0;
        1,sin(phi)*tan(theta),cos(phi)*tan(theta);
       % 0,cos(phi),sin(phi);
        0,cos(phi),-sin(phi);
        0,sin(phi)/cos(theta),cos(phi)/cos(theta);
        ];

    % Correct this state when output is available
    if i ~= 1 % Except case of i = 1 from last if

        % If GPS reading not available plugin the estimated value
        if rem( i, estrate ) == 0
%             Yx = r*cos(ang_vel*i*ts);
%             Yy = r*sin(ang_vel*i*ts);
%             Yz = h;
        Yx = r*cos(w*t(1,i));
        Yy = r*sin(w*t(1,i));
        Yz = h;
        else
            Yx = X(1,i);
            Yy = X(2,i);
            Yz = X(3,i);
        end

        % Generate Output
        % Y = [x,y,z,ax,ay,az]
        Y = [ Yx;Yy;Yz;-((rad*ang_vel)^2/rad)*cos(ang_vel*i*ts);-((rad*ang_vel)^2/rad)*cos(ang_vel*i*ts);-g];
        
        C = [
            1,0,0,0,0,0,0,0,0;
            0,1,0,0,0,0,0,0,0;
            0,0,1,0,0,0,0,0,0;
            0,0,0,0,-r/g,q/g,0,cos(theta),0;
          %  0,0,0,r/g,0,-p/g,-cos(theta)*cos(phi),sin(theta)*sin(phi),0;
          0,0,0,r/g,0,-p/g,cos(theta)*cos(phi),sin(theta)*sin(phi),0;
            0,0,0,-q/g,p/g,0,cos(theta)*sin(phi),sin(theta)*cos(phi),0;
            ];
        
%                C = [1,0,0,0,0,0,0,0,0,0,0,0,0,0,0;
%                   0,1,0,0,0,0,0,0,0,0,0,0,0,0,0;
%                   0,0,1,0,0,0,0,0,0,0,0,0,0,0,0;
%                    0,0,0,0,-(wphi+Bgphi)/g,(wtheta+Bgtheta)/g,0,cos(theta),0,1,0,0,0,Vz/g,-Vy/g;
%                    0,0,0,(wphi+Bgphi)/g,0,-(wpsi+Bgpsi)/g,cos(psi)*cos(theta),sin(psi)*sin(theta),0,0,1,0,-Vz/g,0,Vx/g;               
%                    0,0,0,-(wtheta+Bgtheta)/g,(wpsi+Bgpsi)/g,0,sin(psi)*cos(theta),cos(psi)*sin(theta),0,0,0,1,Vy/g,-Vx/g,0];   

        % Calculate Gain
        K = P * C' * inv( (C * P * C') + R);

        % Correct State Estimate
        X(:,i) = X(:,i) + K * ( Y - C*X(:,i) );

        % Pnext for next set of estimates
        Pnext  = (eye(9) - K*C) * P;
    end % ends if for this state correction

    % Compute value of f(x,u) for these states
    func = [
        cos(theta)*cos(psi)*u+(sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi))*v+(cos(phi)*sin(theta)*cos(psi)+sin(phi)*sin(psi))*w;
        cos(theta)*sin(psi)*u+(sin(phi)*sin(theta)*sin(psi)+cos(phi)*cos(psi))*v+(cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi))*w;
        -sin(theta)*u+sin(phi)*cos(theta)*v+cos(phi)*cos(theta)*w;
        %         -q*w+r*v-g*sin(theta)+g*(-((rad*ang_vel)^2/rad)*cos(ang_vel*i*ts));
        %         -r*u+p*w+g*cos(theta)*sin(phi)+g*(-((rad*ang_vel)^2/rad)*cos(ang_vel*i*ts));
        %         -p*v+q*u+g*cos(theta)*cos(phi)+g*(-g);
        0;
        0;
        0;
        p+q*sin(phi)*tan(theta)+r*cos(phi)*tan(theta);
        %q*cos(phi)+r*sin(phi);
        q*cos(phi)+r*sin(phi);
        (q*sin(phi)/cos(theta))+(r*cos(phi)/cos(theta));
        ];
    % Update P for next estimate
    P = (A * Pnext * A') + Q  ;

    % Estimate next states
    X(:,i+1) = X(:,i) + ( ts * func );

end
figure('Position',[1,8, 0.90*scrsz(3), 0.90*scrsz(4)])
subplot(1,3,1),plot3(X(1,:),X(2,:),X(3,:));grid on;title('Position, Inertial Frame');xlabel('x');ylabel('y');zlabel('z');
subplot(1,3,2),plot(X(1,:),X(2,:));grid on;title('Position, Inertial frame X-Y');xlabel('x');ylabel('y');
subplot(1,3,3),plot(X(1,:),X(3,:));grid on;title('Position, Inertial frame X-Z');xlabel('x');ylabel('z');