function f = simulated_dynamics()
% EN.530.678: HW#4 sample
% 1) compute a reference path using a polynomial in flat output space
% 2) track the path using backstepping
%
% M. Kobilarov, Spring 2014


% boundary conditions in state space
x0 = [1; -1; 0.5; 1.1; -0.3; 1];
xf = [3;  6;   7;   5;   -2; 3];
T = 10;

%%%%%%%%% TRAJECTORY GENERATION %%%%%%%%%%%%%

% norm of initial and final velocity along desired path
% it determines how much the curves will bend
% and can be freely chosen
S.u1 = 0;

% boundary conditions in flat output space 
y0 = uni_h(x0);
yf = uni_h(xf);
dy0 = S.u1*[0; 0; 0; 0; 0; 0]; % desired starting velocity
dyf = S.u1*[0; 0; 0; 0; 0; 0]; % desired end velocity

% compute path coefficients
A = poly3_coeff(y0, dy0, yf, dyf, T);

% plot desired path
X = A*poly3([0:.01:T]);
% plot(X(1,:), X(2,:), '-r')
% hold on

S.dynamics = py.ur5.robot_config('../data');
S.dynamics.Mq([0, 0, 0, 0, 0, 0]);
S.dynamics.Cq([0, 0, 0, 0, 0, 0], [0, 0, 0, 0, 0, 0]);
S.dynamics.Mq_g([0, 0, 0, 0, 0, 0]);

%%%%%%%%% TRAJECTORY TRACKING %%%%%%%%%%%%%
S.A = A;

% gains
S.ko = [1, 0, 0, 0, 0, 0;
        0, 1, 0, 0, 0, 0;
        0, 0, 1, 0, 0, 0;
        0, 0, 0, 1, 0, 0;
        0, 0, 0, 0, 1, 0;
        0, 0, 0, 0, 0, 1];
S.k = [1, 0, 0, 0, 0, 0;
        0, 1, 0, 0, 0, 0;
        0, 0, 1, 0, 0, 0;
        0, 0, 0, 1, 0, 0;
        0, 0, 0, 0, 1, 0;
        0, 0, 0, 0, 0, 1];

% perturb initial condition
x = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
x = x';

% augmented state with dynamic compensator, i.e xi=u1
xa = [x];

% simulate system
[ts, xas] = ode45(@uni_ode, [0 T], xa, [], S);

% visualize
figure;
hold on;
plot([0:0.01:T], X(1,:), '-g');
plot(ts, xas(:,1), '-k');
legend('desired', 'executed');
title('Joint 1 Backstepping Control');
xlabel('Time (s)');
ylabel('Angle (rad)');
hold off;

figure;
hold on;
plot([0:0.01:T], X(2,:), '-g');
plot(ts, xas(:,2), '-k');
legend('desired', 'executed');
title('Joint 2 Backstepping Control');
xlabel('Time (s)');
ylabel('Angle (rad)');
hold off;

figure;
hold on;
plot([0:0.01:T], X(3,:), '-g');
plot(ts, xas(:,3), '-k');
legend('desired', 'executed');
title('Joint 3 Backstepping Control');
xlabel('Time (s)');
ylabel('Angle (rad)');
hold off;

figure;
hold on;
plot([0:0.01:T], X(4,:), '-g');
plot(ts, xas(:,4), '-k');
legend('desired', 'executed');
title('Joint 4 Backstepping Control');
xlabel('Time (s)');
ylabel('Angle (rad)');
hold off;

figure;
hold on;
plot([0:0.01:T], X(5,:), '-g');
plot(ts, xas(:,5), '-k');
legend('desired', 'executed');
title('Joint 5 Backstepping Control');
xlabel('Time (s)');
ylabel('Angle (rad)');
hold off;

figure;
hold on;
plot([0:0.01:T], X(6,:), '-g');
plot(ts, xas(:,6), '-k');
legend('desired', 'executed');
title('Joint 6 Backstepping Control');
xlabel('Time (s)');
ylabel('Angle (rad)');
hold off;

% plot(ts, xas(:,2), '-g');
% plot(ts, xas(:,3), '-b');
% plot(ts, xas(:,4), '-y');
% plot(ts, xas(:,5), '-m');
% plot(ts, xas(:,6), '-c');
% hold off;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function A = poly3_coeff(y0, dy0, yf, dyf, T)
% computes cubic curve connecting (y0,dy0) and (yf, dyf) at time T

Y = [y0, dy0, yf, dyf];
L = [poly3(0), dpoly3(0), poly3(T), dpoly3(T)];
A = Y*inv(L);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function y = uni_h(x)
% output function

y = x(1:6);


function f = poly3(t)
f = [t.^3; t.^2; t; ones(size(t))];

function f = dpoly3(t)
f = [3*t.^2; 2*t; ones(size(t)); zeros(size(t))];

function f = d2poly3(t)
f = [6*t; 2; zeros(size(t)); zeros(size(t))];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function ua = uni_ctrl(t, xa, S)
% tracking control law

% get desired outputs:
% yd = [1; 2; 3; 4; 5; 6];
% dyd = [0; 0; 0; 0; 0; 0];
% d2yd = [0; 0; 0; 0; 0; 0];
yd = S.A*poly3(t);
dyd = S.A*dpoly3(t);
d2yd = S.A*d2poly3(t);

% get current output
y = uni_h(xa);

% current velocity
dy = xa(7:12);

% errors
e = y - yd;
de = dy - dyd;

% z-state
z = S.ko*e + de;

M = double(S.dynamics.Mq(y));
C = double(S.dynamics.Cq(y, yd));
g = double(S.dynamics.Mq_g(y))';

ua = M*(d2yd - e - S.ko*de - S.k*z) + C*dy + g;


% augmented inputs ua=(du1, u2)
% ua = [s(1);
%       s(2)/u1];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function R = Rot(a)

R = [cos(a), -sin(a);
     sin(a), cos(a)];


function dxa = uni_ode(t, xa, S)
% unicycle ODE
ua = uni_ctrl(t, xa, S);

y = uni_h(xa);

dy = xa(7:12)

M = double(S.dynamics.Mq(y));
C = double(S.dynamics.Cq(y, yd))
g = double(S.dynamics.Mq_g(y))';

dxa = [xa(7);
       xa(8);
       xa(9);
       xa(10);
       xa(11);
       xa(12);
       M\(ua - C*dy - g)];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%