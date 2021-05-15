clear all
clc

global gridN
gridN = 47;

tic
% Minimize the simulation time
time_min = @(x) x(1);
% The initial parameter guess; 1 second, gridN positions, gridN velocities,
% gridN accelerations
x0 = [1; linspace(0,1,gridN)'; linspace(0,1,gridN)'; ones(gridN, 1) * 5];

x_d = [0.686648,0.686989,0.686382,0.684392,0.680017,0.673863,0.666126,0.656892,0.64619,0.634054,0.620525,0.60565,0.589481,0.572077,0.5535,0.533817,0.5131,0.491425,0.468905,0.446197,0.425647,0.410777,0.401854,0.4,0.4,0.398582,0.396657,0.39663,0.398485,0.402066,0.406186,0.409796,0.412899,0.415495,0.417588,0.41918,0.420275,0.420877,0.420992,0.420625,0.41978,0.418458,0.416129,0.412593,0.407922,0.402305,0.4];
x_d_dot = [0,0.00453162,-0.104261,-0.311633,-0.53569,-0.698041,-0.848959,-0.997348,-1.14257,-1.284,-1.42103,-1.5531,-1.67964,-1.80016,-1.91415,-2.02119,-2.12087,-2.21281,-2.27342,-2.26643,-1.77699,-1.20919,-0.523804,0,0,-0.277937,-0.108149,0.093802,0.274862,0.437492,0.386509,0.335626,0.284904,0.234403,0.184182,0.134299,0.084813,0.0357817,-0.0127383,-0.060691,-0.108021,-0.166874,-0.2958,-0.409674,-0.527539,-0.527656,0];
y_d = [0.109227,0.113075,0.124624,0.143862,0.170088,0.198085,0.226175,0.253857,0.281038,0.307629,0.333541,0.358689,0.382993,0.406377,0.428767,0.450097,0.470303,0.489329,0.507115,0.523547,0.537103,0.545606,0.549505,0.550312,0.550312,0.553071,0.559256,0.567152,0.576847,0.588304,0.600541,0.612622,0.624529,0.636248,0.64776,0.659051,0.670104,0.680904,0.691434,0.701681,0.711628,0.721253,0.72992,0.737334,0.743504,0.748508,0.750312];
y_d_dot = [0,0.76989,1.54028,2.30549,2.77097,2.82625,2.79017,2.74471,2.69009,2.62655,2.55439,2.47396,2.38565,2.28987,2.18708,2.07778,1.96249,1.84176,1.71137,1.57756,1.09592,0.600124,0.225551,0,0,0.544174,0.693542,0.879876,1.05862,1.23099,1.21618,1.1997,1.18158,1.16182,1.14043,1.11745,1.09289,1.06676,1.03911,1.00994,0.979301,0.933373,0.802414,0.681211,0.547748,0.415436,0];
z_d = [0.33617,0.331296,0.322159,0.313165,0.304177,0.295197,0.286228,0.277272,0.268331,0.259409,0.250507,0.241627,0.232774,0.223948,0.215153,0.206391,0.197664,0.188976,0.180328,0.171726,0.163175,0.154646,0.146624,0.144117,0.144117,0.142549,0.137797,0.131646,0.129004,0.130229,0.133415,0.136575,0.139703,0.142797,0.145853,0.148867,0.151836,0.154756,0.157623,0.160435,0.163188,0.16584,0.165593,0.160685,0.151876,0.145804,0.144117];
z_d_dot = [0,0.00453162,-0.104261,-0.311633,-0.53569,-0.698041,-0.848959,-0.997348,-1.14257,-1.284,-1.42103,-1.5531,-1.67964,-1.80016,-1.91415,-2.02119,-2.12087,-2.21281,-2.27342,-2.26643,-1.77699,-1.20919,-0.523804,0,0,-0.277937,-0.108149,0.093802,0.274862,0.437492,0.386509,0.335626,0.284904,0.234403,0.184182,0.134299,0.084813,0.0357817,-0.0127383,-0.060691,-0.108021,-0.166874,-0.2958,-0.409674,-0.527539,-0.527656,0];

x_d_dot = [0];
for i=2:47
   x_d_dot = [x_d_dot,  (x_d(i) - x_d(i-1))/0.01];
end

x_d_dot_dot = [0];
for i=2:47
   x_d_dot_dot = [x_d_dot_dot,  (x_d_dot(i) - x_d_dot(i-1))/0.01];
end


y_d_dot = [0];
for i=2:47
   y_d_dot = [y_d_dot,  (y_d(i) - y_d(i-1))/0.01];
end

y_d_dot_dot = [0];
for i=2:47
   y_d_dot_dot = [y_d_dot_dot,  (y_d_dot(i) - y_d_dot(i-1))/0.01];
end


z_d_dot = [0];
for i=2:47
   z_d_dot = [z_d_dot,  (z_d(i) - z_d(i-1))/0.01];
end

z_d_dot_dot = [0];
for i=2:47
   z_d_dot_dot = [z_d_dot_dot,  (z_d_dot(i) - z_d_dot(i-1))/0.01];
end


x0 = [length(x_d)*0.01; x_d'; y_d'; z_d'; 
                x_d_dot'; y_d_dot'; z_d_dot';
                x_d_dot_dot'; y_d_dot_dot'; z_d_dot_dot'];
% No linear inequality or equality constraints
A = [];
b = [];
Aeq = [];
Beq = [];
% Lower bound the simulation time at zero seconds, and bound the
% accelerations between -10 and 30
lb = [0;    ones(gridN*2, 1) * -Inf;
            ones(gridN, 1) * 0.15;
            ones(gridN*3, 1) * -1;
            ones(gridN*3, 1) * -50];
        
ub = [Inf;  ones(gridN*3, 1) * Inf; 
            ones(gridN*3, 1) * 1;
            ones(gridN*3, 1) * 50];
        
% Options for fmincon
options = optimoptions(@fmincon, 'TolFun', 0.00000001, 'MaxIter', 10000, ...
                       'MaxFunEvals', 100000, 'Display', 'iter', ...
                       'DiffMinChange', 0.001, 'Algorithm', 'sqp');
% Solve for the best simulation time + control input
optimal = fmincon(time_min, x0, A, b, Aeq, Beq, lb, ub, ...
              @constraints_obs, options);

% Discretize the times
sim_time = optimal(1);
delta_time = sim_time / gridN;
times = 0 : delta_time : sim_time - delta_time;
% Get the state + accelerations (control inputs) out of the vector
positions = optimal(2             : 1 + gridN * 3);
vels      = optimal(2 + gridN * 3 : 1 + gridN * 6);
accs      = optimal(2 + gridN * 6 : end);



time_min = @(x) x(1);
x0 = [length(x_d)*0.01; x_d'; y_d'; z_d'; 
                x_d_dot'; y_d_dot'; z_d_dot';
                x_d_dot_dot'; y_d_dot_dot'; z_d_dot_dot'];
% No linear inequality or equality constraints
A = [];
b = [];
Aeq = [];
Beq = [];
% Lower bound the simulation time at zero seconds, and bound the
% accelerations between -10 and 30
lb = [0;    ones(gridN*2, 1) * -Inf;
            ones(gridN, 1) * 0.15;
            ones(gridN*3, 1) * -2;
            ones(gridN*3, 1) * -50];
        
ub = [Inf;  ones(gridN*3, 1) * Inf; 
            ones(gridN*3, 1) * 2;
            ones(gridN*3, 1) * 50];
        

optimal2 = fmincon(time_min, x0, A, b, Aeq, Beq, lb, ub, ...
              @constraints_no_obs, options);
positions_no = optimal2(2             : 1 + gridN * 3);
vels_no      = optimal2(2 + gridN * 3 : 1 + gridN * 6);
accs_no      = optimal2(2 + gridN * 6 : end);


% Make the plots
figure(1)
hold on;
subplot(3, 3, 1)
plot(times,positions(1:gridN));
xlabel('Time [s]');
ylabel ('Joint position [rad]')
title('Desired q1 trajectory')
subplot(3, 3, 4)
plot(times,vels(1:gridN));
xlabel('Time [s]');
ylabel ('Joint speed [rad/s]')
title('Desired q1_dot trajectory')
subplot(3, 3, 7)
plot(times,accs(1:gridN));
xlabel('Time [s]');
ylabel ('Joint accel [rad/s^2]')
title('Desired q1_dot_dot trajectory')

subplot(3, 3, 2)
plot(times,positions(1+gridN:gridN*2));
xlabel('Time [s]');
ylabel ('Joint position [rad]')
title('Desired q1 trajectory')
subplot(3, 3, 5)
plot(times,vels(1+gridN:gridN*2));
xlabel('Time [s]');
ylabel ('Joint speed [rad/s]')
title('Desired q1_dot trajectory')
subplot(3, 3, 8)
plot(times,accs(1+gridN:gridN*2));
xlabel('Time [s]');
ylabel ('Joint accel [rad/s^2]')
title('Desired q1_dot_dot trajectory')

subplot(3, 3, 3)
plot(times,positions(1+gridN*2:gridN*3));
xlabel('Time [s]');
ylabel ('Joint position [rad]')
title('Desired q1 trajectory')
subplot(3, 3, 6)
plot(times,vels(1+gridN*2:gridN*3));
xlabel('Time [s]');
ylabel ('Joint speed [rad/s]')
title('Desired q1_dot trajectory')
subplot(3, 3, 9)
plot(times,accs(1+gridN*2:gridN*3));
xlabel('Time [s]');
ylabel ('Joint accel [rad/s^2]')
title('Desired q1_dot_dot trajectory')

figure(2)

[X,Y,Z] = sphere;
r = 0.05;
X2 = X * r;
Y2 = Y * r;
Z2 = Z * r;

surf(X2+0.4,Y2+0.6,Z2+0.15)

figure(3)

subplot(1, 3, 1)

p = plot3(x_d(1:gridN), y_d(1:gridN), z_d(1:gridN));
p.LineWidth = 3;
grid on
xlabel("X")
ylabel("Y")
zlabel("Z")
title(["Initial seed trajectory", "Trajectory time = 0.47s"])
hold on
p = plot3(x_d(1), y_d(1), z_d(1), '-or');
p.LineWidth = 3;
p = plot3(x_d(end), y_d(end), z_d(end), '-og');
p.LineWidth = 3;

subplot(1, 3, 2)

p = plot3(positions_no(1:gridN), positions_no(1+gridN:gridN*2), positions_no(1+gridN*2:gridN*3));
p.LineWidth = 3;
grid on
xlabel("X")
ylabel("Y")
zlabel("Z")
title(["Optimized trajectory, no obstacle", "Trajectory time = " + num2str(optimal2(1)) + "s"])
hold on
p = plot3(positions_no(1), positions_no(1+gridN), positions_no(1+2*gridN), '-or');
p.LineWidth = 3;
p = plot3(positions_no(gridN), positions_no(2*gridN), positions_no(3*gridN), '-og');
p.LineWidth = 3;

subplot(1, 3, 3)
p = plot3(positions(1:gridN), positions(1+gridN:gridN*2), positions(1+gridN*2:gridN*3));
p.LineWidth = 3;
grid on
xlabel("X")
ylabel("Y")
zlabel("Z")
title(["Optimized trajectory, with obstacle", "Trajectory time = " + num2str(optimal(1)) + "s"])
hold on
p = plot3(positions(1), positions(1+gridN), positions(1+2*gridN), '-or');
p.LineWidth = 3;
p = plot3(positions(gridN), positions(2*gridN), positions(3*gridN), '-og');
p.LineWidth = 3;

hold on 
surf(X2+0.4,Y2+0.6,Z2+0.15)

figure(4)

hold on;
subplot(3, 3, 1)
plot(times,positions_no(1:gridN));
xlabel('Time [s]');
ylabel ('Joint position [rad]')
title('Desired q1 trajectory')
subplot(3, 3, 4)
plot(times,vels_no(1:gridN));
xlabel('Time [s]');
ylabel ('Joint speed [rad/s]')
title('Desired q1_dot trajectory')
subplot(3, 3, 7)
plot(times,accs_no(1:gridN));
xlabel('Time [s]');
ylabel ('Joint accel [rad/s^2]')
title('Desired q1_dot_dot trajectory')

subplot(3, 3, 2)
plot(times,positions_no(1+gridN:gridN*2));
xlabel('Time [s]');
ylabel ('Joint position [rad]')
title('Desired q1 trajectory')
subplot(3, 3, 5)
plot(times,vels_no(1+gridN:gridN*2));
xlabel('Time [s]');
ylabel ('Joint speed [rad/s]')
title('Desired q1_dot trajectory')
subplot(3, 3, 8)
plot(times,accs_no(1+gridN:gridN*2));
xlabel('Time [s]');
ylabel ('Joint accel [rad/s^2]')
title('Desired q1_dot_dot trajectory')

subplot(3, 3, 3)
plot(times,positions_no(1+gridN*2:gridN*3));
xlabel('Time [s]');
ylabel ('Joint position [rad]')
title('Desired q1 trajectory')
subplot(3, 3, 6)
plot(times,vels_no(1+gridN*2:gridN*3));
xlabel('Time [s]');
ylabel ('Joint speed [rad/s]')
title('Desired q1_dot trajectory')
subplot(3, 3, 9)
plot(times,accs_no(1+gridN*2:gridN*3));
xlabel('Time [s]');
ylabel ('Joint accel [rad/s^2]')
title('Desired q1_dot_dot trajectory')

disp(sprintf('Finished in %f seconds', toc));


function [ c, ceq ] = constraints_obs( x )
    global gridN
    % No nonlinear inequality constraint needed
    c = [];
    % Calculate the timestep
    sim_time = x(1);
    delta_time = sim_time / gridN;
    % Get the states / inputs out of the vector
    positions = x(2             : 1 + gridN * 3);
    vels      = x(2 + gridN * 3 : 1 + gridN * 6);
    accs      = x(2 + gridN * 6 : end);
    
    % Constrain initial position and velocity to be zero
    ceq = [positions(1) - 0.686672; vels(1); positions(1+gridN) - 0.109074; positions(1+gridN*2) - 0.33617;
        vels(1); vels(1+gridN); vels(1+gridN*2)];
   
    
    for i = 1 : gridN - 1
        % The state at the beginning of the time interval
        x_i = [positions(i); vels(i)];
        y_i = [positions(i + gridN); vels(i + gridN)];
        z_i = [positions(i + gridN*2); vels(i + gridN*2)];
        % What the state should be at the start of the next time interval
        x_n = [positions(i+1); vels(i+1)];
        y_n = [positions(i+1 + gridN); vels(i+1 + gridN)];
        z_n = [positions(i+1 + gridN*2); vels(i+1 + gridN*2)];
        % The time derivative of the state at the beginning of the time
        % interval
        xdot_i = [vels(i); accs(i)];
        ydot_i = [vels(i + gridN); accs(i + gridN)];
        zdot_i = [vels(i + gridN*2); accs(i + gridN*2)];
        % The time derivative of the state at the end of the time interval
        xdot_n = [vels(i+1); accs(i+1)];
        ydot_n = [vels(i+1 + gridN); accs(i+1 + gridN)];
        zdot_n = [vels(i+1 + gridN*2); accs(i+1 + gridN*2)];
        % The end state of the time interval calculated using quadrature
        xend = x_i + delta_time * (xdot_i + xdot_n) / 2;
        yend = y_i + delta_time * (ydot_i + ydot_n) / 2;
        zend = z_i + delta_time * (zdot_i + zdot_n) / 2;
        % Constrain the end state of the current time interval to be
        % equal to the starting state of the next time interval
        ceq = [ceq ; x_n - xend; y_n - yend; z_n - zend];
        
        distance = sqrt((positions(i) - 0.4)^2 + (positions(i + gridN) - 0.6)^2 +...
            (positions(i + gridN*2) - 0.15)^2);
        
        c = [c; 0.05 - distance];
        
    end
    ceq = [ceq ; positions(25) - 0.4; positions(gridN + 25) - 0.45; positions(gridN*2 + 25) - 0.15];
    % Constrain end position to 1 and end velocity to 0
    ceq = [ceq ; positions(gridN) - 0.4; positions(gridN*2) - 0.75; positions(gridN*3) - 0.15;
                        vels(gridN); vels(gridN*2); vels(gridN*3)];
end

function [ c, ceq ] = constraints_no_obs( x )
    global gridN
    % No nonlinear inequality constraint needed
    c = [];
    % Calculate the timestep
    sim_time = x(1);
    delta_time = sim_time / gridN;
    % Get the states / inputs out of the vector
    positions = x(2             : 1 + gridN * 3);
    vels      = x(2 + gridN * 3 : 1 + gridN * 6);
    accs      = x(2 + gridN * 6 : end);
    
    % Constrain initial position and velocity to be zero
    ceq = [positions(1) - 0.686672; vels(1); positions(1+gridN) - 0.109074; positions(1+gridN*2) - 0.33617;
        vels(1); vels(1+gridN); vels(1+gridN*2)];
   
    
    for i = 1 : gridN - 1
        % The state at the beginning of the time interval
        x_i = [positions(i); vels(i)];
        y_i = [positions(i + gridN); vels(i + gridN)];
        z_i = [positions(i + gridN*2); vels(i + gridN*2)];
        % What the state should be at the start of the next time interval
        x_n = [positions(i+1); vels(i+1)];
        y_n = [positions(i+1 + gridN); vels(i+1 + gridN)];
        z_n = [positions(i+1 + gridN*2); vels(i+1 + gridN*2)];
        % The time derivative of the state at the beginning of the time
        % interval
        xdot_i = [vels(i); accs(i)];
        ydot_i = [vels(i + gridN); accs(i + gridN)];
        zdot_i = [vels(i + gridN*2); accs(i + gridN*2)];
        % The time derivative of the state at the end of the time interval
        xdot_n = [vels(i+1); accs(i+1)];
        ydot_n = [vels(i+1 + gridN); accs(i+1 + gridN)];
        zdot_n = [vels(i+1 + gridN*2); accs(i+1 + gridN*2)];
        % The end state of the time interval calculated using quadrature
        xend = x_i + delta_time * (xdot_i + xdot_n) / 2;
        yend = y_i + delta_time * (ydot_i + ydot_n) / 2;
        zend = z_i + delta_time * (zdot_i + zdot_n) / 2;
        % Constrain the end state of the current time interval to be
        % equal to the starting state of the next time interval
        ceq = [ceq ; x_n - xend; y_n - yend; z_n - zend];
        
    end
    ceq = [ceq ; positions(25) - 0.4; positions(gridN + 25) - 0.45; positions(gridN*2 + 25) - 0.15];
    % Constrain end position to 1 and end velocity to 0
    ceq = [ceq ; positions(gridN) - 0.4; positions(gridN*2) - 0.75; positions(gridN*3) - 0.15;
                        vels(gridN); vels(gridN*2); vels(gridN*3)];
end