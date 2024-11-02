%% Multi Variable State Space Control Coursework
clear
close all
clc

% finding flutter onset speed
U = 1:0.0625:13;
b = [];
for i = 1:length(U)
[A,B,C,D,F] = mbuild(U(i));
syso=ss(A,B,eye(4),zeros(4,1));
[x,t] = impulse(syso,10);
hPos = abs(x(:,1));
f = fit(t,hPos,'exp1');
exp = f.b;
b = [b;exp];
end

u = negPos(U,b)

figure(1)
plot(U,b)
xline(u,'k --')
xticks(1:1:15)
xlabel('Speed (m/s)')
ylabel('Exponential Coefficient')
grid on

%% Impulse Response Open Loop

[A,B,C,D,F] = mbuild(20);
syso=ss(A,B,eye(4),zeros(4,1));
[x,t] = impulse(syso,10);

eigen = eig(syso)

figure(2)
subplot(2,1,1)
plot(t,x(:,1))
xticks(1:1:10)
xlabel('t (s)')
ylabel('Heave (m)')
grid on
subplot(2,1,2)
plot(t,x(:,2))
xticks(1:1:10)
xlabel('t (s)')
ylabel('{\alpha} (deg)')
grid on

figure(3)

subplot(2,1,1)
plot(t,x(:,3))
xticks(1:1:10)
xlabel('t (s)')
ylabel('Rate Heave (m)')
grid on
subplot(2,1,2)
plot(t,x(:,4))
xticks(1:1:10)
xlabel('t (s)')
ylabel(' Rate {\alpha} (deg)')
grid on
%% Exp Model Plotting


[A,B,C,D,F] = mbuild(5);
syso=ss(A,B,eye(4),zeros(4,1));
[x,t] = impulse(syso,10);
hPos = abs(x(:,1));
f = fit(t,hPos,'exp1');
exp = f.b;

figure(1)
%plot(t,x(:,1),'b')
plot(f,t,x(:,1))
grid on
xlabel('t (s)')
ylabel('Heave (m)')

%% Testing
clear
close all
clc

%design speed at U0
[A,B,C,D,F] = mbuild(11.4092);

w = 1;
Q = w*(C')*C;
% q1 = 10;
% q2 = 10;
% q3 = 10;
% q4 = 1;
% Q = [q1 0 0 0
%     0 q2 0 0
%     0 0 q3 0
%     0 0 0 q4];
R = 0.1;
K = lqr(A,B,Q,R);

%test speed
[A,B,C,D,F]=mbuild(257);

sysc=ss(A-B*K,F,C,D);
[x,t]=impulse(sysc,10);
xT = x.';
u=-(K([1 2])*(x.')).';

figure(2)
subplot(2,1,1)
plot(t,x(:,1),'k',t,u,'b--')
subplot(2,1,2)
plot(t,x(:,2),'k',t,u,'b--')

eigen = eig(A-B*K)


% figure(3)
% bode(syso([1 2]), sysc([1 2]))
% grid on

%% Controller Design Loop
clear
close all
clc

q1 = 0:0.5:5;
q2 = 5:0.5:15;

uM = zeros(length(q1),length(q2));

for i = 1:1:length(q1)
    for j = 1:1:length(q2)
        U0=11.4109; %open loop system marginal stability speed (11.4109)
        [A,B,C,D,F]=mbuild(U0); %build system at U0
        %build controller
        Q = [q1(i) 0 0 0;
            0 q2(j) 0 0; 
            0 0 0 0; 
            0 0 0 0]; 
        R = 2;
        K = lqr(A,B,Q,R);
        %speed values to test at
        U = 0:1:80;       
        b = zeros(1,length(U));
        bMat = zeros(length(U));
        
        for k = 1:1:length(U)
            [A,B,C,D,F]=mbuild(U(k));
            sysc=ss(A-B*K,F,C,D);
            [x,t]=impulse(sysc,4);

            hPos = abs(x(:,1));
            f = fit(t,hPos,'exp1');
            b(k) = f.b;
            
        end
        bMat(:,j) = b;
        uM(i,j) = negPos(U,b);
               
    end

end

figure(4)
surf(q1,q2,uM)
xlabel('q1')
ylabel('q2')
zlabel('uM')

surf(q2,q1,uM)
xlabel('q2')
ylabel('q1')
zlabel('uM (m/s)')

%% Controller Design Loop using R

clear
close all
clc

% U0=11.4109; %open loop system marginal stability speed (11.4109)
% [A,B,C,D,F]=mbuild(U0); %build system at U0

R = 0.5:0.5:15;
uM = zeros(1,length(R));

for i = 1:1:length(R)
        U0=11.4109; %open loop system marginal stability speed (11.4109)
        [A,B,C,D,F]=mbuild(U0); %build system at U0
        
        Q = [1.43 0 0 0;
            0 9.1 0 0; 
            0 0 0 0; 
            0 0 0 0]; 
        r = R(i);
        K = lqr(A,B,Q,r);
        U = 0:1:100;
        %there is an issue with how b is stored each iteration       
        b = zeros(1,length(U));
                
        for k = 1:1:length(U)
            [A,B,C,D,F]=mbuild(U(k));
            sysc=ss(A-B*K,F,C,D);
            [x,t]=impulse(sysc,4);

            hPos = abs(x(:,1));
            f = fit(t,hPos,'exp1');
            b(k) = f.b;
            
        end
        
        uM(i) = negPos(U,b);
               
end

figure(4)
plot(R,uM,'b')
xlabel('R')
ylabel('uM (m/s)')



%% Controller Design Test
clear
close all
clc

U0=11.4109; %open loop system marginal stability speed (11.4109)
[A,B,C,D,F]=mbuild(U0);

q1 = 0;
q2 = 15;
q3 = 0;
q4 = 0;
Q = [q1 0 0 0; 
    0 q2 0 0; 
    0 0 q3 0;
    0 0 0 q4]; 
R = 0.5;
K = lqr(A,B,Q,R);

U=40; %test speed
[A,B,C,D,F]=mbuild(U);
sysc=ss(A-B*K,F,C,D);
[x,t]=impulse(sysc,3);
xT = x.';
u=-(K([1 2])*(x.')).';
% u = -(K*x')';
eigenvalues_control = eig(sysc)

figure(5)
subplot(2,1,1)
plot(t,x(:,1),'r',t,u,'k--')
legend('Heave','\beta')
xlabel('t (s)')
ylabel('Heave (m)')
grid on
subplot(2,1,2)
plot(t,x(:,2),'b',t,u,'k--')
legend('\alpha','\beta')
xlabel('t (s)')
ylabel('\theta (rad)')
grid on

U1 = 5:1:100;
b1 = [];
for i = 1:length(U1)
[A,B,C,D,F]=mbuild(U1(i));
sysc=ss(A-B*K,F,C,D);
[x,t]=impulse(sysc,10);
xT = x.';
u=-(K([1 2])*(x.')).';

hPos = abs(x(:,1));
f = fit(t,hPos,'exp1');
exp = f.b;
b1 = [b1;exp];
end

u1 = negPos(U1,b1)

figure(6)
plot(U1,b1,'b')
%xticks(0:length(U1)/10:length(U1))
xline(u1,'k--')
xlabel('Speed (m/s)')
ylabel('Exponential Coefficient')
grid on

%Reference cruise speeds
%Small 4 seater single prop plane ~ 64 m/s
%Passenger jet ~ 150-250 m/s
%Fighter jet (F16) ~ 257 m/s (cruise), 601 m/s (max speed)
%SR71 max speed ~ 983.5 m/s