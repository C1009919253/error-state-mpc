X = [0;0;0];
R = eye(3);
g = 9.8;
m = 1.0;
dt = 0.1;
J = diag([0.0312, 0.0312, 0.0663]);
UAV1 = uav(X, R, g, m, dt, J);
UAV2 = uav(X, R, g, m, dt, J);
ii = 0;
XX1 = zeros(3,201);
XX2 = zeros(3,201);
uu1 = zeros(4,201);
uu2 = zeros(4,201);

for i = 0:dt:20
    ii = ii+1;
    XX1(:, ii) = UAV1.X;
    XX2(:, ii) = UAV2.X;
    X0 = UAV1.X;
    R0 = UAV1.R;
    X1 = UAV2.X;
    R1 = UAV2.R;
%     Xd = [0.5*i;-0.5*i;-10.0-0.1*i];
%     Xd = [5*cos(i/3.2);5*sin(i/3.2);-10.0];
%     Xd = [i;5*sin(i/3.2);-10.0-5*cos(i/3.2)];
    Xd = [i;5*sin(i/1.6);-10.0];
%     Rd = [cos(i) -sin(i) 0; sin(i) cos(i) 0; 0 0 1];
    Rd = [1 0 0; 0 1 0; 0 0 1];
%     vd = [0.5;-0.5;-0.1];
    
    for iii = 1:11
        vd(:,iii) = [-5/3.2*sin((i+(iii-1)/10)/3.2);5/3.2*cos((i+(iii-1)/10)/3.2);0];
%         vd(:,iii) = [1.0;5/3.2*cos((i+(iii-1)/10)/3.2);5/3.2*sin((i+(iii-1)/10)/3.2)];
%         wd(:,iii) = [0;0;0];
    end

    wd = [0;0;0];

    Re1 = Rd'*R0;
    Pe1 = Rd'*X0-Rd'*Xd;
    u1 = mympc(UAV1,Re1,Pe1,vd,wd, i);
    Re2 = Rd'*R1;
    Pe2 = Rd'*X1-Rd'*Xd;
    u2 = tra_mpc(UAV2,Re2,Pe2);
%     u = tra_mpc(UAV, R0, X0-Xd);
    UAV1 = UAV1.update(u1.CONTROLS(1, 2), u1.CONTROLS(1,3:5)');
    uu1(1, ii) = u1.CONTROLS(1,2);
    uu1(2:4, ii) = u1.CONTROLS(1,3:5)';
    UAV2 = UAV2.update(u2.CONTROLS(1, 2), u2.CONTROLS(1,3:5)');
    uu2(1, ii) = u2.CONTROLS(1,2);
    uu2(2:4, ii) = u2.CONTROLS(1,3:5)';
end


figure
error = zeros(2,201);
for i = 1:201

%     xxx = [0.05*(i-1);-0.05*(i-1);-10-0.01*i];
%     xxx = [5*cos((i-1)/32);5*sin((i-1)/32);-10.0];
%     xxx = [(i-1)/10;5*sin((i-1)/32);-10.0-5*cos((i-1)/32)];
    xxx = [0.1*(i-1);5*sin((i-1)/16);-10.0];

    error(1, i) = norm(XX1(:, i) - xxx);
    error(2, i) = norm(XX2(:, i) - xxx);
end
plot(0:0.1:20,error)
xlabel('Time/s')
ylabel('Error/m')
legend('Error-state MPC', 'MPC')
grid on

figure
plot3(XX1(1,:),XX1(2,:),-XX1(3,:))
hold on;
plot3(XX2(1,:),XX2(2,:),-XX2(3,:))
i = 0:dt:20;
% Xd = [0.5*i;-0.5*i;-10.0-0.1*i];
% Xd = [5*cos(i/3.2);5*sin(i/3.2);-10.0+0*i];
% Xd = [i;5*sin(i/3.2);-10.0-5*cos(i/3.2)];
Xd = [i;5*sin(i/1.6);-10.0+0*i];
plot3(Xd(1,:),Xd(2,:),-Xd(3,:))
legend('Error-state MPC', 'MPC', 'desired path')
grid on

figure
subplot 221
plot(0:0.1:20, uu1(1,:));
hold on;
plot(0:0.1:20, uu2(1,:));
title("f");
grid on
subplot 222
plot(0:0.1:20, uu1(2,:));
hold on;
plot(0:0.1:20, uu2(2,:));
title("\tau_x");
grid on
subplot 223
plot(0:0.1:20, uu1(3,:));
hold on;
plot(0:0.1:20, uu2(3,:));
title("\tau_y");
grid on
subplot 224
plot(0:0.1:20, uu1(4,:));
hold on;
plot(0:0.1:20, uu2(4,:));
title("\tau_z");
legend('Error-state MPC', 'MPC')
grid on