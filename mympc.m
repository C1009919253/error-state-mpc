function u = mympc(uav, Re, Pe, vd, wd, t)
    BEGIN_ACADO;                                % Always start with "BEGIN_ACADO". 
    
    
    DifferentialState       xe ye ze re1 re2 re3 re4 re5 re6 re7 re8 re9 vx vy vz wx wy wz;
    Control                 a tx ty tz;
    Parameter               vxd vyd vzd;

    TIME                    T;
    
    
    %% Differential Equation
    f = acado.DifferentialEquation();       % Set the differential equation object
    
%     f.add(dot(xe) == re1*vx+re2*vy+re3*vz);
%     f.add(dot(ye) == re4*vx+re5*vy+re6*vz);
%     f.add(dot(ze) == re7*vx+re8*vy+re9*vz);

    ww = skew(wd);

    f.add(dot(xe) == -(ww(1,1)*xe+ww(1,2)*ye+ww(1,3)*ze)+vx-(1.0)); %Rdt*v
    f.add(dot(ye) == -(ww(2,1)*xe+ww(2,2)*ye+ww(2,3)*ze)+vy-(5/1.6*cos((t+T)/1.6)));
    f.add(dot(ze) == -(ww(3,1)*xe+ww(3,2)*ye+ww(3,3)*ze)+vz-(0));
    f.add(dot(re1) == re2*wz - re3*wy);
    f.add(dot(re2) == -re1*wz + re3*wx);
    f.add(dot(re3) == re1*wy - re2*wx);
    f.add(dot(re4) == re5*wz - re6*wy);
    f.add(dot(re5) == -re4*wz + re6*wx);
    f.add(dot(re6) == re4*wy - re5*wx);
    f.add(dot(re7) == re8*wz - re9*wy);
    f.add(dot(re8) == -re7*wz + re9*wx);
    f.add(dot(re9) == re7*wy - re8*wx);
    f.add(dot(vx) == -re3*a);
    f.add(dot(vy) == -re6*a);
    f.add(dot(vz) == 9.8-re9*a/uav.m);
    f.add(dot(wx) == uav.J(3,3)*wy*wz - uav.J(2,2)*wy*wz + tx);
    f.add(dot(wy) == uav.J(1,1)*wx*wz - uav.J(3,3)*wx*wz + ty);
    f.add(dot(wz) == uav.J(2,2)*wy*wx - uav.J(1,1)*wy*wx + tz);

    
    %% Optimal Control Problem
    ocp = acado.OCP(0.0, 1, 10);          % Set up the Optimal Control Problem (OCP)
                                            % Start at 0s control in 20
                                            % intervals upto 1s
    h = {xe, ye, ze, re1-1, re2, re3, re4, re5-1, re6, re7, re8, re9-1, vx, vy, vz, wx, wy, wz};
    Q = 1*eye(18);
    Q(1,1) = 10.0;
    Q(2,2) = 10.0;
    Q(4:12,4:12) = zeros(9,9);
    Q(13:18, 13:18) = 0.2*eye(6);
    r = zeros(1, 18);
%     r(4) = 1;
%     r(8) = 1;
%     r(12) = 1;

    ocp.minimizeLSQ(Q, h, r);
                        
%     ocp.minimizeMayerTerm(xe*xe+ye*ye+ze*ze);   % Minimize a Mayer term
%     ocp.minimizeLSQ(xe*xe+ye*ye+ze*ze);
    ocp.subjectTo( f );

    ocp.subjectTo(0 <= a <= 50.0);
    ocp.subjectTo(-5.0 <= vx <= 5.0);
    ocp.subjectTo(-5.0 <= vy <= 5.0);
    ocp.subjectTo(-5.0 <= vz <= 5.0);
    ocp.subjectTo(-5.0 <= wx <= 5.0);
    ocp.subjectTo(-5.0 <= wy <= 5.0);
    ocp.subjectTo(-5.0 <= wz <= 5.0);
                                            
    ocp.subjectTo( 'AT_START', xe == Pe(1) );
    ocp.subjectTo( 'AT_START', ye == Pe(2) );
    ocp.subjectTo( 'AT_START', ze == Pe(3) );
    ocp.subjectTo( 'AT_START', re1 == Re(1, 1) );
    ocp.subjectTo( 'AT_START', re2 == Re(1, 2) );
    ocp.subjectTo( 'AT_START', re3 == Re(1, 3) );
    ocp.subjectTo( 'AT_START', re4 == Re(2, 1) );
    ocp.subjectTo( 'AT_START', re5 == Re(2, 2) );
    ocp.subjectTo( 'AT_START', re6 == Re(2, 3) );
    ocp.subjectTo( 'AT_START', re7 == Re(3, 1) );
    ocp.subjectTo( 'AT_START', re8 == Re(3, 2) );
    ocp.subjectTo( 'AT_START', re9 == Re(3, 3) );
    ocp.subjectTo( 'AT_START', vx == uav.v(1) );
    ocp.subjectTo( 'AT_START', vy == uav.v(2) );
    ocp.subjectTo( 'AT_START', vz == uav.v(3) );
    ocp.subjectTo( 'AT_START', wx == uav.w(1) );
    ocp.subjectTo( 'AT_START', wy == uav.w(2) );
    ocp.subjectTo( 'AT_START', wz == uav.w(3) );
    
    %% Optimization Algorithm
    algo = acado.OptimizationAlgorithm(ocp); % Set up the optimization algorithm
    
%     algo.set('INTEGRATOR_TOLERANCE', 1e-5 ); % Set some parameters for the algorithm
    
    
END_ACADO;           % Always end with "END_ACADO".
                     % This will generate a file problemname_ACADO.m. 
                     % Run this file to get your results. You can
                     % run the file problemname_ACADO.m as many
                     % times as you want without having to compile again.



u = test_RUN();
end