classdef uav

    properties
        X, R, v, w, g, m, dt, J
    end

    methods
        function obj = uav(X0, R0, g0, m0, dt0, J)
            obj.X = X0;
            obj.R = R0;
            obj.g = g0;
            obj.m = m0;
            obj.dt = dt0;
            obj.J = J

            obj.v = [0;0;0];
            obj.w = [0;0;0];

        end

        function obj = update(obj, f, t)
            obj.v = obj.v + obj.dt*([0;0;obj.g]-f/obj.m*obj.R*[0;0;1]);
            obj.w = obj.w + obj.dt*(-cross(obj.w, obj.J*obj.w)+t/obj.m);
            obj.X = obj.X + obj.dt*obj.v;
            obj.R = obj.R * expm(obj.dt*(skew(obj.w)));
        end       

    end
end