classdef OmnidirectionalVehicle
    properties
        % Arguments
        r           double; % wheel radius
        m           double; % vehicle mass
        Jv          double; % vehicle rotational inertia
        kt          double; % torque coefficient of DC motor
        kb          double; % back-emf coefficient of DC motor
        ra          double; % electrical resistance of DC motor
        ngb         double; % gearbox ratio
        psi         double; % angle between center of mass and each wheel traction direction
        L           double; % length center of mass to each wheel
        % Variables
        phi         double; % angle between fixed frame and vehicle frame
        phi_dot     double; % angular velocity of center of mass
        x           double; % x position of center of mass
        x_dot       double; % x speed of center of mass
        y           double; % y position of center of mass
        y_dot       double; % y speed of center of mass
        PI_vf       double; % rotation matrix
        p_dot       double; % center of mass speed vector
        p_dot_dot   double; % center of mass acceleration vector
        omega       double; % wheel angular speed
        omega_dot   double; % wheel angular acceleration
        v           double; % input voltage
        f           double; % force on each wheel
        delta       double;
        MA          double;
        MB          double;
        Mc          double;
        Mm          double;
        M1          double;
        M2          double;
    end

    methods
        function obj = OmnidirectionalVehicle(r,m,Jv,kt,kb,ra,ngb,psi,L)
            obj.r = r;
            obj.m = m;
            obj.Jv = Jv;
            obj.kt = kt;
            obj.kb = kb;
            obj.ra = ra;
            obj.ngb = ngb;
            obj.psi = psi;
            obj.L = L;

            obj.PI_vf = zeros(3,3);
            obj.Mc = zeros(3,3);
            obj.Mm = diag([1/obj.m;1/obj.m;1/obj.Jv]);
            obj.MA = eye(3,3) .* (1/obj.r) .* ((obj.kt*obj.ngb)/obj.ra);
            obj.MA = eye(3,3) .* (1/obj.r) .* ((obj.kt*obj.kb*(obj.ngb)^2)/obj.ra);
        end

        function obj = compute_rotation(obj)
            obj.PI_vf(1,1) = cos(obj.phi);
            obj.PI_vf(1,2) = sin(obj.phi);
            obj.PI_vf(2,1) = -sin(obj.phi);
            obj.PI_vf(2,2) = cos(obj.phi);
            obj.PI_vf(3,3) = 1.0;
        end

        function obj = compute_Mc(obj)
            alpha = obj.delta - obj.psi;

            obj.Mc(1,1) = -cos(alpha(1));
            obj.Mc(1,2) = -sin(alpha(1));
            obj.Mc(1,3) = obj.L(1)*sin(obj.psi(1));

            obj.Mc(2,1) = -cos(alpha(2));
            obj.Mc(2,2) = -sin(alpha(2));
            obj.Mc(2,3) = obj.L(2)*sin(obj.psi(2));

            obj.Mc(3,1) = -cos(alpha(3));
            obj.Mc(3,2) = -sin(alpha(3));
            obj.Mc(3,3) = obj.L(3)*sin(obj.psi(3));
        end

        function obj = compute_omega(obj)
            obj.omega = (1/obj.r) .* obj.Mc .* obj.PI_vf .* obj.p_dot;
        end

        function obj = compute_force(obj)
            obj.f = obj.MA .* obj.v - obj.MB .* obj.omega;
        end

        function obj = compute_M1(obj)
            obj.M1 = (1/obj.r) .* obj.Mc .* obj.Mm .* obj.Mc' .* obj.MA;
        end

        function obj = compute_M2(obj)
            obj.M2 = (1/obj.r) .* obj.Mc .* obj.Mm .* obj.Mc' .* obj.MB;
        end

        function obj = compute_p_dot_dot(obj)
            obj.p_dot_dot = obj.Mm .* obj.PI_vf' .* obj.Mc' .* obj.f;
        end

        function obj = compute_omega_dot(obj)
            obj.omega_dot = obj.M1.* obj.M2.*obj.omega;
        end
    end
end
