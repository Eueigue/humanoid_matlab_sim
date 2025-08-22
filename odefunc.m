function dydt = odefunc(y_ode, contact_wrench_result, Foot_state, m, I, g, rL, rR, rE, flag_EXT, f_EXT)
dydt = zeros(12, 1);

theta    = y_ode([1:3]);
COM      = y_ode([4:6]);
w        = y_ode([7:9]);
dCOM     = y_ode([10:12]);

mL = contact_wrench_result([1:3]);
fL = contact_wrench_result([4:6]);
mR = contact_wrench_result([7:9]);
fR = contact_wrench_result([10:12]);

etaL = 1;
etaR = 1;
etaE = 0;

if Foot_state == 2
    etaL = 1;
    etaR = 1;
elseif Foot_state ==  1 % LF swing
    etaL = 0;
    etaR = 1;
elseif Foot_state == -1 % RF swing
    etaL = 1;
    etaR = 0;
end

if flag_EXT == 0
    etaE = 0;
elseif flag_EXT == 1
    etaE = 1;
end

grav_ = [0;0;-g];

roll = theta(1); pitch = theta(2); yaw = theta(3);
T(1, 1) = cos(pitch) * cos(yaw);
T(1, 2) = -sin(yaw);
T(1, 3) = 0.0;
T(2, 1) = cos(pitch) * sin(yaw);
T(2, 2) = cos(yaw);
T(2, 3) = 0.0;
T(3, 1) = -sin(pitch);
T(3, 2) = 0.0;
T(3, 3) = 1.0;

dydt([1:3])   = T\w;
dydt([4:6])   = dCOM;
dydt([7:9])   = I\(etaL * (mL + skew(rL) * fL) + etaR * (mR + skew(rR) * fR) + etaE * (skew(rE) * f_EXT));
dydt([10:12]) = (etaL * fL + etaR * fR + etaE * f_EXT) / m + grav_;
end