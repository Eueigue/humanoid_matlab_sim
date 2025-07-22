function Foot_next = footTrajectory(t_step, step_phase, number_of_step, Foot_state, L_or_R, T_step, p_total)
LF_init = [0;  0.5*PARA.pelvis_width; 0];
RF_init = [0; -0.5*PARA.pelvis_width; 0];

P3 = p_total(:, step_phase+1);

if step_phase == 2
    % P0 = P0 + [0; (L_or_R)*(0.5)*PARA.pelvis_width; 0];
    P0 = p_total(:, step_phase-1) + [0; (L_or_R)*(0.5)*PARA.pelvis_width; 0];
elseif step_phase == 3
    if Foot_state == 1
        P0 = LF_init;
    elseif Foot_state == -1
        P0 = RF_init;
    end
else
    if Foot_state == 1
        % P0 = LF_prev;
        P0 = p_total(:, step_phase-1);
    elseif Foot_state == -1
        % P0 = RF_prev;
        P0 = p_total(:, step_phase-1);
    end
end
P1 = [0.5*(P0(1)+P3(1)); 0.5*(P0(2)+P3(2)); PARA.Foot_up_max];
P2 = [0.5*(P0(1)+P3(1)); 0.5*(P0(2)+P3(2)); PARA.Foot_up_max];

% s = t_step/(SDB_step_time(step_phase)-PARA.dt);
% s = t_step/(T_step-PARA.dt);
s = t_step/(T_step-PARA.dt);

coeff0 = P0;
coeff1 = (-3*P0 + 3*P1);
coeff2 = ( 3*P0 - 6*P1 + 3*P2);
coeff3 = (-1*P0 + 3*P1 - 3*P2 + P3);

% dcoeff0 = coeff1;
% dcoeff1 = 2*coeff2;
% dcoeff2 = 3*coeff3;

Foot_next =  coeff0 +  coeff1*s +  coeff2*s*s + coeff3*s*s*s;

end