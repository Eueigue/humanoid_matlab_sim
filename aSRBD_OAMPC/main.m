%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% MODEL PREDICTIVE CONTROLLER (SRBD)
%%%
%%% Author: Kwanwoo Lee (kwlee365@snu.ac.kr)
%%% Date: 2025. 01. 21. 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%--- Path setting
clc; clear all; close all;
restoredefaultpath;
folder = fileparts(which('main.m'));
addpath(genpath(folder));
%---

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% User Input %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Step information
number_of_step = 10;             % Number of steps
step_length = 0.3;               % Step stride
step_width = PARA.pelvis_width;  % Step width
step_time = 0.6;                 % Step period
L_or_R = 1;                      % First swing foot: 1: Left foot / -1: Right foot

% Disturbance information
F = 500;                                     % [N]:   Magnitude of impact force
deg = 0;                                     % [deg]: Degree of Impact force (0: to Right / 90: to Back / 180: to Left / 270: to Front)
Impact_force_x =   F * cos(PARA.R2D * deg);  % [N]:   x-dir impact force
Impact_force_y = - F * sin(PARA.R2D * deg);  % [N]:   y-dir impact force
Impact_duration = 0.05;                      % [s]:   Impact duration
Impact_timing = 0.3;                         % [s]:   Timing of impact
Impact_step_number = 4;         

% Flags
flag_HORIZON_CHANGED = 0;       % Set to 1 if the number of MPC horizon is changed
flag_VISUALIZATION = 1;         % Set to 1 for graphic ON
flag_VISUALIZATION_ROBOT = 1;   % Set to 1 to show robot
flag_PLOT = 0;                  % Set to 1 to show plots
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%--- Get gradients and hessians with CasADI
if flag_HORIZON_CHANGED == 1
    disp(['Now creating gradients and hessians with CasADI for the MPC horizon: ', num2str(PARA.H), '...']);
    Copy_of_getGradHessWithCasadi();
end
%---
%--- Global variables
global flag_EXIT flag_PAUSE flag_FAIL;
global dx dy;
dx = 0; dy = 0;
%---

%--- SDB(Step Data Buffer)
p_ref_total = zmpTotal(number_of_step, step_length, step_width, L_or_R);
p_total = zmpTotal(number_of_step, step_length, step_width, L_or_R);

T_step_ref_total = stepTimeTotal(number_of_step, step_time);
T_step_total = stepTimeTotal(number_of_step, step_time);
%---

%--- Initialization
t = 0; t_step = 0;
i = 1; i_max = 1E06;
T_step = T_step_total(:, 1);
T_step_ref = T_step_ref_total(:, 1);
step_phase = 1;
iter_error = 0;
% Flags
flag_STEP_CHANGE = 0;
flag_EXIT = 0;
flag_PAUSE = 0;
flag_FAIL = 0;
flag_ERROR = 0;
% Preview control
[Gi, Gx, Gp] = findPreviewGain(PARA.T_preview, PARA.dt, PARA.zc);
[Gi_MPC, Gx_MPC, Gp_MPC] = findPreviewGain(PARA.T_preview, PARA.dt_MPC, PARA.zc);
p_err_sum_x = 0; p_err_sum_y = 0;
p_err_sum_x_ref = 0; p_err_sum_y_ref = 0;
% ZMP
p_des = p_total(:, 1);
% COM
COM = [0; 0; PARA.zc];
dCOM = [0; 0; 0];
COM_prev_step = COM;
dCOM_prev_step = dCOM;
COM_err = [0; 0; 0];
theta_err = [0; 0; 0];
% COM ref.
COM_ref = [0; 0; PARA.zc];
dCOM_ref = [0; 0; 0];
ddCOM_ref = [0; 0; 0];
% Foot
Foot_state = 2; % DSP
LF = [0;  0.5*PARA.pelvis_width; 0]; LF_prev = LF;
RF = [0; -0.5*PARA.pelvis_width; 0]; RF_prev = RF;
color_LF = [0.9290 0.6940 0.1250];
color_RF = [0.6 0.6 0.6];

contact = (L_or_R) .* RF;
contact_ref = (L_or_R) .* RF;

f_z_mg = PARA.m_all * PARA.g;

% Torso
theta = [0; 0; 0];
w = [0; 0; 0];
%---

%--- Data save
t_stored = zeros(1, i_max);
t_step_stored = zeros(1, i_max);
impact_force_stored = zeros(2, i_max);
T_step_stored = zeros(1, i_max);
COM_stored = zeros(3, i_max);
dCOM_stored = zeros(3, i_max);
COM_next_stored = zeros(3, i_max);
COM_ref_stored = zeros(3, i_max);
contact_stored = zeros(3, i_max);
contact_ref_stored = zeros(3, i_max);
contact_temp_stored = zeros(3, i_max);
theta_stored = zeros(3, i_max);
w_stored = zeros(3, i_max);
w_ref_stored = zeros(3, i_max);
p_stored = zeros(3, i_max);
p_next_foot_step_stored = zeros(3, i_max);
p_ref_stored = zeros(3, i_max); 
mL_stored = zeros(3, i_max); 
fL_stored = zeros(3, i_max); 
mR_stored = zeros(3, i_max); 
fR_stored = zeros(3, i_max);
delcontact_stored = zeros(3, i_max);
etaL_stored = zeros(1, i_max);
etaR_stored = zeros(1, i_max);
etak_stored = zeros(1, i_max);
LF_stored = zeros(3, i_max);
RF_stored = zeros(3, i_max);
ticktock_stored = zeros(1, i_max);

step1 = 1:600; step2 = 601:1200; step3 = 1201:1800; step4 = 1801:2400; step5 = 2401:3000; 
step6 = 3001:3600; step7 = 3601:4200; step8 = 4201:4800; step9 = 4801:5400; step10 = 5401:6000;

%%
%--- Main Loop
i_final = 0;
while 1
    tic;

    % Step change
    flag_STEP_CHANGE = checkStepEnd(t_step, T_step);
    if flag_STEP_CHANGE == 1
        % Update step phase
        step_phase = step_phase + 1;
        if step_phase > number_of_step + 3
            flag_EXIT = 1;
        end

        % Update foot state
        Foot_state = (-1)*Foot_state;
        if step_phase == 2
            Foot_state = L_or_R;
        elseif step_phase >= number_of_step + 3
            Foot_state = 2;
        end

        % Update step time
        if step_phase <= number_of_step + 3
            T_step_ref = T_step_ref_total(1, step_phase);
            T_step = T_step_total(:, step_phase);
        end
        
        % Update current step location (contact & contact_ref)
        if Foot_state == 1
            contact = RF_prev;
        elseif Foot_state == -1
            contact = LF_prev;
        elseif Foot_state == 2
            if L_or_R == 1 && mod(number_of_step, 2) == 0
                contact = RF_prev;
            elseif L_or_R == 1 && mod(number_of_step, 2) == 1
                contact = LF_prev;
            elseif L_or_R == -1 && mod(number_of_step, 2) == 0
                contact = LF_prev;
            elseif L_or_R == -1 && mod(number_of_step, 2) == 1
                contact = RF_prev;
            else
                disp('Last Contact Failed!!');
                break;
            end
        else
            disp('Contact Update Failed!!');
            break;
        end

        contact_ref = p_ref_total(:, step_phase + 1);
        
        % Reset t_step & flag
        t_step = 0;
        flag_STEP_CHANGE = 0; 
    end

    % Exit flag
    if (norm(COM_err) > 0.2 || norm(theta_err) > 0.5)
        disp('Walking fail!!');
        i_final = i;
        flag_FAIL = 1;
        break;
    elseif (flag_EXIT == 1)
        disp('Walking finish!!');
        % close(10);
        i_final = i;
        break;
    end

    % Disturbance
    disturbance_duration = Impact_duration; % [sec]
    disturbance_timing = Impact_timing;
    if (step_phase == Impact_step_number + 1) && ((t_step >= disturbance_timing))
        flag_ERROR = 1;
    end
    if (flag_ERROR == 1) && (iter_error > (Impact_duration/PARA.dt))
        flag_ERROR = 0;
    end
    if flag_ERROR == 1
        disturbance_magnitude = [Impact_force_x; Impact_force_y; 0]; % [N]
        ddCOM_dist = disturbance_magnitude/PARA.m_all;
        dCOM_dist = ddCOM_dist.*PARA.dt;
        COM_dist = dCOM_dist.*PARA.dt + 0.5.*ddCOM_dist.*PARA.dt.*PARA.dt;
        COM = COM + COM_dist;
        dCOM = dCOM + dCOM_dist;
        iter_error = iter_error + 1;
    else     
        disturbance_magnitude = [0; 0; 0]; % [N]
        ddCOM_err = [0; 0; 0];
    end
    
    % Reference    
    [COM_ref_next, dCOM_ref_next, ddCOM_ref_next, p_err_sum_x_ref_next, p_err_sum_y_ref_next] = previewControl(t_step, step_phase, p_ref_total, T_step_ref_total, Gi, Gx, Gp, PARA.A_preview, PARA.B_preview, PARA.C_preview, COM_ref, dCOM_ref, ddCOM_ref, p_err_sum_x_ref, p_err_sum_y_ref);      
    [theta_ref_horizon, COM_ref_horizon, w_ref_horizon, dCOM_ref_horizon, contact_ref_horizon, rL_ref_horizon, rR_ref_horizon, etaL_ref_horizon, etaR_ref_horizon, etak_ref_horizon, fL_ref_horizon, fR_ref_horizon] = ...
        mpcRefWindow(t_step, step_phase, Foot_state, L_or_R, LF_prev, RF_prev, ...
                     COM_ref, dCOM_ref, ddCOM_ref, ...
                     p_err_sum_x_ref, p_err_sum_y_ref, T_step_ref, p_ref_total, T_step_ref_total, Gi_MPC, Gx_MPC, Gp_MPC);

    COM_err = COM - COM_ref;
    theta_err = theta - zeros(3,1);
        
    % Calc control input
    x0 = [theta; COM; w; dCOM; contact];
    
    [mL, fL, mR, fR, delcontact] =  ...
        nextState(x0, ...
                  theta, COM, w, dCOM, contact, ...
                  contact_ref, ...
                  theta_ref_horizon, COM_ref_horizon, w_ref_horizon, dCOM_ref_horizon, contact_ref_horizon, ...
                  rL_ref_horizon, rR_ref_horizon, fL_ref_horizon, fR_ref_horizon, etaL_ref_horizon, etaR_ref_horizon, etak_ref_horizon);

    contact_wrench_result = [mL; fL; mR; fR];
    rL = LF_prev - COM;
    rR = RF_prev - COM;
    
    if step_phase ~= 1
        p_total(:, step_phase + 1) = contact + delcontact;
    end

    % if etak_ref_horizon(1) == 1 && Foot_state == 1
    %     rR = p_total(:, step_phase + 1) - COM;
    % elseif etak_ref_horizon(1) == 1 && Foot_state == -1
    %     rL = p_total(:, step_phase + 1) - COM;
    % end

    % Plant response
    t_span = [0 PARA.dt];
    y0 = [theta; COM; w; dCOM];
    [t_ode, y_ode] = ode45(@(t_ode, y_ode) odefunc(y_ode, contact_wrench_result, Foot_state, PARA.m_all, PARA.I, PARA.g, rL, rR), t_span, y0);  
    theta_next = [y_ode(end, [1:3])]';
    COM_next   = [y_ode(end, [4:6])]';
    w_next     = [y_ode(end, [7:9])]';
    dCOM_next  = [y_ode(end, [10:12])]';

    % Foot trajectory
    if Foot_state == 2
        LF = LF_prev;
        RF = RF_prev;
        if step_phase == number_of_step + 2
            if L_or_R == 1 && mod(number_of_step, 2) == 0
                LF = footTrajectory(t_step, step_phase, number_of_step, Foot_state, L_or_R, T_step, p_total);
            elseif L_or_R == 1 && mod(number_of_step, 2) == 1
                RF = footTrajectory(t_step, step_phase, number_of_step, Foot_state, L_or_R, T_step, p_total);
            elseif L_or_R == -1 && mod(number_of_step, 2) == 0
                RF = footTrajectory(t_step, step_phase, number_of_step, Foot_state, L_or_R, T_step, p_total);
            elseif L_or_R == -1 && mod(number_of_step, 2) == 1
                LF = footTrajectory(t_step, step_phase, number_of_step, Foot_state, L_or_R, T_step, p_total);
            end
        end
    elseif Foot_state ==  1 % LF swing
        LF = footTrajectory(t_step, step_phase, number_of_step, Foot_state, L_or_R, T_step, p_total);
        RF = RF_prev;
        contact = RF_prev;
    elseif Foot_state == -1 % RF swing
        LF = LF_prev;
        RF = footTrajectory(t_step, step_phase, number_of_step, Foot_state, L_or_R, T_step, p_total);
        contact = LF_prev;
    end

%     if (step_phase == number_of_step + 2)
%         LF = LF_prev;
%         RF = RF_prev;
%     end
    
    
    % Time ticktock
    ticktock = toc;
    
    % Data save
    t_stored(:, i) = t;
    t_step_stored(:, i) = t_step;
    T_step_stored(:, i) = T_step;
    COM_stored(:, i) = COM;
    COM_next_stored(:, i) = COM_next;
    dCOM_stored(:, i) = dCOM;
    COM_ref_stored(:, i) = COM_ref;
    theta_stored(:,i) = theta;
    w_stored(:, i) = w;
    w_ref_stored(:, i) = w_ref_horizon(:, 1);
    contact_stored(:,i) = contact;
    contact_ref_stored(:,i) = contact_ref;
    p_stored(:, i) = p_total(:, step_phase);
    p_next_foot_step_stored(:, i) = p_total(:, step_phase + 1);
    p_ref_stored(:, i) = p_ref_total(:, step_phase);
    mL_stored(:, i) = mL;
    fL_stored(:, i) = fL;
    mR_stored(:, i) = mR;
    fR_stored(:, i) = fR;
    LF_stored(:, i) = LF;
    RF_stored(:, i) = RF;
    delcontact_stored(:, i) = delcontact;
    etaL_stored(1, i) = etaL_ref_horizon(1);
    etaR_stored(1, i) = etaR_ref_horizon(1);
    etak_stored(1, i) = etak_ref_horizon(1);
    ticktock_stored(:, i) = ticktock;

    % Time waits for no one
    t = t + PARA.dt;
    t_step = t_step + PARA.dt;

    COM_ref = COM_ref_next; dCOM_ref = dCOM_ref_next; ddCOM_ref = ddCOM_ref_next;
    p_err_sum_x_ref = p_err_sum_x_ref_next; p_err_sum_y_ref = p_err_sum_y_ref_next;
    
    theta = theta_next; w = w_next;
    COM = COM_next; dCOM = dCOM_next;
        
    % contact(1:2, :) = contact_next(1:2, :);
    LF_prev = LF;
    RF_prev = RF;

    i = i + 1;
end
%---

%%
% %-- Plot
% t_stored = t_stored(:, 2:i-1);
% t_step_stored = t_step_stored(:, 2:i-1);
% impact_force_stored = impact_force_stored(:, 2:i-1);
% T_step_stored = T_step_stored(:, 2:i-1);
% COM_stored = COM_stored(:, 2:i-1); COM_ref_stored = COM_ref_stored(:, 2:i-1); dCOM_stored = dCOM_stored(:, 2:i-1); 
% theta_stored = theta_stored(:, 2:i-1); 
% p_ref_stored = p_ref_stored(:, 2:i-1); 
% mL_stored = mL_stored(:, 2:i-1); fL_stored = fL_stored(:, 2:i-1); mR_stored = mR_stored(:, 2:i-1); fR_stored = fR_stored(:, 2:i-1);
% etaR_stored = etaR_stored(:, 2:i-1); etaL_stored = etaL_stored(:, 2:i-1);
% LF_stored = LF_stored(:, 2:i-1); RF_stored = RF_stored(:, 2:i-1);
% ticktock_stored = ticktock_stored(:, 2:i-1);


%% --- Animation

% World generation
if ishandle(10)
    close(10)
end
if flag_VISUALIZATION == 1
    fig_world = figure(10);
    set(fig_world, 'Position', [1000 540 920 455], 'Renderer', 'OpenGL', 'Color',[1,1,1], 'KeyPressFcn', @printfig);
    axe = axes('Parent', fig_world);
    if flag_VISUALIZATION_ROBOT == 1
        ax = 55; ay = 15;
    else
        ax = 0; ay = 89.999;
    end
    view([ax ay]);
    set(axe, 'XLim', [-0.5 0.5], 'YLim', [-0.5 0.5], 'ZLim', [-0.02 1], 'DataAspectRatio', [1 1 1]);
    grid on; grid minor;
    xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
end

i_viz = 1;
while 1
    if (i_viz >= i_final-1)
        disp('ANIMATION FINISHED!!!');
        break
    end
    COM = COM_stored(:,i_viz);
    COM_ref = COM_ref_stored(:,i_viz);
    theta = theta_stored(:,i_viz); 
    LF = LF_stored(:,i_viz);
    RF = RF_stored(:,i_viz);
    fL = fL_stored(:, i_viz);
    fR = fR_stored(:, i_viz);


    if flag_VISUALIZATION == 1
        % Camera control
        ax = ax - dx;
        ay = ay - dy;
        dx = 0; dy = 0;
        view([ax, ay]);
        set(axe,'XLim',[-0.5+COM(1) 0.5+COM(1)],'YLim',[-0.5+COM(2) 0.5+COM(2)],'ZLim',[-0.02 1.0], 'DataAspectRatio', [1 1 1]);
    end

    % Visualization
    if mod(i_viz, 100) == 1  
        if flag_VISUALIZATION == 1
            if flag_VISUALIZATION_ROBOT == 1
                %--- Inverse kinematics - COM
                pCOM = COM; 
                qPEL = mat2quat(rotX_rad(theta(1))*rotY_rad(-theta(2))); rotmPEL = quat2mat(qPEL);
                pLF = LF;   %qLF = mat2quat(rotX_rad(theta(1))*rotY_rad(-theta(2))); 
                qLF = [1 0 0 0];
                pRF = RF;   %qRF = mat2quat(rotX_rad(theta(1))*rotY_rad(-theta(2))); 
                qRF = [1 0 0 0];

                x_target = [pCOM; qPEL'; pLF; qLF'; pRF; qRF'];
                [q_target, pPEL] = IK_COM(x_target);
                q1 = q_target(1); q2 = q_target(2); q3 = q_target(3); q4  = q_target(4);  q5  = q_target(5);  q6  = q_target(6); % LLEG
                q7 = q_target(7); q8 = q_target(8); q9 = q_target(9); q10 = q_target(10); q11 = q_target(11); q12 = q_target(12); % RLEG
                %---

                %--- Leg
                T0 = [rotmPEL pPEL; [0 0 0 1]];
                % Left leg
                T1_LLEG =      T0*[rotZ_rad(q1) [0 PARA.l1 0]' ; [0 0 0 1]];
                T2_LLEG = T1_LLEG*[rotX_rad(q2) [0 0 0]' ; [0 0 0 1]];
                T3_LLEG = T2_LLEG*[rotY_rad(q3) [0 PARA.l2 -PARA.l3]' ; [0 0 0 1]];
                T4_LLEG = T3_LLEG*[rotY_rad(q4) [0 0 -PARA.l4]' ; [0 0 0 1]];
                T5_LLEG = T4_LLEG*[rotY_rad(q5) [0 0 -PARA.l5]' ; [0 0 0 1]];
                T6_LLEG = T5_LLEG*[rotX_rad(q6) [0 0 0]' ; [0 0 0 1]];
                Te_LLEG = T6_LLEG*[eye(3) [0 0 -PARA.l6]' ; [0 0 0 1]];

                link01_LLEG_x = [T0(1,4) T1_LLEG(1,4)];         link01_LLEG_y = [T0(2,4) T1_LLEG(2,4)];   link01_LLEG_z = [T0(3,4) T1_LLEG(3,4)];
                link12_LLEG_x = [T1_LLEG(1,4) T2_LLEG(1,4)];	link12_LLEG_y = [T1_LLEG(2,4) T2_LLEG(2,4)];   link12_LLEG_z = [T1_LLEG(3,4) T2_LLEG(3,4)];
                link23_LLEG_x = [T2_LLEG(1,4) T3_LLEG(1,4)];	link23_LLEG_y = [T2_LLEG(2,4) T3_LLEG(2,4)];   link23_LLEG_z = [T2_LLEG(3,4) T3_LLEG(3,4)];
                link34_LLEG_x = [T3_LLEG(1,4) T4_LLEG(1,4)];	link34_LLEG_y = [T3_LLEG(2,4) T4_LLEG(2,4)];   link34_LLEG_z = [T3_LLEG(3,4) T4_LLEG(3,4)];
                link45_LLEG_x = [T4_LLEG(1,4) T5_LLEG(1,4)];	link45_LLEG_y = [T4_LLEG(2,4) T5_LLEG(2,4)];   link45_LLEG_z = [T4_LLEG(3,4) T5_LLEG(3,4)];
                link56_LLEG_x = [T5_LLEG(1,4) T6_LLEG(1,4)];	link56_LLEG_y = [T5_LLEG(2,4) T6_LLEG(2,4)];   link56_LLEG_z = [T5_LLEG(3,4) T6_LLEG(3,4)];
                link6e_LLEG_x = [T6_LLEG(1,4) Te_LLEG(1,4)];	link6e_LLEG_y = [T6_LLEG(2,4) Te_LLEG(2,4)];   link6e_LLEG_z = [T6_LLEG(3,4) Te_LLEG(3,4)];
                
                link01_LLEG = cylinder(axe, [link01_LLEG_x; link01_LLEG_y; link01_LLEG_z]', 0.005, [0 0 0], 1, 20);
                link23_LLEG = cylinder(axe, [link23_LLEG_x; link23_LLEG_y; link23_LLEG_z]', 0.005, [0 0 0], 1, 20);
                link34_LLEG = cylinder(axe, [link34_LLEG_x; link34_LLEG_y; link34_LLEG_z]', 0.005, [0 0 0], 1, 20);
                link45_LLEG = cylinder(axe, [link45_LLEG_x; link45_LLEG_y; link45_LLEG_z]', 0.005, [0 0 0], 1, 20);
                link6e_LLEG = cylinder(axe, [link6e_LLEG_x; link6e_LLEG_y; link6e_LLEG_z]', 0.005, 'k', 1, 20);
     
                T1_RLEG =      T0*[rotZ_rad(q7) [0 -PARA.l1 0]' ; [0 0 0 1]];
                T2_RLEG = T1_RLEG*[rotX_rad(q8) [0 0 0]' ; [0 0 0 1]];
                T3_RLEG = T2_RLEG*[rotY_rad(q9) [0 -PARA.l2 -PARA.l3]' ; [0 0 0 1]];
                T4_RLEG = T3_RLEG*[rotY_rad(q10) [0 0 -PARA.l4]' ; [0 0 0 1]];
                T5_RLEG = T4_RLEG*[rotY_rad(q11) [0 0 -PARA.l5]' ; [0 0 0 1]];
                T6_RLEG = T5_RLEG*[rotX_rad(q12) [0 0 0]' ; [0 0 0 1]];
                Te_RLEG = T6_RLEG*[eye(3) [0 0 -PARA.l6]' ; [0 0 0 1]];

                link01_RLEG_x = [T0(1,4) T1_RLEG(1,4)];    link01_RLEG_y = [T0(2,4) T1_RLEG(2,4)];   link01_RLEG_z = [T0(3,4) T1_RLEG(3,4)];
                link12_RLEG_x = [T1_RLEG(1,4) T2_RLEG(1,4)];	link12_RLEG_y = [T1_RLEG(2,4) T2_RLEG(2,4)];   link12_RLEG_z = [T1_RLEG(3,4) T2_RLEG(3,4)];
                link23_RLEG_x = [T2_RLEG(1,4) T3_RLEG(1,4)];	link23_RLEG_y = [T2_RLEG(2,4) T3_RLEG(2,4)];   link23_RLEG_z = [T2_RLEG(3,4) T3_RLEG(3,4)];
                link34_RLEG_x = [T3_RLEG(1,4) T4_RLEG(1,4)];	link34_RLEG_y = [T3_RLEG(2,4) T4_RLEG(2,4)];   link34_RLEG_z = [T3_RLEG(3,4) T4_RLEG(3,4)];
                link45_RLEG_x = [T4_RLEG(1,4) T5_RLEG(1,4)];	link45_RLEG_y = [T4_RLEG(2,4) T5_RLEG(2,4)];   link45_RLEG_z = [T4_RLEG(3,4) T5_RLEG(3,4)];
                link56_RLEG_x = [T5_RLEG(1,4) T6_RLEG(1,4)];	link56_RLEG_y = [T5_RLEG(2,4) T6_RLEG(2,4)];   link56_RLEG_z = [T5_RLEG(3,4) T6_RLEG(3,4)];
                link6e_RLEG_x = [T6_RLEG(1,4) Te_RLEG(1,4)];	link6e_RLEG_y = [T6_RLEG(2,4) Te_RLEG(2,4)];   link6e_RLEG_z = [T6_RLEG(3,4) Te_RLEG(3,4)];

                link01_RLEG = cylinder(axe, [link01_RLEG_x; link01_RLEG_y; link01_RLEG_z]', 0.005, [0 0 0], 1, 20);
                link23_RLEG = cylinder(axe, [link23_RLEG_x; link23_RLEG_y; link23_RLEG_z]', 0.005, [0 0 0], 1, 20);
                link34_RLEG = cylinder(axe, [link34_RLEG_x; link34_RLEG_y; link34_RLEG_z]', 0.005, [0 0 0], 1, 20);
                link45_RLEG = cylinder(axe, [link45_RLEG_x; link45_RLEG_y; link45_RLEG_z]', 0.005, [0 0 0], 1, 20);
                link6e_RLEG = cylinder(axe, [link6e_RLEG_x; link6e_RLEG_y; link6e_RLEG_z]', 0.005, 'k', 1, 20);
                %---

                %--- Torso
                TORSO_x = 0.12;   % [m]
                TORSO_y = 0.2;   % [m]
                TORSO_z = 0.22;   % [m]

                T1_TORSO = T0*[eye(3) [0 0 PARA.l0-0.5*TORSO_z]'; [0 0 0 1]];
                T2_TORSO_1 = T0*[eye(3) [ 0.5*TORSO_x  0.5*TORSO_y PARA.l0+0.5*TORSO_z]'; [0 0 0 1]];
                T2_TORSO_2 = T0*[eye(3) [-0.5*TORSO_x  0.5*TORSO_y PARA.l0+0.5*TORSO_z]'; [0 0 0 1]];
                T2_TORSO_3 = T0*[eye(3) [-0.5*TORSO_x -0.5*TORSO_y PARA.l0+0.5*TORSO_z]'; [0 0 0 1]];
                T2_TORSO_4 = T0*[eye(3) [ 0.5*TORSO_x -0.5*TORSO_y PARA.l0+0.5*TORSO_z]'; [0 0 0 1]];
                T3_TORSO_1 = T0*[eye(3) [ 0.5*TORSO_x  0.5*TORSO_y PARA.l0-0.5*TORSO_z]'; [0 0 0 1]];
                T3_TORSO_2 = T0*[eye(3) [-0.5*TORSO_x  0.5*TORSO_y PARA.l0-0.5*TORSO_z]'; [0 0 0 1]];
                T3_TORSO_3 = T0*[eye(3) [-0.5*TORSO_x -0.5*TORSO_y PARA.l0-0.5*TORSO_z]'; [0 0 0 1]];
                T3_TORSO_4 = T0*[eye(3) [ 0.5*TORSO_x -0.5*TORSO_y PARA.l0-0.5*TORSO_z]'; [0 0 0 1]];

                link01_TORSO_x = [T0(1,4) T1_TORSO(1,4)];    link01_TORSO_y = [T0(2,4) T1_TORSO(2,4)];   link01_TORSO_z = [T0(3,4) T1_TORSO(3,4)];
                link2_TORSO_x = [T2_TORSO_1(1,4) T2_TORSO_2(1,4) T2_TORSO_3(1,4) T2_TORSO_4(1,4) T2_TORSO_1(1,4)];
                link2_TORSO_y = [T2_TORSO_1(2,4) T2_TORSO_2(2,4) T2_TORSO_3(2,4) T2_TORSO_4(2,4) T2_TORSO_1(2,4)];
                link2_TORSO_z = [T2_TORSO_1(3,4) T2_TORSO_2(3,4) T2_TORSO_3(3,4) T2_TORSO_4(3,4) T2_TORSO_1(3,4)];
                link3_TORSO_x = [T3_TORSO_1(1,4) T3_TORSO_2(1,4) T3_TORSO_3(1,4) T3_TORSO_4(1,4) T3_TORSO_1(1,4)];
                link3_TORSO_y = [T3_TORSO_1(2,4) T3_TORSO_2(2,4) T3_TORSO_3(2,4) T3_TORSO_4(2,4) T3_TORSO_1(2,4)];
                link3_TORSO_z = [T3_TORSO_1(3,4) T3_TORSO_2(3,4) T3_TORSO_3(3,4) T3_TORSO_4(3,4) T3_TORSO_1(3,4)];
                link41_TORSO_x = [T2_TORSO_1(1,4) T3_TORSO_1(1,4)]; link41_TORSO_y = [T2_TORSO_1(2,4) T3_TORSO_1(2,4)]; link41_TORSO_z = [T2_TORSO_1(3,4) T3_TORSO_1(3,4)];
                link42_TORSO_x = [T2_TORSO_2(1,4) T3_TORSO_2(1,4)]; link42_TORSO_y = [T2_TORSO_2(2,4) T3_TORSO_2(2,4)]; link42_TORSO_z = [T2_TORSO_2(3,4) T3_TORSO_2(3,4)];
                link43_TORSO_x = [T2_TORSO_3(1,4) T3_TORSO_3(1,4)]; link43_TORSO_y = [T2_TORSO_3(2,4) T3_TORSO_3(2,4)]; link43_TORSO_z = [T2_TORSO_3(3,4) T3_TORSO_3(3,4)];
                link44_TORSO_x = [T2_TORSO_4(1,4) T3_TORSO_4(1,4)]; link44_TORSO_y = [T2_TORSO_4(2,4) T3_TORSO_4(2,4)]; link44_TORSO_z = [T2_TORSO_4(3,4) T3_TORSO_4(3,4)];

                link01_TORSO = cylinder(axe, [link01_TORSO_x; link01_TORSO_y; link01_TORSO_z]', 0.005, [0, 0, 0], 1, 20);
                link2_TORSO_1 = cylinder(axe, [link2_TORSO_x(1:2); link2_TORSO_y(1:2); link2_TORSO_z(1:2)]', 0.005, [0, 0, 0], 1, 20);
                link2_TORSO_2 = cylinder(axe, [link2_TORSO_x(2:3); link2_TORSO_y(2:3); link2_TORSO_z(2:3)]', 0.005, [0, 0, 0], 1, 20);
                link2_TORSO_3 = cylinder(axe, [link2_TORSO_x(3:4); link2_TORSO_y(3:4); link2_TORSO_z(3:4)]', 0.005, [0, 0, 0], 1, 20);
                link2_TORSO_4 = cylinder(axe, [[link2_TORSO_x(4) link2_TORSO_x(1)]; [link2_TORSO_y(4) link2_TORSO_y(1)]; [link2_TORSO_z(4) link2_TORSO_z(1)]]', 0.005, [0, 0, 0], 1, 20);
                link3_TORSO_1 = cylinder(axe, [link3_TORSO_x(1:2); link3_TORSO_y(1:2); link3_TORSO_z(1:2)]', 0.005, [0, 0, 0], 1, 20);
                link3_TORSO_2 = cylinder(axe, [link3_TORSO_x(2:3); link3_TORSO_y(2:3); link3_TORSO_z(2:3)]', 0.005, [0, 0, 0], 1, 20);
                link3_TORSO_3 = cylinder(axe, [link3_TORSO_x(3:4); link3_TORSO_y(3:4); link3_TORSO_z(3:4)]', 0.005, [0, 0, 0], 1, 20);
                link3_TORSO_4 = cylinder(axe, [[link3_TORSO_x(4) link3_TORSO_x(1)]; [link3_TORSO_y(4) link3_TORSO_y(1)]; [link3_TORSO_z(4) link3_TORSO_z(1)]]', 0.005, [0, 0, 0], 1, 20);
                link4_TORSO_1 = cylinder(axe, [link41_TORSO_x; link41_TORSO_y; link41_TORSO_z]', 0.005, [0, 0, 0], 1, 20);
                link4_TORSO_2 = cylinder(axe, [link42_TORSO_x; link42_TORSO_y; link42_TORSO_z]', 0.005, [0, 0, 0], 1, 20);
                link4_TORSO_3 = cylinder(axe, [link43_TORSO_x; link43_TORSO_y; link43_TORSO_z]', 0.005, [0, 0, 0], 1, 20);
                link4_TORSO_4 = cylinder(axe, [link44_TORSO_x; link44_TORSO_y; link44_TORSO_z]', 0.005, [0, 0, 0], 1, 20);
                %---
            end

            % COM
            visual_COM = animatedline('Marker', 'o', 'MarkerFaceColor', 'green', 'MarkerEdgeColor', 'black');
            addpoints(visual_COM, COM(1), COM(2), COM(3));
    
            visual_COM_ref = animatedline('Marker', 'o', 'MarkerFaceColor', 'red', 'MarkerEdgeColor', 'black');
            addpoints(visual_COM_ref, COM_ref(1), COM_ref(2), COM_ref(3));
        
            % LF
            force_draw_scale = 5e-4;
            visual_LF_p1 = LF + [ 0.5*PARA.Foot_length;  0.5*PARA.Foot_width; 0];
            visual_LF_p2 = LF + [-0.5*PARA.Foot_length;  0.5*PARA.Foot_width; 0];
            visual_LF_p3 = LF + [-0.5*PARA.Foot_length; -0.5*PARA.Foot_width; 0];
            visual_LF_p4 = LF + [ 0.5*PARA.Foot_length; -0.5*PARA.Foot_width; 0];
            visual_LF_x = [visual_LF_p1(1) visual_LF_p2(1) visual_LF_p3(1) visual_LF_p4(1) visual_LF_p1(1)];
            visual_LF_y = [visual_LF_p1(2) visual_LF_p2(2) visual_LF_p3(2) visual_LF_p4(2) visual_LF_p1(2)];
            visual_LF_z = [visual_LF_p1(3) visual_LF_p2(3) visual_LF_p3(3) visual_LF_p4(3) visual_LF_p1(3)];
    
            hold on
            visual_fL = quiver3(LF(1), LF(2), LF(3), fL(1) * force_draw_scale, fL(2) * force_draw_scale, fL(3) * force_draw_scale, 0, ...
                                'Color', 'r', 'LineWidth', 1.5, 'MaxHeadSize', 1);
            if (Foot_state == 1)
                visual_LF = animatedline(visual_LF_x, visual_LF_y, visual_LF_z, 'color', color_LF, 'LineWidth', 1.5);
                visual_LF_center = animatedline('Marker', 'o', 'MarkerFaceColor', color_LF, 'MarkerEdgeColor', 'k', 'MarkerSize', 5);
                addpoints(visual_LF_center, LF(1), LF(2), LF(3));
            elseif (Foot_state == -1)
                visual_LF = animatedline(visual_LF_x, visual_LF_y, visual_LF_z, 'color', color_LF, 'LineWidth', 1.5);
                visual_LF_center = animatedline('Marker', 'o', 'MarkerFaceColor', color_LF, 'MarkerEdgeColor', 'k', 'MarkerSize', 5);
                addpoints(visual_LF_center, LF(1), LF(2), LF(3));
            else
                visual_LF = animatedline(visual_LF_x, visual_LF_y, visual_LF_z, 'color', color_LF, 'LineWidth', 1.5);
                visual_LF_center = animatedline('Marker', 'o', 'MarkerFaceColor', color_LF, 'MarkerEdgeColor', 'k', 'MarkerSize', 5);
                addpoints(visual_LF_center, LF(1), LF(2), LF(3));
            end
            % RF
            visual_RF_p1 = RF + [ 0.5*PARA.Foot_length;  0.5*PARA.Foot_width; 0];
            visual_RF_p2 = RF + [-0.5*PARA.Foot_length;  0.5*PARA.Foot_width; 0];
            visual_RF_p3 = RF + [-0.5*PARA.Foot_length; -0.5*PARA.Foot_width; 0];
            visual_RF_p4 = RF + [ 0.5*PARA.Foot_length; -0.5*PARA.Foot_width; 0];
            visual_RF_x = [visual_RF_p1(1) visual_RF_p2(1) visual_RF_p3(1) visual_RF_p4(1) visual_RF_p1(1)];
            visual_RF_y = [visual_RF_p1(2) visual_RF_p2(2) visual_RF_p3(2) visual_RF_p4(2) visual_RF_p1(2)];
            visual_RF_z = [visual_RF_p1(3) visual_RF_p2(3) visual_RF_p3(3) visual_RF_p4(3) visual_RF_p1(3)];
            hold on
            visual_fR = quiver3(RF(1), RF(2), RF(3), fR(1) * force_draw_scale, fR(2) * force_draw_scale, fR(3) * force_draw_scale, 0, ...
                                'Color', 'r', 'LineWidth', 1.5, 'MaxHeadSize', 1);
            if (Foot_state == 1)
                visual_RF = animatedline(visual_RF_x, visual_RF_y, visual_RF_z, 'color', color_RF, 'LineWidth', 1.5);
                visual_RF_center = animatedline('Marker', 'o', 'MarkerFaceColor', color_RF, 'MarkerEdgeColor', 'k', 'MarkerSize', 5);
                addpoints(visual_RF_center, RF(1), RF(2), RF(3));       
            elseif (Foot_state == -1)
                visual_RF = animatedline(visual_RF_x, visual_RF_y, visual_RF_z, 'color', color_RF, 'LineWidth', 1.5);
                visual_RF_center = animatedline('Marker', 'o', 'MarkerFaceColor', color_RF, 'MarkerEdgeColor', 'k', 'MarkerSize', 5);
                addpoints(visual_RF_center, RF(1), RF(2), RF(3));       
            else
                visual_RF = animatedline(visual_RF_x, visual_RF_y, visual_RF_z, 'color', color_RF, 'LineWidth', 1.5);
                visual_RF_center = animatedline('Marker', 'o', 'MarkerFaceColor', color_RF, 'MarkerEdgeColor', 'k', 'MarkerSize', 5);
                addpoints(visual_RF_center, RF(1), RF(2), RF(3));           
            end           

            drawnow;
            if flag_PAUSE == 1
                disp('Walking pause!!');
                waitforbuttonpress;
                flag_PAUSE = 0;
            end

            delete(visual_COM);
            delete(visual_COM_ref);
            delete(visual_LF); delete(visual_RF);
            delete(visual_LF_center); delete(visual_RF_center);
            delete(visual_fL); delete(visual_fR);
            if flag_VISUALIZATION_ROBOT == 1
                delete(link01_LLEG); delete(link23_LLEG); delete(link34_LLEG); delete(link45_LLEG); delete(link6e_LLEG); 
                delete(link01_RLEG); delete(link23_RLEG); delete(link34_RLEG); delete(link45_RLEG); delete(link6e_RLEG); 
                delete(link01_TORSO); 
                delete(link2_TORSO_1); delete(link2_TORSO_2); delete(link2_TORSO_3); delete(link2_TORSO_4); 
                delete(link3_TORSO_1); delete(link3_TORSO_2); delete(link3_TORSO_3); delete(link3_TORSO_4); 
                delete(link4_TORSO_1); delete(link4_TORSO_2); delete(link4_TORSO_3); delete(link4_TORSO_4); 
            end
        end
    end

    i_viz = i_viz + 1;
end
%---

%% Plot

if flag_PLOT == 1
    fig1 = figure('Units','normalized', ...
                  'OuterPosition',[ (1-0.9)/2, (1-0.9*0.8)/2, 0.9, 0.9*0.8 ], ...
                  'Name','Analysis');
    numCols = 3;
    tile = tiledlayout(2, numCols); 
    %tile.TileSpacing = 'compact';
    %tile.Padding = 'compact';
    getTile = @(row, col) (row - 1) * numCols + col;

    %––– Fig 1: delcontact_stored(1,:) & reference
    subplot1 = nexttile(getTile(1, 1));
    h1 = plot(t_stored(1:i-1), delcontact_stored(1,1:i-1),        'b-',  'LineWidth', 2.5); hold on;
    h2 = plot(t_stored(1:i-1), (1.0) * step_length * ones(1,i-1), 'r-.', 'LineWidth', 1.5); % 한보폭
    h3 = plot(t_stored(1:i-1), (0.5) * step_length * ones(1,i-1), 'r-.', 'LineWidth', 1.5); % 반보폭
    grid on;
    subplot1.XTick = t_stored(1):T_step:t_stored(i-1)+PARA.dt;
    subplot1.YTick = (-1.0)*step_length : 0.5*step_length : 1.0*step_length;
    xlim(subplot1, [t_stored(1), t_stored(i-1)+PARA.dt]);
    ylim(subplot1, [(1.1)*min(delcontact_stored(1,:)), (1.1)*max(delcontact_stored(1,:))]);
    xlabel(subplot1, 'time (s)');
    ylabel(subplot1, '\deltac_x (m)');
    legend(subplot1, [h1,h2,h3], {'\deltac_x','Stride^{Ref}', 'Half\_Stride^{Ref}'}, 'Location','northeastoutside');
    title(subplot1, '\deltac_x');
    
    %––– Fig 2: delcontact_stored(2,:) & reference
    subplot2 = nexttile(getTile(2, 1));
    h1 = plot(t_stored(1:i-1), delcontact_stored(2,1:i-1),                           'b-', 'LineWidth', 2.5); hold on;
    h2 = plot(t_stored(1:i-1), (1.0)  * step_width * ones(1,i-1), '-.', 'Color', color_LF, 'LineWidth', 1.5);
    h3 = plot(t_stored(1:i-1), (-1.0) * step_width * ones(1,i-1), '-.', 'Color', color_RF, 'LineWidth', 1.5);
    grid on;       
    subplot2.XTick = t_stored(1):T_step:t_stored(i-1)+PARA.dt;
    subplot2.YTick = (-1.0)*step_width : 0.5*step_width : 1.0*step_width;
    xlim(subplot2, [t_stored(1), t_stored(i-1)+PARA.dt]);
    ylim(subplot2, [(1.1)*min(delcontact_stored(2,:)), (1.1)*max(delcontact_stored(2,:))]);
    xlabel(subplot2, 'time (s)');
    ylabel(subplot2, '\deltac_y (m)');
    legend(subplot2, [h1,h2,h3], {'\deltac_y','LF^{Ref}', 'RF^{Ref}'}, 'Location','northeastoutside');
    title(subplot2, '\deltac_y');
    
    %––– Fig 3: (X좌표) 현재접촉위치, 기준접촉위치, 다음접촉위치, 현재COM, 기준COM
    subplot3 = nexttile(getTile(1, 2)); 
    h1 = plot(t_stored(1:i-1), p_stored(1,1:i-1),                'b-',  'LineWidth', 2.5); hold on;
    h2 = plot(t_stored(1:i-1), p_ref_stored(1,1:i-1),            'r-.', 'LineWidth', 1.5); 
    h3 = plot(t_stored(1:i-1), p_next_foot_step_stored(1,1:i-1), 'k:',  'LineWidth', 1.5);
    h4 = plot(t_stored(1:i-1), COM_stored(1,1:i-1),              'g-',  'LineWidth', 2.5);
    h5 = plot(t_stored(1:i-1), COM_ref_stored(1,1:i-1),          'm-',  'LineWidth', 1.5);
    grid on;       
    subplot3.XTick = t_stored(1):T_step:t_stored(i-1)+PARA.dt;
    subplot3.YTick = (-1.0)*number_of_step*step_length : 0.5*step_length : 1.0*number_of_step*step_length;
    xlim(subplot3, [t_stored(1), t_stored(i-1)+PARA.dt]);
    ylim(subplot3, [(1.1)*min(p_stored(1,:)), (1.1)*max(p_stored(1,:))]);
    xlabel(subplot3, 'time (s)');
    ylabel(subplot3, 'x (m)');
    legend(subplot3, [h1,h2,h3,h4,h5], {'c_x','c^{Ref}_x','c^{Next}_x','COM_x','COM^{Ref}_x'}, 'Location','northeastoutside');
    title(subplot3, 'Footstep Location X');
    
    %––– Fig 4: (Y좌표) 현재접촉위치, 기준접촉위치, 다음접촉위치, 현재COM, 기준COM
    subplot4 = nexttile(getTile(2, 2)); 
    h1 = plot(t_stored(1:i-1), p_stored(2,1:i-1),                'b-',  'LineWidth', 2.5); hold on;
    h2 = plot(t_stored(1:i-1), p_ref_stored(2,1:i-1),            'r-.', 'LineWidth', 1.5); 
    h3 = plot(t_stored(1:i-1), p_next_foot_step_stored(2,1:i-1), 'k:',  'LineWidth', 1.5);
    h4 = plot(t_stored(1:i-1), COM_stored(2,1:i-1),              'g-',  'LineWidth', 2.5);
    h5 = plot(t_stored(1:i-1), COM_ref_stored(2,1:i-1),          'm-',  'LineWidth', 1.5);
    grid on;       
    subplot4.XTick = t_stored(1):T_step:t_stored(i-1)+PARA.dt;
    subplot4.YTick = (-1.0)*step_width : 0.5*step_width : 1.0*step_width;
    xlim(subplot4, [t_stored(1), t_stored(i-1)+PARA.dt]);
    ylim(subplot4, [(1.1)*min(p_stored(2,:)), (1.1)*max(p_stored(2,:))]);
    xlabel(subplot4, 'time (s)');
    ylabel(subplot4, 'y (m)');
    legend(subplot4, [h1,h2,h3,h4,h5], {'c_y','c^{Ref}_y','c^{Next}_y','COM_y','COM^{Ref}_y'}, 'Location','northeastoutside');
    title(subplot4, 'Footstep Location Y');
    
    
    %––– Fig 5: LF_x, RF_x
    subplot5 = nexttile(getTile(1, 3)); 
    h1 = plot(t_stored(1:i-1), LF_stored(1,1:i-1),    '-',  'Color', color_LF, 'LineWidth', 2.5); hold on;
    h2 = plot(t_stored(1:i-1), RF_stored(1,1:i-1),    '-',  'Color', color_RF, 'LineWidth', 2.5); 
    h3 = plot(t_stored(1:i-1), p_ref_stored(1,1:i-1), 'r-.'                  , 'LineWidth', 1.5);
    grid on;       
    subplot5.XTick = t_stored(1):T_step:t_stored(i-1)+PARA.dt;
    subplot5.YTick = (-1.0)*number_of_step*step_length : 0.5*step_length : 1.0*number_of_step*step_length;
    xlim(subplot5, [t_stored(1), t_stored(i-1)+PARA.dt]);
    ylim(subplot5, [(1.1)*min(p_stored(1,:)), (1.1)*max(p_stored(1,:))]);
    xlabel(subplot5, 'time (s)');
    ylabel(subplot5, 'x (m)');
    legend(subplot5, [h1,h2,h3], {'LF_x','RF_x','c^{Ref}_x'}, 'Location','northeastoutside');
    title(subplot5, 'LF_x & RF_x');
    
    %––– Fig 6: LF_y, RF_y
    subplot6 = nexttile(getTile(2, 3)); 
    h1 = plot(t_stored(1:i-1), LF_stored(2,1:i-1),    '-',  'Color', color_LF, 'LineWidth', 2.5); hold on;
    h2 = plot(t_stored(1:i-1), RF_stored(2,1:i-1),    '-',  'Color', color_RF, 'LineWidth', 2.5); 
    h3 = plot(t_stored(1:i-1), p_ref_stored(2,1:i-1), 'r-.'                  , 'LineWidth', 1.5);
    grid on;       
    subplot6.XTick = t_stored(1):T_step:t_stored(i-1)+PARA.dt;
    subplot6.YTick = (-1.0)*step_width : 0.5*step_width : 1.0*step_width;
    xlim(subplot6, [t_stored(1), t_stored(i-1)+PARA.dt]);
    ylim(subplot6, [(1.1)*min(p_stored(2,:)), (1.1)*max(p_stored(2,:))]);
    xlabel(subplot6, 'time (s)');
    ylabel(subplot6, 'y (m)');
    legend(subplot6, [h1,h2,h3], {'LF_y','RF_y','c^{Ref}_y'}, 'Location','northeastoutside');
    title(subplot6, 'LF_y & RF_y');
    
    
    
    %%
    fig2 = figure('Units','normalized', ...
                  'OuterPosition',[ (1-0.9)/2, (1-0.9*0.8)/2, 0.9, 0.9*0.8 ], ...
                  'Name','w_Analysis');
    
    %––– Fig 1: wx
    subplot(1,3,1);
    plot(1:i, w_stored(1,1:i),       'b-',  'LineWidth', 2.5); hold on;
    plot(1:i, w_ref_stored(1,1:i),   'r--', 'LineWidth', 1.5);
    grid on;
    ax = gca;         
    ax.XTick = 0:600:i;
    xlim([1 i]);
    xlabel('Step index');
    ylabel('w_x');
    legend('w_x','w\_Ref_x');
    title('w_x VS w\_Ref_x');
    
    
    %––– Fig 2: wy
    subplot(1,3,2);
    plot(1:i, w_stored(2,1:i),       'b-',  'LineWidth', 2.5); hold on;
    plot(1:i, w_ref_stored(2,1:i),   'r--', 'LineWidth', 1.5);
    grid on;
    ax = gca;         
    ax.XTick = 0:600:i;
    xlim([1 i]);
    xlabel('Step index');
    ylabel('w_y');
    legend('w_y','w\_Ref_y');
    title('w_y VS w\_Ref_y');
    
    %––– Fig 3: wz
    subplot(1,3,3);
    plot(1:i, w_stored(3,1:i),       'b-',  'LineWidth', 2.5); hold on;
    plot(1:i, w_ref_stored(3,1:i),   'r--', 'LineWidth', 1.5);
    grid on;
    ax = gca;         
    ax.XTick = 0:600:i;
    xlim([1 i]);
    xlabel('Step index');
    ylabel('w_z');
    legend('w_z','w\_Ref_z');
    title('w_z VS w\_Ref_z');

end