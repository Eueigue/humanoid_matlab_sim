clc; clear all; close all;

% Load Workspace
S_1 = load("extModel_m_000_0.mat");
S_2 = load("SRBD_F_000_deg_000_m_017_0.mat");
S_3 = load("extModel_m_017_0.mat");

% Shared Variables
t_stored = S_1.t_stored(1:7802);
number_of_step = S_1.number_of_step;
step_length = S_1.step_length;
step_width = S_1.step_width;
T_step = S_1.T_step;
Impact_t_idx = 2701;
Ext_t_idx = S_1.Ext_t_idx;

% Plot

%--- Font, Line Style ---%
AXFS   = 18;    % Axis Font Size (Tick)
LBFS   = 20;    % Lable Font Size
TLFS   = 25;    % Title Font Size
LGFS   = 18;    % Legend Font Size
LW     = 2.5;   % Line Width
LW_REF = 2.5;   % Line Width (Reference, Dot Line)

fig1 = figure('Name','Ext_Model');
numRows = 2;
numCols = 2;
tile = tiledlayout(numRows, numCols); 
% tile.TileSpacing = 'compact';
% tile.Padding = 'compact';
getTile = @(row, col) (row - 1) * numCols + col;

%––– Fig 1: COM_x
subplot1 = nexttile(getTile(1, 1));  
% h1 = plot(t_stored(1:S_1.i-1), S_1.COM_stored(1,1:S_1.i-1),      'b-',  'LineWidth', LW); hold on;
h2 = plot(t_stored(1:S_2.i-1), S_2.COM_stored(1,1:S_2.i-1),      'g-',  'LineWidth', LW); hold on;
h3 = plot(t_stored(1:S_3.i-1), S_3.COM_stored(1,1:S_3.i-1),      'r-',  'LineWidth', LW);
h4 = plot(t_stored(1:S_1.i-1), S_1.COM_ref_stored(1,1:S_1.i-1),  ':k',  'LineWidth', LW_REF);
h5 = xline(subplot1, t_stored(Ext_t_idx),                        ':r',  'LineWidth', LW_REF);
grid on;       
subplot1.XTick = t_stored(1):T_step:t_stored(S_1.i-1)+PARA.dt;
% subplot1.YTick = [0, 0.5*step_length, 1.5*step_length:step_length:step_length*(number_of_step-1.5), step_length*(number_of_step-1)];
subplot1.YTick = [0:1:2, 2.7];
xlim(subplot1, [t_stored(1), t_stored(S_1.i-1)+PARA.dt]);
% ylim(subplot1, [(1.1)*min(S_1.p_stored(1,:)), (1.1)*max(S_1.p_stored(1,:))]);
ylim(subplot1, [0, 2.8]);
set(subplot1, 'Fontsize', AXFS);
xlabel(subplot1, 'time (s)', 'FontSize', LBFS, 'FontWeight', 'bold');
ylabel(subplot1, 'x (m)', 'FontSize', LBFS, 'FontWeight', 'bold');
legend(subplot1, [h2,h3,h4,h5], {'SRBD','aSRBD', 'COM^{Ref}_x', 'Object'}, 'Location','northeastoutside', 'FontSize', LGFS);
title(subplot1, 'COM_{X}', 'FontSize', TLFS, 'FontWeight', 'bold');
% set(subplot1, 'Fontsize', AXFS);

%-- Fig 2: COM_z
subplot2 = nexttile(getTile(2, 1)); 
% h1 = plot(t_stored(1:S_1.i-1), S_1.COM_stored(3,1:S_1.i-1),     'b-',  'LineWidth', LW); hold on;
h2 = plot(t_stored(1:S_2.i-1), S_2.COM_stored(3,1:S_2.i-1),     'g-',  'LineWidth', LW); hold on;
h3 = plot(t_stored(1:S_3.i-1), S_3.COM_stored(3,1:S_3.i-1),     'r-',  'LineWidth', LW);
h4 = plot(t_stored(1:S_1.i-1), S_1.COM_ref_stored(3,1:S_1.i-1), ':k',  'LineWidth', LW_REF);
h5 = xline(subplot2, t_stored(Ext_t_idx),                       ':r',  'LineWidth', LW_REF);
grid on;       
subplot2.XTick = t_stored(1):T_step:t_stored(S_1.i-1)+PARA.dt;
subplot2.YTick = 0.65:0.04:0.81;
xlim(subplot2, [t_stored(1), t_stored(S_1.i-1)+PARA.dt]);
ylim(subplot2, [0.65, 0.81]);
set(subplot2, 'Fontsize', AXFS);
xlabel(subplot2, 'time (s)', 'FontSize', LBFS, 'FontWeight', 'bold');
ylabel(subplot2, 'z (m)', 'FontSize', LBFS, 'FontWeight', 'bold');
% legend(subplot2, [h1,h2,h3,h4,h5], {'m = 0kg','m = 8kg','m = 17kg', 'COM^{Ref}_z', 'Object'}, 'Location','northeastoutside', 'FontSize', LGFS);
legend(subplot2, [h2,h3,h4,h5], {'SRBD','aSRBD', 'COM^{Ref}_z', 'Object'}, 'Location','northeastoutside', 'FontSize', LGFS);
title(subplot2, 'COM_{Z}', 'FontSize', TLFS, 'FontWeight', 'bold');

%-- Fig 3: Ground Reaction Force

% --- Critical Value: ---
eps0 = 1e-3;

t1 = t_stored(1:S_1.i-1);
t2 = t_stored(1:S_2.i-1);
t3 = t_stored(1:S_3.i-1);
f1 = S_1.fL_stored(3,1:S_1.i-1) + S_1.fR_stored(3,1:S_1.i-1);
f2 = S_2.fL_stored(3,1:S_2.i-1) + S_2.fR_stored(3,1:S_2.i-1);
f3 = S_3.fL_stored(3,1:S_3.i-1) + S_3.fR_stored(3,1:S_3.i-1);

subplot3 = nexttile(getTile(1, 2), [2 1]); 
% h1 = plot(t1, f1,                         'b-',  'LineWidth', LW); hold on;
h2 = plot(t2, f2,                         'g-',  'LineWidth', LW); hold on;
h3 = plot(t3, f3,                         'r-',  'LineWidth', LW);
h4 = yline(subplot3, PARA.f_z_max,        ':k',  'LineWidth', LW_REF);
h5 = xline(subplot3, t_stored(Ext_t_idx), ':r',  'LineWidth', LW_REF);
grid on;       
subplot3.XTick = t_stored(1):T_step:t_stored(S_1.i-1)+PARA.dt;
subplot3.YTick = 0:100:600;
xlim(subplot3, [t_stored(1), t_stored(S_1.i-1)+PARA.dt]);
ylim(subplot3, [0, 650]);
set(subplot3, 'Fontsize', AXFS);
xlabel(subplot3, 'time (s)', 'FontSize', LBFS, 'FontWeight', 'bold');
ylabel(subplot3, 'f_z (N)', 'FontSize', LBFS, 'FontWeight', 'bold');
legend(subplot3, [h2,h3,h4,h5], {'SRBD','aSRBD', "f_{z, max}", 'Object'}, 'Location','northeastoutside', 'FontSize', LGFS);
title(subplot3, 'Ground\_Reaction\_Force\_Z', 'FontSize', TLFS, 'FontWeight', 'bold');

set(fig1, 'WindowState', 'maximized');
