function comparePlots(F, deg)
    
    projectRoot = pwd; % 프로젝트 루트는 현재 폴더(pwd) 
    D      = dir(projectRoot); % 돌릴 서브디렉토리 목록 (matlab_ws 제외)
    names  = {D.name};
    isDir  = [D.isdir];
    skip   = ismember(names, {'.','..','matlab_ws', '.git'});
    subdirs= names(isDir & ~skip);
    workspaceDir = fullfile(projectRoot, 'matlab_ws'); % 각 서브폴더 main 실행 → 지정 변수 저장
    varsToSave = {'COM_stored','delcontact_stored','p_stored'};
    for k = 1:numel(subdirs)
        thisDir = fullfile(projectRoot, subdirs{k});
        fprintf('>> Running %s/main.m ...\n', subdirs{k});
        cd(thisDir);
        run('main.m');
        save(fullfile(workspaceDir, [subdirs{k} '_workspace.mat']), varsToSave{:});
    end
    cd(projectRoot); % 다시 프로젝트 루트로 돌아오기

%     clc; clear all; close all; % .mat 파일 불러와 비교 플롯
%     d1 = load(fullfile(workspaceDir, 'SRBD_MPC_workspace.mat'));
%     d2 = load(fullfile(workspaceDir, 'aSRBD_OAMPC_workspace.mat'));
%     figure; hold on;
%       plot(d1.COM(1,:), d1.COM(2,:), '-o', 'DisplayName','SRBD\_MPC');
%       plot(d2.COM(1,:), d2.COM(2,:), '-x', 'DisplayName','aSRBD\_OAMPC');
%     hold off;
%     xlabel('COM X [m]');  ylabel('COM Y [m]');
%     title('Center of Mass Trajectory Comparison');
%     legend('Location','best');
%     grid on;
end



d1.delcontact_stored
d1.step_length
d1.step_width
d1.PARA.dt
d1.T_step
d1.p_stored
d1.p_ref_stored
d1.p_next_foot_step_stored
d1.COM_stored



t_step = d1.t_step; T_step = d1.T_step;
step_legnth = d1.step_length; step_width = d1.step_width;
d1.COM_stored(1:), d2.COM_stored(1:)
d1.COM_stored(2:), d2.COM_stored(2:)
d1.theta_stored(1:), d2.theta_stored(1:)
d1.theta_stored(2:), d2.theta_stored(2:)














% topDir = pwd;
% D      = dir(topDir);
% names    = {D.name};
% isDir    = [D.isdir];
% isDot    = ismember(names,{'.','..'});
% isHidden = strncmp('.', names, 1);
% subdirs = names(isDir & ~isDot & ~isHidden);
% 
% for k = 1:numel(subdirs)
%     thisDir = fullfile(topDir, subdirs{k});
%     origDir = pwd;
%     cd(thisDir);              % ← 여기서 작업 폴더를 main.m이 있는 곳으로
%     run('main.m');            % ← 이제 F5와 똑같이 실행됩니다!
%     cd(origDir);              % ← 끝나면 돌아오기
% 
%     % 결과 저장
%     varsToSave = {'COM_stored','delcontact_stored','p_stored'};
%     outName    = fullfile(topDir, ['results_' subdirs{k} '.mat']);
%     save(outName, varsToSave{:});
% end


%%

% figure; hold on;
% colors = lines(numel(fieldnames(results)));
% names  = fieldnames(results);
% for k = 1:numel(names)
%     data = results.(names{k});
%     % 예: COM x축 궤적만 그려보기
%     plot(data.COM(1,:), 'LineWidth',2,'Color',colors(k,:));
% end
% legend(names,'Interpreter','none');
% xlabel('time step'); ylabel('COM\_x');
% title('각 MPC 버전 COM\_x 비교');
% 
% 
% 
% 
% 
% 
% %%
% 
% restoredefaultpath;                  % ① 깨끗한 기본 경로 상태로
% folder = fileparts(which('main.m')); % ② main.m 있는 폴더 찾고
% addpath(genpath(folder));           % ③ 그 폴더 + 하위폴더 전부 경로에 추가
% 
% 
% 
% %%
% % (1) 두 개의 main 파일 실행
% run(fullfile(topDir,'aSRBD_MPC','main.m'));
% save('tmp1.mat','COM_stored','delcontact_stored');
% clearvars -except topDir
% 
% run(fullfile(topDir,'SRBD_MPC','main.m'));
% save('tmp2.mat','COM_stored','delcontact_stored');
% 
% % (2) 저장된 .mat 불러와서 비교 플롯
% d1 = load('tmp1.mat');  d2 = load('tmp2.mat');
% figure; plot(d1.COM_stored(1,:),'-r'); hold on; plot(d2.COM_stored(1,:),'-b');
% legend('aSRBD','SRBD');