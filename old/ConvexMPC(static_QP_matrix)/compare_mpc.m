clear; clc; close all;
rootPath = pwd;
oldPath = fullfile(rootPath, 'old/old_fixed_B_zero_constraints');
nTrials = 20; 
nSteps = 400; % 한 회당 스텝 수

% 데이터 전체를 담을 행렬 초기화 (Trial x Step)
raw_new_times = zeros(nTrials, nSteps);
raw_old_times = zeros(nTrials, nSteps);

for trial = 1:nTrials
    fprintf('\n[Trial %d/%d] 실행 중...\n', trial, nTrials);

    addpath(fullfile(rootPath, 'genRef'), fullfile(rootPath, 'getMPC'), fullfile(rootPath, 'sim'), fullfile(rootPath, 'utils'));
    clear functions; 
    main; % 여기서 time_history가 생성됨
    raw_new_times(trial, :) = time_history(1, :); % 통째로 저장
    rmpath(fullfile(rootPath, 'genRef'), fullfile(rootPath, 'getMPC'), fullfile(rootPath, 'sim'), fullfile(rootPath, 'utils'));

    cd(oldPath); 
    addpath(fullfile(oldPath, 'genRef'), fullfile(oldPath, 'getMPC'), fullfile(oldPath, 'sim'), fullfile(oldPath, 'utils'));
    clear functions;
    old_main; 
    raw_old_times(trial, :) = time_history(1, :); % 통째로 저장
    rmpath(fullfile(oldPath, 'genRef'), fullfile(oldPath, 'getMPC'), fullfile(oldPath, 'sim'), fullfile(oldPath, 'utils'));
    cd(rootPath);
    
    clearvars -except rootPath oldPath nTrials nSteps raw_new_times raw_old_times;
end

% ms 단위 변환
raw_new_times = raw_new_times * 1000;
raw_old_times = raw_old_times * 1000;

% 각 Trial별 평균/최대값
new_trial_means = mean(raw_new_times, 2); 
old_trial_means = mean(raw_old_times, 2);
new_trial_maxs  = max(raw_new_times, [], 2);
old_trial_maxs  = max(raw_old_times, [], 2);

total_avg_new = mean(raw_new_times, 'all');
total_std_new = std(raw_new_times, 0, 'all');

avg_new_per_step = mean(raw_new_times, 1); 
avg_old_per_step = mean(raw_old_times, 1);

std_new_per_step = std(raw_new_times, 0, 1);
std_old_per_step = std(raw_old_times, 0, 1);

figure('Color', 'w', 'Name', 'Step-wise Analysis');
hold on; grid on;

% Old Version Plot
plot(1:nSteps, avg_old_per_step);
% New Version Plot
plot(1:nSteps, avg_new_per_step);


xlabel('Iteration (Step)');
ylabel('Computation Time (ms)');
title(sprintf('Average Computation Time over %d Trials', nTrials));
legend('Location', 'northeast');

ylim([0, max([avg_new_per_step, avg_old_per_step])*1.5]); 

%% 2. Trial별 평균 연산 시간 박스 플롯 추가
figure('Color', 'w', 'Name', 'Trial-wise Mean Distribution');

% 박스 플롯을 위해 데이터를 결합 (각 열이 하나의 그룹이 됨)
% old_trial_means와 new_trial_means는 nTrials x 1 벡터임
plotData = [old_trial_means, new_trial_means];

% 박스 플롯 그리기
boxplot(plotData, 'Labels', {'Old Version', 'New Version'}, 'Whisker', 1.5);

% 그래프 세부 설정
grid on;
ylabel('Average Computation Time (ms)');
title(sprintf('Distribution of Average Time per Trial (n=%d)', nTrials));

% 시각적 가독성 향상 (선 굵기 등)
set(gca, 'FontSize', 11);

hold off;