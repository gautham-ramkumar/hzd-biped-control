%
% Input:
%       t: time(s) for the corresponding gait in x
%       x: [q1, q2, q3, dq1, dq2, dq3]
%
% angles are in radians, velocities are in rad/s
%

function animate_results(t,x)

% Time is always 0 at the begining of an ODE45 solution, so it is the most
% consistent way to seperate phases and steps
%
% Find index when time = 0:
ind0 = find(t == 0);
% Number of steps taken:
n = length(ind0);

% Add extra element to index to help with changing origin after each step
ind0(length(ind0)+1) = length(t)+1;

% Get model paramters
[r,m,Mh,Mt,l,g] = func_model_params;
params = [r,m,Mh,Mt,l,g];

%%% Estimate x axis limits for animation window
% Use last frame of first step (pre-impact) for accurate step length
j = ind0(2) - 1;
[~,~,~,~,~, P2_end] = func_compute_pMh_pMt_pm1_pm2_pcm_P2(x(j,1:3),x(j,4:6),params);
delta = 0.5;
step_length = abs(P2_end(1));
xlim_min = -delta;
xlim_max = n * step_length + delta;

% Defining figure properties
fh = figure('Name','3 link biped model in the sagittal plane',...
    'Renderer','opengl',...
    'GraphicsSmoothing','on');
ah = axes('Box','on',...
    'XGrid','off',...
    'YGrid','off',...
    'DataAspectRatio',[1,1,1],...
    'PlotBoxAspectRatio',[1,1,1],...
    'Parent',fh);
xlabel(ah,'[m]');
ylabel(ah,'[m]');
xlim(ah, [xlim_min, xlim_max]);   % fixed: was [-xlim_max,-xlim_min] (flipped)
ylim(ah,[-0.5 3]);

hold(ah,'off'); % To animate

L_step = [0; 0];
step_num = 1;   % track step number to alternate leg colors

for i = 1:length(t)
    
    q = x(i,1:3);
    dq = x(i,4:6);
    
    [pMh,pMt,pm1,pm2,~, P2] = func_compute_pMh_pMt_pm1_pm2_pcm_P2(q,dq,params);
    
    % If it is the 1st step, origin should be at (0,0)
    if i < ind0(2)
        L_step = [0; 0];
        step_num = 1;
    % If step number > 1, origin must be shifted by step length of previous step
    elseif sum(i == ind0) && i ~= 1 % True only if i matches one of the entries of ind0 except 1
        j = i-1;  % last frame of previous step (pre-impact) — used only for step length
        [~,~,~,~,~, P2_prev] = func_compute_pMh_pMt_pm1_pm2_pcm_P2(x(j,1:3),x(j,4:6),params);
        L_step = [L_step(1) + P2_prev(1); 0];  % advance origin by step x-length; y stays on ground
        step_num = step_num + 1;
    end
    
    % After each impact the stance/swing roles swap.
    % On odd steps:  pm1=stance(red),  pm2=swing(green)
    % On even steps: pm1=swing(green), pm2=stance(red)
    if mod(step_num, 2) == 1
        c_stance = 'r'; c_swing = 'g';
        p_stance = pm1; p_swing = pm2;
    else
        c_stance = 'g'; c_swing = 'r';
        p_stance = pm2; p_swing = pm1;
    end
    
    % Clear figure 
    cla(ah);
    % Add dots to beginning of stance leg and end of swing leg
    line(ah, [L_step(1),L_step(1)], [L_step(2),L_step(2)],'Color',c_stance,'Marker','.')
    line(ah, L_step(1)+[P2(1),P2(1)], L_step(2)+[P2(2),P2(2)],'Color',c_swing,'Marker','.')
    
    % Plot links as lines
    line(ah, L_step(1)+[0,pMh(1)], L_step(2)+[0,pMh(2)],'Color',c_stance,'LineStyle','--','LineWidth',1.5)
    line(ah, L_step(1)+[pMh(1),pMt(1)], L_step(2)+[pMh(2),pMt(2)],'Color','k','LineStyle','--','LineWidth',1.5)
    line(ah, L_step(1)+[pMh(1),P2(1)], L_step(2)+[pMh(2),P2(2)],'Color',c_swing,'LineStyle','--','LineWidth',1.5)
    
    % Plot point masses as circles
    line(ah, L_step(1)+[p_stance(1),p_stance(1)], L_step(2)+[p_stance(2),p_stance(2)],'Color',c_stance,'Marker','o','MarkerSize',8)
    line(ah, L_step(1)+[p_swing(1),p_swing(1)], L_step(2)+[p_swing(2),p_swing(2)],'Color',c_swing,'Marker','o','MarkerSize',8)
    line(ah, L_step(1)+[pMh(1),pMh(1)], L_step(2)+[pMh(2),pMh(2)],'Color','k','Marker','o','MarkerSize',10)
    line(ah, L_step(1)+[pMt(1),pMt(1)], L_step(2)+[pMt(2),pMt(2)],'Color','k','Marker','o','MarkerSize',10)
    
    title(ah, sprintf('Step %d', step_num));
    %legend([p1, p2, p3],{'stance leg','swing leg','torso'});

    drawnow limitrate
    %pause(0.02);
end