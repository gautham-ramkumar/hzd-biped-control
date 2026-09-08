% diagnostic_anim.m - check step detection and save animation frames
set_path;

f = [-0.4907, -2.3947, 0.3053, 0.6256, 0.8814, 2.6360, 2.7919, 2.9537];
alpha2 = [-f(5), -f(4), f(3:5)];
alpha3 = [-f(8)+2*f(6), -f(7)+2*f(6), f(6:8)];
a = [alpha2, alpha3];
x_max = f(1); x_min = -f(1); delq = x_max - x_min;

q2m  = alpha2(5); q3m  = alpha3(5);
dq2m = 4*(alpha2(5)-alpha2(4))*f(2)/delq;
dq3m = 4*(alpha3(5)-alpha3(4))*f(2)/delq;
x_minus = [f(1), q2m, q3m, f(2), dq2m, dq3m];

[x_plus0, ~] = func_impact_map(x_minus);
dq1p = x_plus0(4);
x_plus0(2) = bezier(0, 4, alpha2);
x_plus0(3) = bezier(0, 4, alpha3);
x_plus0(5) = d_ds_bezier(0, 4, alpha2) * dq1p / delq;
x_plus0(6) = d_ds_bezier(0, 4, alpha3) * dq1p / delq;
s_params = [x_plus0(1), x_max];

ti = 0; tf = 5;
n = 5;
t_tot = []; x_tot = [];
xp = x_plus0; sp = s_params;

[r,m,Mh,Mt,l,g] = func_model_params; params = [r,m,Mh,Mt,l,g];

for i = 1:n
    opts = odeset('Event', @(t,x) evfn(t,x,sp(1),sp(2)), 'AbsTol',1e-4,'RelTol',1e-4);
    [t, xs] = ode45(@(t,x) func_full_dynamics(t,x,a,sp), [ti,tf], xp, opts);
    t_tot = [t_tot; t];
    x_tot = [x_tot; xs];
    fprintf('Step %d: t=[%.4f..%.4f]  q1_end=%.4f  frames=%d\n', ...
            i, t(1), t(end), xs(end,1), length(t));
    [xp2,~] = func_impact_map(xs(end,:));
    dq1n = xp2(1,4);
    xp2(1,2) = bezier(0,4,alpha2);
    xp2(1,3) = bezier(0,4,alpha3);
    xp2(1,5) = d_ds_bezier(0,4,alpha2)*dq1n/delq;
    xp2(1,6) = d_ds_bezier(0,4,alpha3)*dq1n/delq;
    xp = xp2(1,:);  sp = [xp(1), x_max];
end

fprintf('\nt_tot length = %d\n', length(t_tot));
fprintf('t_tot first 5: '); fprintf('%.4f ', t_tot(1:5)'); fprintf('\n');

% Check step starts via t<=t_prev
step_starts = 1;
for ii = 2:length(t_tot)
    if t_tot(ii) <= t_tot(ii-1)
        step_starts(end+1) = ii;
    end
end
fprintf('Steps detected = %d, at frames: ', length(step_starts));
fprintf('%d ', step_starts); fprintf('\n');

% Compute expected L_step per step
fprintf('\nExpected world stance foot positions per step:\n');
L_cur = 0;
for s = 1:length(step_starts)
    ss = step_starts(s);
    if s < length(step_starts)
        se = step_starts(s+1) - 1;
    else
        se = length(t_tot);
    end
    [~,~,~,~,~,P2e] = func_compute_pMh_pMt_pm1_pm2_pcm_P2(x_tot(se,1:3),x_tot(se,4:6),params);
    fprintf('  Step %d: frames [%d..%d], L_cur=%.3f, P2_end.x=%.3f → next L=%.3f\n', ...
            s, ss, se, L_cur, P2e(1), L_cur+P2e(1));
    if s < length(step_starts)
        L_cur = L_cur + P2e(1);
    end
end

% Save 3 snapshot frames: start/mid/end of each step
fprintf('\nSaving animation snapshots...\n');
snap_steps = [1, 3, 5];
L_snap = 0;
step_L = zeros(1, length(step_starts));
L_c = 0;
for s = 1:length(step_starts)
    step_L(s) = L_c;
    ss = step_starts(s);
    if s < length(step_starts); se=step_starts(s+1)-1; else; se=length(t_tot); end
    [~,~,~,~,~,P2e]=func_compute_pMh_pMt_pm1_pm2_pcm_P2(x_tot(se,1:3),x_tot(se,4:6),params);
    if s < length(step_starts); L_c = L_c + P2e(1); end
end

fig = figure('Visible','off','Position',[0 0 800 700]);
ah = axes('Parent',fig);
hold(ah,'on'); 
% Set axis limits similar to reference image
xlim(ah,[-0.2, 2.2]); ylim(ah,[-0.3 2.0]);
xlabel(ah,'[m]'); ylabel(ah,'[m]');
set(ah, 'FontSize', 12);

for s = 1:2
    ss = step_starts(s);
    if s < length(step_starts); se=step_starts(s+1)-1; else; se=length(t_tot); end
    % 4 frames per step
    frames = round(linspace(ss, se, 4));
    
    for k = 1:length(frames)
        idx = frames(k);
        q=x_tot(idx,1:3); dq=x_tot(idx,4:6);
        [pMh,pMt,pm1,pm2,~,P2]=func_compute_pMh_pMt_pm1_pm2_pcm_P2(q,dq,params);
        Ls = [step_L(s); 0];
        
        if mod(s,2)==1; cs='r'; cw='g'; else; cs='g'; cw='r'; end
        
        % Plot links
        line(ah,Ls(1)+[0,pMh(1)],  Ls(2)+[0,pMh(2)],   'Color',cs,'LineWidth',1,'LineStyle','--');
        line(ah,Ls(1)+[pMh(1),P2(1)],Ls(2)+[pMh(2),P2(2)],'Color',cw,'LineWidth',1,'LineStyle','--');
        line(ah,Ls(1)+[pMh(1),pMt(1)],Ls(2)+[pMh(2),pMt(2)],'Color','k','LineWidth',1,'LineStyle','--');
        
        % Plot point masses as circles
        line(ah,Ls(1)+pm1(1), Ls(2)+pm1(2), 'Color',cs,'Marker','o','MarkerSize',8, 'LineStyle', 'none');
        line(ah,Ls(1)+pm2(1), Ls(2)+pm2(2), 'Color',cw,'Marker','o','MarkerSize',8, 'LineStyle', 'none');
        line(ah,Ls(1)+pMt(1), Ls(2)+pMt(2), 'Color','k','Marker','o','MarkerSize',8, 'LineStyle', 'none');
        line(ah,Ls(1)+pMh(1), Ls(2)+pMh(2), 'Color','k','Marker','o','MarkerSize',8, 'LineStyle', 'none');
        
        % Plot feet as solid dots
        line(ah, Ls(1), Ls(2), 'Color', cs, 'Marker', '.', 'MarkerSize', 15, 'LineStyle', 'none');
        line(ah, Ls(1)+P2(1), Ls(2)+P2(2), 'Color', cw, 'Marker', '.', 'MarkerSize', 15, 'LineStyle', 'none');
    end
    
    % Draw impact line at the end of the step
    if s == 1
        impact_x = Ls(1) + P2(1);
        line(ah, [impact_x, impact_x], [-0.3, 2.0], 'Color', [0.6 0.6 0.6], 'LineStyle', '--', 'LineWidth', 1.5);
        text(ah, impact_x - 0.05, 1.85, 'impact', 'Color', [0.4 0.4 0.4], 'HorizontalAlignment', 'right', 'FontSize', 11);
    end
end
saveas(fig, 'anim_snapshot.png');
fprintf('Saved anim_snapshot.png\n');

function [lim,term,dir] = evfn(~,x,q1min,q1max)
    s = (x(1)-q1min)/(q1max-q1min);
    if s >= 1; s = 1; end
    lim = s-1; term = 1; dir = [];
end
