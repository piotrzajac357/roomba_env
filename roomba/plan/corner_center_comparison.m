clear variables; close all;
%%
% import options
opts = delimitedTextImportOptions("NumVariables", 6);
opts.DataLines = [4, Inf];
opts.Delimiter = ",";
opts.VariableNames = ["t", "Q_E", "Q_cov", "Q_O", "Q_D", "suction"];
opts.VariableTypes = ["double", "double", "double", "double", "double", "double"];
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";
opts = setvaropts(opts, ["t", "Q_E", "Q_cov", "Q_O", "Q_D", "suction"], "ThousandsSeparator", ",");
filepath = "C:\Users\zajac\Desktop\pm\roomba_env\roomba\tests_results\";

%% import data 
stc_sc_c = readtable(filepath + "log_stc_showcase_center.txt", opts);
ba_sc_c  = readtable(filepath + "log_ba_showcase_center.txt", opts);
rg_sc_c  = readtable(filepath + "log_rg_showcase_center.txt", opts);
dwa_sc_c = readtable(filepath + "log_dwa_showcase_center.txt", opts);

stc_c_c = readtable(filepath + "log_stc_clover_center.txt", opts);
ba_c_c  = readtable(filepath + "log_ba_clover_center.txt", opts);
rg_c_c  = readtable(filepath + "log_rg_clover_center.txt", opts);
dwa_c_c = readtable(filepath + "log_dwa_clover_center.txt", opts);

stc_n_c = readtable(filepath + "log_stc_narrow_center.txt", opts);
ba_n_c  = readtable(filepath + "log_ba_narrow_center.txt", opts);
rg_n_c  = readtable(filepath + "log_rg_narrow_center.txt", opts);
dwa_n_c = readtable(filepath + "log_dwa_narrow_center.txt", opts);

stc_o_c = readtable(filepath + "log_stc_obstacles_center.txt", opts);
ba_o_c  = readtable(filepath + "log_ba_obstacles_center.txt", opts);
rg_o_c  = readtable(filepath + "log_rg_obstacles_center.txt", opts);
dwa_o_c = readtable(filepath + "log_dwa_obstacles_center.txt", opts);

stc_sc_r = readtable(filepath + "log_stc_showcase_corner.txt", opts);
ba_sc_r  = readtable(filepath + "log_ba_showcase_corner.txt", opts);
rg_sc_r  = readtable(filepath + "log_rg_showcase_corner.txt", opts);
dwa_sc_r = readtable(filepath + "log_dwa_showcase_corner.txt", opts);

stc_c_r = readtable(filepath + "log_stc_clover_corner.txt", opts);
ba_c_r  = readtable(filepath + "log_ba_clover_corner.txt", opts);
rg_c_r  = readtable(filepath + "log_rg_clover_corner.txt", opts);
dwa_c_r = readtable(filepath + "log_dwa_clover_corner.txt", opts);

stc_n_r = readtable(filepath + "log_stc_narrow_corner.txt", opts);
ba_n_r  = readtable(filepath + "log_ba_narrow_corner.txt", opts);
rg_n_r  = readtable(filepath + "log_rg_narrow_corner.txt", opts);
dwa_n_r = readtable(filepath + "log_dwa_narrow_corner.txt", opts);

stc_o_r = readtable(filepath + "log_stc_obstacles_corner.txt", opts);
ba_o_r  = readtable(filepath + "log_ba_obstacles_corner.txt", opts);
rg_o_r  = readtable(filepath + "log_rg_obstacles_corner.txt", opts);
dwa_o_r = readtable(filepath + "log_dwa_obstacles_corner.txt", opts);

%% plot some
figure(1);
sgtitle('Plan: 1, Miejsce startu: lewy dolny róg')
subplot(221)
plot(dwa_n_c.t./60,dwa_n_c.Q_E);
hold on; grid on
plot(dwa_o_c.t./60,dwa_o_c.Q_E);
title('procent zużycia baterii');
xlabel('t[min]'), ylabel('Q_E[%]'),legend('center','corner',"Location",'best')
axis([0 60 0 100]);

subplot(222)
plot(dwa_n_c.t./60,dwa_n_c.Q_D);
hold on; grid on
plot(dwa_o_c.t./60,dwa_o_c.Q_D);
title('przebyta droga');
xlabel('t[min]'), ylabel('Q_D[m]'), legend('STC','BA*',"Location",'best')
axis([0 60 0 1200]);

subplot(223)
plot(dwa_n_c.t./60,dwa_n_c.Q_cov);
hold on; grid on
plot(dwa_o_c.t./60,dwa_o_c.Q_cov);
title('procent pokrycia');
xlabel('t[min]'), ylabel('Q_c_o_v[%]'), legend('STC','BA*',"Location",'best')
axis([0 60 0 100]);

subplot(224)
plot(dwa_n_c.t./60,dwa_n_c.Q_O);
hold on; grid on
plot(dwa_o_c.t./60,dwa_o_c.Q_O);
title('wykonane pełne obroty');
xlabel('t[min]'), ylabel('Q_O[obroty]'), legend('STC','BA*',"Location",'best')
axis([0 60 0 120]);






