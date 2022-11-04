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

%% import data - plan_showcase_center
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


%% plot results - sc
figure(1);
sgtitle('Plan: 1, Miejsce startu: środek')
subplot(221)
plot(stc_sc_c.t./60,stc_sc_c.Q_E);
hold on; grid on
plot(ba_sc_c.t./60,ba_sc_c.Q_E);
plot(rg_sc_c.t./60,rg_sc_c.Q_E);
plot(dwa_sc_c.t./60,dwa_sc_c.Q_E);
title('procent zużycia baterii');
xlabel('t[min]'), ylabel('Q_E[%]'),legend('STC','BA*','RG','DWA*',"Location",'best')
axis([0 60 0 100]);

subplot(222)
plot(stc_sc_c.t./60,stc_sc_c.Q_D);
hold on; grid on
plot(ba_sc_c.t./60,ba_sc_c.Q_D);
plot(rg_sc_c.t./60,rg_sc_c.Q_D);
plot(dwa_sc_c.t./60,dwa_sc_c.Q_D);
title('przebyta droga');
xlabel('t[min]'), ylabel('Q_D[m]'), legend('STC','BA*','RG','DWA*',"Location",'best')
axis([0 60 0 1200]);

subplot(223)
plot(stc_sc_c.t./60,stc_sc_c.Q_cov);
hold on; grid on
plot(ba_sc_c.t./60,ba_sc_c.Q_cov);
plot(rg_sc_c.t./60,rg_sc_c.Q_cov);
plot(dwa_sc_c.t./60,dwa_sc_c.Q_cov);
title('procent pokrycia');
xlabel('t[min]'), ylabel('Q_c_o_v[%]'), legend('STC','BA*','RG','DWA*',"Location",'best')
axis([0 60 0 100]);

subplot(224)
plot(stc_sc_c.t./60,stc_sc_c.Q_O);
hold on; grid on
plot(ba_sc_c.t./60,ba_sc_c.Q_O);
plot(rg_sc_c.t./60,rg_sc_c.Q_O);
plot(dwa_sc_c.t./60,dwa_sc_c.Q_O);
title('wykonane pełne obroty');
xlabel('t[min]'), ylabel('Q_O[obroty]'), legend('STC','BA*','RG','DWA*',"Location",'best')
axis([0 60 0 120]);

%% plot results - clover
figure(2);
sgtitle('Plan: 2, Miejsce startu: środek')
subplot(221)
plot(stc_c_c.t./60,stc_c_c.Q_E);
hold on; grid on
plot(ba_c_c.t./60,ba_c_c.Q_E);
plot(rg_c_c.t./60,rg_c_c.Q_E);
plot(dwa_c_c.t./60,dwa_c_c.Q_E);
title('procent zużycia baterii');
xlabel('t[min]'), ylabel('Q_E[%]'), legend('STC','BA*','RG','DWA*',"Location",'best')
axis([0 60 0 100]);

subplot(222)
plot(stc_c_c.t./60,stc_c_c.Q_D);
hold on; grid on
plot(ba_c_c.t./60,ba_c_c.Q_D);
plot(rg_c_c.t./60,rg_c_c.Q_D);
plot(dwa_c_c.t./60,dwa_c_c.Q_D);
title('przebyta droga');
xlabel('t[min]'), ylabel('Q_D[m]'), legend('STC','BA*','RG','DWA*',"Location",'best')
axis([0 60 0 1200]);

subplot(223)
plot(stc_c_c.t./60,stc_c_c.Q_cov);
hold on; grid on
plot(ba_c_c.t./60,ba_c_c.Q_cov);
plot(rg_c_c.t./60,rg_c_c.Q_cov);
plot(dwa_c_c.t./60,dwa_c_c.Q_cov);
title('procent pokrycia');
xlabel('t[min]')
ylabel('Q_c_o_v[%]')
legend('STC','BA*','RG','DWA*',"Location",'best')
axis([0 60 0 100]);

subplot(224)
plot(stc_c_c.t./60,stc_c_c.Q_O);
hold on; grid on
plot(ba_c_c.t./60,ba_c_c.Q_O);
plot(rg_c_c.t./60,rg_c_c.Q_O);
plot(dwa_c_c.t./60,dwa_c_c.Q_O);
title('wykonane pełne obroty');
xlabel('t[min]'), ylabel('Q_O[obroty]'), legend('STC','BA*','RG','DWA*',"Location",'best')
axis([0 60 0 120]);

%% plot results - narrow
figure(3);
sgtitle('Plan: 3, Miejsce startu: środek')
subplot(221)
plot(stc_n_c.t./60,stc_n_c.Q_E);
hold on; grid on
plot(ba_n_c.t./60,ba_n_c.Q_E);
plot(rg_n_c.t./60,rg_n_c.Q_E);
plot(dwa_n_c.t./60,dwa_n_c.Q_E);
title('procent zużycia baterii');
xlabel('t[min]'), ylabel('Q_E[%]'), legend('STC','BA*','RG','DWA*',"Location",'best')
axis([0 65 0 100]);

subplot(222)
plot(stc_n_c.t./60,stc_n_c.Q_D);
hold on; grid on
plot(ba_n_c.t./60,ba_n_c.Q_D);
plot(rg_n_c.t./60,rg_n_c.Q_D);
plot(dwa_n_c.t./60,dwa_n_c.Q_D);
title('przebyta droga');
xlabel('t[min]'), ylabel('Q_D[m]'), legend('STC','BA*','RG','DWA*',"Location",'best')
axis([0 65 0 1200]);

subplot(223)
plot(stc_n_c.t./60,stc_n_c.Q_cov);
hold on; grid on
plot(ba_n_c.t./60,ba_n_c.Q_cov);
plot(rg_n_c.t./60,rg_n_c.Q_cov);
plot(dwa_n_c.t./60,dwa_n_c.Q_cov);
title('procent pokrycia');
xlabel('t[min]'), ylabel('Q_c_o_v[%]'), legend('STC','BA*','RG','DWA*',"Location",'best')
axis([0 65 0 100]);

subplot(224)
plot(stc_n_c.t./60,stc_n_c.Q_O);
hold on; grid on
plot(ba_n_c.t./60,ba_n_c.Q_O);
plot(rg_n_c.t./60,rg_n_c.Q_O);
plot(dwa_n_c.t./60,dwa_n_c.Q_O);
title('wykonane pełne obroty');
xlabel('t[min]'), ylabel('Q_O[obroty]'), legend('STC','BA*','RG','DWA*',"Location",'best')
axis([0 65 0 120]);

%% plot results - obstacles
figure(4);
sgtitle('Plan: 4, Miejsce startu: środek')
subplot(221)
plot(stc_o_c.t./60,stc_o_c.Q_E);
hold on; grid on
plot(ba_o_c.t./60,ba_o_c.Q_E);
plot(rg_o_c.t./60,rg_o_c.Q_E);
plot(dwa_o_c.t./60,dwa_o_c.Q_E);
title('procent zużycia baterii');
xlabel('t[min]'), ylabel('Q_E[%]'), legend('STC','BA*','RG','DWA*',"Location",'best')
axis([0 60 0 100]);

subplot(222)
plot(stc_o_c.t./60,stc_o_c.Q_D);
hold on; grid on
plot(ba_o_c.t./60,ba_o_c.Q_D);
plot(rg_o_c.t./60,rg_o_c.Q_D);
plot(dwa_o_c.t./60,dwa_o_c.Q_D);
title('przebyta droga');
xlabel('t[min]'), ylabel('Q_D[m]'), legend('STC','BA*','RG','DWA*',"Location",'best')
axis([0 60 0 1200]);

subplot(223)
plot(stc_o_c.t./60,stc_o_c.Q_cov);
hold on; grid on
plot(ba_o_c.t./60,ba_o_c.Q_cov);
plot(rg_o_c.t./60,rg_o_c.Q_cov);
plot(dwa_o_c.t./60,dwa_o_c.Q_cov);
title('procent pokrycia');
xlabel('t[min]'), ylabel('Q_c_o_v[%]'), legend('STC','BA*','RG','DWA*',"Location",'best')
axis([0 60 0 100]);

subplot(224)
plot(stc_o_c.t./60,stc_o_c.Q_O);
hold on; grid on
plot(ba_o_c.t./60,ba_o_c.Q_O);
plot(rg_o_c.t./60,rg_o_c.Q_O);
plot(dwa_o_c.t./60,dwa_o_c.Q_O);
title('wykonane pełne obroty');
xlabel('t[min]'), ylabel('Q_O[obroty]'), legend('STC','BA*','RG','DWA*',"Location",'best')
axis([0 60 0 120]);

