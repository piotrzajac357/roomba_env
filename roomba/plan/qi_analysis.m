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

%% import data - plan_showcase_center
stc_sc_c = readtable("C:\Users\zajac\Desktop\pm\roomba_env\roomba\tests_results\log_stc_showcase_center.txt", opts);
ba_sc_c  = readtable("C:\Users\zajac\Desktop\pm\roomba_env\roomba\tests_results\log_ba_showcase_center.txt", opts);
rg_sc_c  = readtable("C:\Users\zajac\Desktop\pm\roomba_env\roomba\tests_results\log_rg_showcase_center.txt", opts);
dwa_sc_c = readtable("C:\Users\zajac\Desktop\pm\roomba_env\roomba\tests_results\log_dwa_showcase_center.txt", opts);

%% plot results
figure(1);
subplot(221)
plot(stc_sc_c.t,stc_sc_c.Q_E);
hold on; grid on
plot(ba_sc_c.t,ba_sc_c.Q_E);
plot(rg_sc_c.t,rg_sc_c.Q_E);
plot(dwa_sc_c.t,dwa_sc_c.Q_E);
title('Zużyta energia na planie 1');
xlabel('t[s]')
ylabel('Q_E[%]')
legend('stc','ba','rg','dwa*')

subplot(222)
plot(stc_sc_c.t,stc_sc_c.Q_D);
hold on; grid on
plot(ba_sc_c.t,ba_sc_c.Q_D);
plot(rg_sc_c.t,rg_sc_c.Q_D);
plot(dwa_sc_c.t,dwa_sc_c.Q_D);
title('Przebyta droga na planie 1');
xlabel('t[s]')
ylabel('Q_D[%]')
legend('stc','ba','rg','dwa*')


subplot(223)
plot(stc_sc_c.t,stc_sc_c.Q_cov);
hold on; grid on
plot(ba_sc_c.t,ba_sc_c.Q_cov);
plot(rg_sc_c.t,rg_sc_c.Q_cov);
plot(dwa_sc_c.t,dwa_sc_c.Q_cov);

subplot(224)
plot(stc_sc_c.t,stc_sc_c.Q_O);
hold on; grid on
plot(ba_sc_c.t,ba_sc_c.Q_O);
plot(rg_sc_c.t,rg_sc_c.Q_O);
plot(dwa_sc_c.t,dwa_sc_c.Q_O);

