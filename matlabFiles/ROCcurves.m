%% Hall set

%% PSM K = 1

TN = 17491;

FP1=24/TN;TPR1=0;
FP5=751/TN;TPR5=0.100872;
FP10=1868/TN;TPR10=0.148855;
FP20=1895/TN;TPR20=0.283533;
FP30=1907/TN;TPR30=0.292257;
FP50=1911/TN;TPR50=0.303708;
FP100=1911/TN;TPR100=0.303708;
FP1000=1911/TN;TPR1000=0.303708;

x1 = [0 TPR1 TPR5 TPR10 TPR20 TPR30 TPR50 TPR100 TPR1000 TPR1000 TPR1000 TPR1000 TPR1000 TPR1000];
y1 = [0 FP1 FP5 FP10 FP20 FP30 FP50 FP100 FP1000 FP1000 FP1000 FP1000 FP1000 FP1000];

plot(y1,x1)

%% PSM K = 2

TN = 16682;

FP1=34/TN;TPR1=0.00163577;
FP5=1665/TN;TPR5=0.356052;
FP10=2033/TN;TPR10=0.44711;
FP20=2199/TN;TPR20=0.492366;
FP30=2199/TN;TPR30=0.495093;
FP50=2199/TN;TPR50=0.495093;
FP100=2199/TN;TPR100=0.495093;
FP1000=2199/TN;TPR1000=0.495093;

x2 = [0 TPR1 TPR5 TPR10 TPR20 TPR30 TPR50 TPR100 TPR1000 TPR1000 TPR1000 TPR1000];
y2 = [0 FP1 FP5 FP10 FP20 FP30 FP50 FP100 FP1000 FP1000 FP1000 FP1000];

plot(y2,x2)

%% PSM K = 3
TN = 16994;

FP1=22/TN;TPR1=0.00109051;
FP5=1296/TN;TPR5=0.344057;
FP10=1692/TN;TPR10=0.460742;
FP20=2150/TN;TPR20=0.494547;
FP30=2152/TN;TPR30=0.496728;
FP40=2152/TN;TPR40=0.498364;
FP100=2152/TN;TPR100=0.498909;
FP1000=2152/TN;TPR1000=0.498909;
FP10000=2152/TN;TPR10000=0.498909;

y3 = [0 FP1 FP5 FP10 FP20 FP30 FP40 FP100 FP1000 FP10000 FP10000 FP10000 FP10000];
x3 = [0 TPR1 TPR5 TPR10 TPR20 TPR30 TPR40 TPR100 TPR1000 TPR10000 TPR10000 TPR10000 TPR10000];

plot(y3,x3)


%%
plot(y1,x1,y2,x2,y3,x3,[0 0.2],[0 0.2],'k-','LineWidth',3)
ylabel('True Positive Rate(Sensitivity)');
xlabel('False Positive Rate(Specificity)');
legend('K = 1', 'K = 2', 'K = 3', 'Diagonal');
title('PSM - Hall dataset');
grid on
%% Feature grouping K = 1, alpha: 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 
TN = 16119;


TN = 16124;
FP1=1138/TN; TPR1=0.00780437;
FP2=2917/TN;TPR2=0.309053;
FP3=4071/TN;TPR3=0.369927;
FP4=5922/TN;TPR4=0.582726;
FP5=6005/TN;TPR5=0.582726;
FP6=6024/TN;TPR6=0.584287;
FP7=6061/TN;TPR7=0.584287;
FP8=6293/TN;TPR8=0.584807;
FP9=6344/TN;TPR9=0.584807;
FP10=6344/TN;TPR10=0.584807;
FP11=6360/TN;TPR11=0.584807;
FP12=6360/TN;TPR12=0.584807;
FP13=6360/TN;TPR13=0.584807;
FP14=6360/TN;TPR14=0.584807;
FP15=6360/TN;TPR15=0.584807;

x3 = [0 TPR1 TPR2 TPR3 TPR4 TPR5 TPR6 TPR7 TPR8 TPR9 TPR10 TPR11 TPR12 TPR13 TPR14 TPR15];
y3 = [0 FP1 FP2 FP3 FP4 FP5 FP6 FP7 FP8 FP9 FP10 FP11 FP12 FP13 FP14 FP15];

plot(y1,x1)
%% Feature grouping K = 2, alpha: 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 
TN = 15157;
FP1=2192/TN;TPR1=0.0599054;
FP2=4273/TN;TPR2=0.534945;
FP3=6189/TN;TPR3=0.593799;
FP4=6251/TN;TPR4=0.595376;
FP5=6256/TN;TPR5=0.595376;
FP6=6258/TN;TPR6=0.595901;
FP7=6267/TN;TPR7=0.595901;
FP8=6269/TN;TPR8=0.595901;
FP9=6275/TN;TPR9=0.595901;
FP10=6275/TN;TPR10=0.595901;
FP11=6275/TN;TPR11=0.595901;
FP12=6279/TN;TPR12=0.595901;
FP13=6279/TN;TPR13=0.595901;
FP14=6279/TN;TPR14=0.595901;
FP15=6279/TN;TPR15=0.595901;

x2 = [0 TPR1 TPR2 TPR3 TPR4 TPR5 TPR6 TPR7 TPR8 TPR9 TPR10 TPR11 TPR12 TPR13 TPR14 TPR15];
y2 = [0 FP1 FP2 FP3 FP4 FP5 FP6 FP7 FP8 FP9 FP10 FP11 FP12 FP13 FP14 FP15];

plot(y1,x1,y2,x2)

%% Feature grouping K = 3, alpha: 1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 
TN = 14460;

FP1=4302/TN;TPR1=0.47609;
FP1_2=4330/TN;TPR1_2=0.478192;
FP1_5=5182/TN;TPR1_5=0.539674;
FP2=6927/TN;TPR2=0.591172;
FP3=7125/TN;TPR3=0.595376;
FP4=7196/TN;TPR4=0.595901;
FP5=7201/TN;TPR5=0.595901;
FP6=7205/TN;TPR6=0.596427;
FP7=7238/TN;TPR7=0.596427;
FP8=7244/TN;TPR8=0.596952;
FP9=7244/TN;TPR9=0.596952;
FP10=7244/TN;TPR10=0.596952;
FP11=7244/TN;TPR11=0.596952;
FP12=7244/TN;TPR12=0.596952;
FP13=7244/TN;TPR13=0.596952;
FP14=7244/TN;TPR14=0.596952;
FP15=7244/TN;TPR15=0.596952;

x3 = [0 TPR1 TPR1_2 TPR1_5  TPR2 TPR3 TPR4 TPR5 TPR6 TPR7 TPR8 TPR9 TPR10 TPR11 TPR12 TPR13 TPR14 TPR15];
y3 = [0 FP1 FP1_2 FP1_5 FP2 FP3 FP4 FP5 FP6 FP7 FP8 FP9 FP10 FP11 FP12 FP13 FP14 FP15];
%%
plot(y1,x1,y2,x2,y3,x3,[0 0.59],[0 0.59],'k-','LineWidth',3)
ylabel('True Positive Rate(Sensitivity)');
xlabel('False Positive Rate(Specificity)');
legend('K = 1', 'K = 2', 'K = 3', 'Diagonal');
title('Feature grouping - Hall dataset');
grid on
%% Img difference K = 1, alpha:  1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 t = 5.53
TN = 14409;


