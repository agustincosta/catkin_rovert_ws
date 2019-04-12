clear all
close all

data = readtable('PIDlog.txt', 'HeaderLines',1)
data1 = readtable('./Logs/Escalon/escalonRueda1_0_100.txt','HeaderLines',1)
data2 = readtable('./Logs/Escalon/escalonRueda2_0_100.txt','HeaderLines',1)
data3 = readtable('./Logs/Escalon/escalonRueda3_0_100.txt','HeaderLines',1)
data4 = readtable('./Logs/Escalon/escalonRueda4_0_100.txt','HeaderLines',1)


t = [0:.1:0.1*1317];

size(data)

figure(1)

plot(t,data.Var1);
hold on
plot(t,data.Var2)
hold off

Ts = 0.1

mydata = iddata(data2.Var2(1:489),data2.Var1(1:489),Ts);

mydata.InputName = 'Velocidad Angular';
mydata.InputUnit = 'rad/s';
mydata.OutputName = 'Velocidad Angular';
mydata.OutputUnit = 'rad/s';

mydatad = detrend(mydata);

ss = ssest(mydatad,1,'Ts',Ts)



t = [0:.01:0.01*488];

figure (2)
hold on
plot(t,data1.Var2(1:489));
plot(t,-data1.Var3(1:489));
hold off

-data1.Var3(1:488);



%4.89 hay que recortar el tiempo
S = stepinfo(-data1.Var3(1:489),t,3.34)

Ts = 0.01

mydata = iddata(-data1.Var3(1:489),data1.Var2(1:489),Ts);

mydata.InputName = 'Velocidad Angular';
mydata.InputUnit = 'rad/s';
mydata.OutputName = 'Velocidad Angular';
mydata.OutputUnit = 'rad/s';

mydatad = detrend(mydata);

ss1 = ssest(mydatad,1,'Ts',Ts)

figure(3)
hold on
plot(t,data2.Var2(1:489));
plot(t,data2.Var3(1:489));
hold off

data2.Var2(1:488);



%4.89 hay que recortar el tiempo
S = stepinfo(data2.Var3(1:489),t,3.34)

Ts = 0.01

mydata = iddata(data2.Var3(1:489),data2.Var2(1:489),Ts);

mydata.InputName = 'Velocidad Angular';
mydata.InputUnit = 'rad/s';
mydata.OutputName = 'Velocidad Angular';
mydata.OutputUnit = 'rad/s';

mydatad = detrend(mydata);

ss2 = ssest(mydatad,[1,2,3,4],'Ts',Ts)

H = [tf([0.828 -0.01998],[-8.206 0],0.01)]

%sys = ss(H)


step(H)

