path_search

ctrlpoints=[xd'-xd(1);yd'-yd(1)];
% n+1=10
n = size(ctrlpoints,2)-1;
pp = 3;
m = 1+pp+n;

stoptime = 100;
t1 = zeros([1,pp+1]);
t3 = stoptime*ones([1,pp+1]);
t2 = linspace(0,stoptime,m+1-2*pp-2);

knots=[t1, t2, t3];
% m+1=14    m=p+n+1 p =3  up ~ un-p

p = spmak(knots,ctrlpoints); % 生成B样条函数


t = linspace(knots(pp+1),knots(m-pp+1),1000);
dp = fnder(p);
ddp = fnder(dp);
dddp = fnder(ddp);

Xd = double(fnval(p, t));
Vd = double(fnval(dp, t));
Ad = double(fnval(ddp, t));
Jd = fnval(dddp, t);

% 
xt = Xd(1,:)-Xd(1,1);
yt = Xd(2,:)-Xd(2,1);
dxt = Vd(1,:);
dyt = Vd(2,:);
ddxt = Ad(1,:);
ddyt = Ad(2,:);
dddxt = Jd(1,:);
dddyt = Jd(2,:);

DD = [xt,yt,dxt,dyt,ddxt,ddyt];
% Zd = zeros(1,size(Xd,2));
save("desire.mat","DD");
% plot(xt,yt,LineWidth=3,Color='r')

xts = timeseries(xt,t);
yts = timeseries(yt,t);
dxts = timeseries(dxt,t);
dyts = timeseries(dyt,t);
ddxts = timeseries(ddxt,t);
ddyts = timeseries(ddyt,t);
dddxts = timeseries(dddxt,t);
dddyts = timeseries(dddyt,t);
stoptime = knots(m-pp+1);
sample_tt = stoptime/1000;
