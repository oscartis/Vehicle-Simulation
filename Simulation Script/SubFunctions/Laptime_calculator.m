function[Laptime] = Laptime_calculator(cP_onTrack,t,d,i,sampleTime,cPi,trackPath)
k=0;
temp=0;
while temp <=0
temp = d(i-k);
k=k+1;
end

x1 = cP_onTrack(i,1);
x2 = cP_onTrack(i-k,1);
t1 = t;
t2 = t-k*sampleTime;

X =[x1,x2];
T =[t1,t2];

Laptime = interp1(X,T,trackPath(cPi));
1;
