% 对已有负荷数据进行预处理
function result=datapretreat()

mpc=data('loaddata.xlsx','Sheet1');
a=[];b=[];a1=[];b1=[];
xavg=zeros(1,24);
xsum=zeros(1,24);
for i=1:size(mpc.load1,1)
    for k=1:24
        if mpc.time(i,1)==(k-1)
            xsum(k)=mpc.load1(i,1)+xsum(k);
        end
    end
    xavg=xsum/4;
end

for i=1:size(mpc.load1,1)
    for k=1:24
        if mpc.time(i,1)==(k-1)  %修正不良数据 
            if mpc.load1(i,1)>1.2*xavg(k) 
                mpc.load1(i,1)=1.2*xavg(k);
            end
            if mpc.load1(i,1)<0.8*xavg(k) 
                mpc.load1(i,1)=0.8*xavg(k);
            end 
            b(end+1)=mpc.load1(i,1)-xavg(k); %零均值化处理
        end
    end
    a1(end+1)=i;   %第几个小时
    b1(end+1)=mpc.load1(i,1);  %对应负荷    
    
end
% plot(a1,b),grid on;
% plot(a1,b1),grid on;
% xlabel('time'),ylabel('Load');
% title('5.2--5.5: Load curve');

n=length(mpc.load1)/24;
timeload=zeros(24,n);
j=0;
for k=1:24
    for i=1:size(mpc.load1,1)
        if mpc.time(i)==(k-1)
            j=j+1;
            timeload(k,j)=mpc.load1(i);
            if j==4
                j=0;
            end
        end
    end
end

result.a1=a1;
result.b1=b1;
result.timeload=timeload;
