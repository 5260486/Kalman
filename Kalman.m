%% 卡尔曼滤波

%%获得数据
mpc=datapretreat;
data=mpc.timeload;
s=size(data);

%% 参数设置
N =s(2) ;     % 设置数据长度为N
t = (1:N);   % 生成时间轴
fai = 1;     % 状态转移方程，本例中是连续多天同一时刻的温度，可看作为缓变状态，可以认为是1
b = 0;       % 控制输入
X = 0;       % 设置初值
P=[];
for i=1:s(1)
    A=data(i,:);
    B= cov(A); % 初始系统协方差
    P(end+1)=B; %初始协方差阵
end

V = 50;      % 设置生成的信号的噪声标准差
Q = 1;       % 设置状态转移方差Q和观测方差R
R = 0.05;     % 对象测量噪声的协方差  

%% 初始化
% y = zeros(1,N);           % 观测数据
x_filter = zeros(24,1);    % 存储卡尔曼估计的结果，缓存起来用于画图形
x_filter2 = zeros(24,1); 
K = zeros(24,3);           % 存储卡尔曼预测中的增益k，缓存起来用于画图形

%% 初始化真实数据和观测数据
% Lk=Hnk*Lnk+Hpk*Lpk+Htk*Ltk+vk; %第t天k时刻负荷=该时刻基本负荷Lnk+前一天同时刻负荷Lpk+该时刻气温Ltk+误差vk
Lnksum=zeros(24,1);
Lnk=zeros(24,1);
Ltksum=zeros(24,1);
Ltk=zeros(24,1);
Lk=mpc.timeload(:,2:N);
for j=1:24
    for i=1:N-1
        Lnksum(j)=mpc.timeload(j,i)+Lnksum(j);  
        Ltksum(j)=mpc.tem(j,i)+Ltksum(j);
    end
        Lnk=Lnksum/(N-1);   
        Ltk=Ltksum/(N-1);
end

Hnk=1; %第k时刻负荷值的参数矩阵，可看作不变
bsum=zeros(3,1);

for i=1:N-1
%     L=abs(Lk-Hnk*Lnk);
    L=Lk-Hnk*Lnk;
    L=L(:,i);
    Lpk=mpc.timeload(:,i);
    Lkx=[ones(24,1),Lpk,Ltk];
    [br,bint,r,rint,s]=regress(L,Lkx); %线性回归得到Hpk，Htk
    vk=br(1);
    Hpk=br(2);
    Htk=br(3);
    H=[Hnk,Hpk,Htk]; % 观测矩阵
    Hk(i,:)=H;
    X=[Lnk,Lpk,Ltk];
    y=X*H'+vk;
    if i==1
        xk.x1=X;
    end
    if i==2
        xk.x2=X;
    end
    if i==3 
        xk.x3=X; %得到真实数据
    end  
    Y(:,i)=y;    %得到观测数据

end  

%% 卡尔曼滤波
T=mpc.tem;
u=[30 30 -10 30 30 100 30 -10 -10 0 -100 -100 -100 -100 30 100 100 30 30 30 30 -100 -100 -100 -100]; %试验所得修正系数
for j=1:24  %每个小时
    y=Y(j,:);%单个小时前三天的观测值
    for i=1:N-1  %前三天
        if i==1
            X=xk.x1;
        end
        if i==2
            X=xk.x2;
        end
        if i==3 
            X=xk.x3; %真实数据
        end  
        x=X(j,:);
        H=Hk(i,:);
        
        % 预测步
        x_ = fai*x + b;            %预测当前状态
        P_(j) = fai*P(j)*fai'+Q;

        % 更新步
        k = P_(j)*H'/(H*P_(j)*H'+R);
        x = x_ + k'*(y(i) - x_*H');
        P(j) = (1-H*k)*P_(j); %更新协方差

        % 存储滤波结果
        if i==N-1
            x_filter(j)=x*H';
            x_filter2(j)=x*H'+u(j)*(T(j,i)-Ltk(j));%对当天气温进行修正
            K(j,:) = k;
        end
    end
end

%% 画出预测值与实际值的对比图
y1=[];
for i=(24*(N-1)+1):24*N
    y1(end+1)=mpc.b1(i);%第四天的真实值
end
hour=0:23;

% plot(hour,x_filter,'b-o',hour,y1,'k-+');grid on;
% legend('Kalman1','Real');
% xlabel('t/h');ylabel('Load/W');

% plot(hour,x_filter2,'r-o',hour,y1,'k-+');grid on;
% legend('Kalman2','Real');
% xlabel('t/h');ylabel('Load/W');

% plot(hour,x_filter,'b-*',hour,x_filter2,'r-o',hour,y1,'k-+');grid on;
% legend('Kalman1','Kalman2','Real');
% xlabel('t/h');ylabel('Load/W');

%% 绝对误差计算
d1=[];
d2=[];
for i=1:24
    d1(end+1)=100*abs(x_filter(i)-y1(i))/y1(i);
    d2(end+1)=100*abs(x_filter2(i)-y1(i))/y1(i);
end
% bar(hour,d1);
bar(hour,d2);
xlabel('t/h');ylabel('误差%');
