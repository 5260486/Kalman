%% 卡尔曼滤波
mpc=datapretreat;
data=mpc.timeload;
s=size(data);
P=[];
%% 参数设置
N =s(2) ;     % 设置数据长度为N
t = (1:N);   % 生成时间轴
a = 1;       % 状态转移方程，为一维数据时，a的值是1
b = 0;       % 控制输入
c = 1;       % 当sigma2_是一维数据时，c: 观测方程=1
x = 5;       % 设置初值 初始的均值和方差 
for i=1:s(1)
    A=data(i,:);
    B= cov(A); % 初始系统协方差
    P(end+1)=B;
end
V = 50;      % 设置生成的信号的噪声标准差
Q = 1;       % 设置状态转移方差Q和观测方差R
R = 0.5;     % 对象测量噪声的协方差  


%% 初始化
real_signal = zeros(1,N); % 真实信号
z = zeros(1,N);           % 观测数据
x_filter = zeros(1,N);    % 存储卡尔曼估计的结果，缓存起来用于画图形
K = zeros(1,N);           % 存储卡尔曼预测中的增益k，缓存起来用于画图形
y=[];

% 初始化真实数据和观测数据
 for j=1:24
    for i=1:N
       %真实数据
       real_signal(i) = mpc.timeload(j,i); 
       %观测数据，基本负荷+前一天同时刻负荷+该时刻气温+误差
       z(i) = real_signal(i) + normrnd(0, V);
    end

    %% 卡尔曼滤波
    for i=1:N
        % 预测步
        x_ = a*x + b;            %预测当前状态
        P_(j) = a*P(j)*a'+Q;

        % 更新步
        k = P_(j)*c'/(c*P_(j)*c'+R);
        x = x_ + k*(z(i) - c*x_);
        P(j) = (1-k*c)*P_(j); %更新协方差

        % 存储滤波结果
        x_filter(i) = x;
        K(i) = k;
        
        if i==N
            y(end+1)=x; %估计值
        end
    end
 end

y1=[];
for i=(24*(N-1)+1):24*N
    y1(end+1)=mpc.b1(i);%真实值
end
date=0:23;
plot(date,y,date,y1);grid on;
legend('kalman','real');

%% 展示
% %画出卡尔曼增益k 可以看到系统很快会达到稳态，k会很快收敛成为一个常数，此时卡尔曼滤波也就相当于低通滤波了
% plot(t, K);legend('K');
% % 画出波形， 真实值 - 观测值 - 卡尔曼估计值
% figure(2)
% plot(t, real_signal, 'r', t, z, 'g', t, x_filter, 'b')
% legend('real','measure','filter');
