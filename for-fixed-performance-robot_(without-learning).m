%% parameter Terminology
% cf - fatigue constant
% cr - recovery constant
% f0 - applied force for a particular task
% mvc - initial maximum isometric force one can generate at rest
% hp - dynamic model for the human physical performance
% hc - dynamic model for the human cognitive performance
% fth - an equilibrium point at which the fatigue and recovery balance
% out.It is called the lowest threshold point 
% e - robot exploration rate
% R - robot performance matrix
% po - probability of assigned task
% rp - sum of robot performance considring the object of arrival
% a,b - physical performance parameter constant
% t - time bound
% fmax_iso - maximum isometric force one can produce
% f - chnaging force co-efficient
% pc_max - max cognitive performance parameter
% pc_min - min cognitive performance parameter
% r - utilization factor
% B - task difficulty [0.3 to 1]
% pr - robot performance parameter
% fs - faith control parameter 
% fth - threshold for the faith
% dp - performance difference between human and robot
% T - trust
% Td - upper limit of trust dependability
% Tud - upper limit of trust underdependibility
% h_all - overall human performance
% a - human physical performance co-efficient
% 1-a - human cognitive performance co-efficient
% n_r - number of robots
% eta - human assigned task
% cc - cognitive co-efficient



clc
clear all
close all

%% Human Physical Performance

% parameter initialization
cf=10^-4;
cr=2.4*10^-4;
f0=50;
mvc=200;
hp_init=1; % initially human physical performance will be high
fth=151.9;
n=0:0.5:9;
t=length(n);
fmax_iso_init=1; % initially human physical performance will be high
f=10^5.4;
n_r=3; % initialize the number of robots
eta=0.2;
cc=0.8;

% non-Linearized Physical Performance Calculation
fmax_iso(1)=mvc;
for i=1:t
    hp(i)=(fmax_iso(i)-fth)/(mvc-fth);
    fmax_iso(i+1)=fmax_iso(i)-(n_r*cf*fmax_iso(i)*f*exp(-i)/mvc)+(cr*(mvc-fmax_iso(i)));
%     fmax_iso(i+1)=fmax_iso(i)-(n_r*cf*fmax_iso(i)*f*exp(-(n_r/1.5)*i)/mvc)+(cr*(mvc-fmax_iso(i)));
   
end


%% Human Cognitive Performance for Workload

% parameter initialization
pc_max=1;
pc_min=0;
C=0.3; % C = 0.7, 0.3
rp1=0.7, rp2=0.5, rp3=0.3;
po1=0.5, po2=0.3, po3=1-(po1+po2); % sum should be 1
R=[rp1, rp2, rp3];
PO=[po1, po2, po3];
rp=sum(R.*PO);
e=1-rp;
r_init=0;

% non-Linearized Cognitive performance Performance Parameter
r(1)=r_init;
for i=1:t
    hc(i)=1-(pc_max-pc_min)*((r(i)/(1-C))^(1-C))*(((1-r(i))/C)^C)+pc_min;
    r(i+1)=(r(i)+(cc*e+eta-2*r(i))/(2*t));
    if (e-r(i))<=0 % Productive cognitive experience
       r(i)=0;
    end
end
hc((i+1):t)=zeros(1,length((i+1):t));
 
%% Robot performance Parameter Effect
pr=rp*ones(1,t);
pr1=(rp1*po1/rp)*ones(1,t);
pr2=(rp2*po2/rp)*ones(1,t);
pr3=(rp3*po3/rp)*ones(1,t);


%% human Overall Performance parameter
h=0.1*hp+0.7*hc+0.2*pr;

%% Check for h<=0.2 index
j=min(find(h<0.2)) % point for which h is below 0.2

if j>0
   t=j; 
end

disp('t'), t
disp('hc'), hc(t)
disp('h'), h(t)
disp('r'), r(t)

%% Overall Performnace Calculation
% Initialization for faith calculation
h_all=0.3*hp(1:t)+0.7*h(1:t)
fp=0.2*h_all;
fd=0.7*h_all;

for k=2:t
    if pr(k-1)<fp(k-1)
        p=0.6;
    elseif pr(k-1)>=fp(k-1) && pr(k-1)<fd(k-1)
        p=0.4;
    elseif pr(k-1)>=fd(k-1) && pr(k-1)<0.9
        p=0.2;
    else
        p=0.1;
    end
    h_all(k)=p*hp(k)+(1-p)*h(k);
    fp(k)=0.2*h_all(k); % for dynamic changing of threshold
    fd(k)=0.7*h_all(k); % for dynamic changing of threshold
end


 
%% Trust Calculation from the robot performance
eps=0.2;

T=zeros(1,length(h_all));
T1=zeros(1,length(h_all));
T2=zeros(1,length(h_all));
T3=zeros(1,length(h_all));

% for mul-robot
for m=1:length(h_all)
    if pr(m)<fp(m) 
        T(m)=0;
    elseif pr(m)>=fp(m) && pr(m)<fd(m)
        T(m)=eps;
    elseif pr(m)>=fd(m) 
        if pr(m)-fd(m) >0
            ang=3*(pr(m)-fd(m)); % 2.5
        end
        T(m)=min(1,eps+tanh(ang));
    end
end

% for robot 1
for m=1:length(h_all)
    if pr1(m)<fp(m) 
        T1(m)=0;
    elseif pr1(m)>=fp(m) && pr1(m)<fd(m)
        T1(m)=eps;
    elseif pr1(m)>=fd(m) 
        if pr1(m)-fd(m) >0
            ang=3*(pr1(m)-fd(m)); % 2.5
        end
        T1(m)=min(1,eps+tanh(ang));
    end
end

% for robot 2
for m=1:length(h_all)
    if pr2(m)<fp(m) 
        T2(m)=0;
    elseif pr2(m)>=fp(m) && pr2(m)<fd(m)
        T2(m)=eps;
    elseif pr2(m)>=fd(m) 
        if pr2(m)-fd(m) >0
            ang=3*(pr2(m)-fd(m)); % 2.5
        end
        T2(m)=min(1,eps+tanh(ang));
    end
end

% for robot 3
for m=1:length(h_all)
    if pr3(m)<fp(m) 
        T3(m)=0;
    elseif pr3(m)>=fp(m) && pr3(m)<fd(m)
        T3(m)=eps;
    elseif pr3(m)>=fd(m) 
        if pr3(m)-fd(m) >0
            ang=3*(pr3(m)-fd(m)); % 2.5
        end
        T3(m)=min(1,eps+tanh(ang));
    end
end

% Human Performance
figure(11)
grid on
% All plotting under the same curve
hold on
% plot(n(1:t),h(1:t),'-yo','Linewidth',25,'MarkerSize',15)
plot(n(1:t),h_all(1:t),'-k','Linewidth',25)
% plot(n(1:t),hp(1:t),'-co','Linewidth',25,'MarkerSize',15)
hold off
% legend('Physical Performance','Cognitive Performance', 'Performance')

set (gca,'fontsize',60)
xlabel('Time (Hour)','fontsize',60,'fontweight','b','color','k')
ylabel('Human Performance','fontsize',60,'fontweight','b','color','k')
xlim([0 8])
ylim([0 1.01])


% Workload
figure(13)
rppo=(1-rp1)*po1+(1-rp2)*po2+(1-rp3)*po3;
grid on
% All plotting under the same curve
hold on
plot(n(1:t),(1-rp1)*po1.*(1-hc(1:t))/rppo,'-g','Linewidth',45)
plot(n(1:t),(1-rp2)*po2.*(1-hc(1:t))/rppo,'-r','Linewidth',25)
plot(n(1:t),(1-rp3)*po3.*(1-hc(1:t))/rppo,'-b','Linewidth',25)
plot(n(1:t),1-hc(1:t),'-k','Linewidth',25)
hold off
% legend({'for Robot 1','for Robot 2', 'for Robot 3', 'for Multi-Robot'},'FontSize',14)
set (gca,'fontsize',60)
xlabel('Time (Hour)','fontsize',60,'fontweight','b','color','k')
ylabel('Human Workload','fontsize',60,'fontweight','b','color','k')
xlim([0 8])
ylim([0 1.01])

% Trust
figure(15)
grid on
% All plotting under the same curve
hold on

plot(n(1:t),T1(1:t),'-g','Linewidth',25)
plot(n(1:t),T2(1:t),'-r','Linewidth',25)
plot(n(1:t),T3(1:t),'-b','Linewidth',25)

plot(n(1:t),T(1:t),'-k','Linewidth',25)
hold off
set (gca,'fontsize',60)
% legend({'for Robot 1','for Robot 2', 'for Robot 3', 'for Multi-Robot'},'FontSize',30)
xlabel('Time (Hour)','fontsize',60,'fontweight','b','color','k')
ylabel('Trust','fontsize',60,'fontweight','b','color','k')
xlim([0 8])
ylim([0 1.01])




C
disp('pr='),pr(1)
Pfw=[(1-rp1)*po1.*(1-hc(1:t))/rppo; (1-rp2)*po2.*(1-hc(1:t))/rppo; (1-rp3)*po3.*(1-hc(1:t))/rppo; 1-hc(1:t)];
PfT=[rp1*po1*T(1:t)/rp; rp2*po2*T(1:t)/rp; rp3*po3*T(1:t)/rp; T(1:t)];