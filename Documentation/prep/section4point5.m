close all;clear all
global v_in R C L T d
v_in=1.5;
R=680;
C=200e-6;
L=180e-6;
T=1e-4;
d=0.1;
options=odeset('MaxStep', 1e-5);
[t,x]= ode23(@boost,[0:1e-7:0.5], [0 0], options);
figure,plot(t,x) %%Plot the data
%figure,plot(t,x,t,pwm(t,T,d)) %%Plot the data

xlabel('Time [s]');
ylabel('Voltage [V] , Current [A]');
legend('Current','Voltage','PWM');
ylim([0 inf]);
title(['Ode23 Simulation d = ',num2str(d)]);

function dx=boost(t,x)
    global v_in R C L T d
    dx=zeros(2,1);
    if pwm(t,T,d)==1
    dx(1) = v_in/L;
    dx(2) = -x(2)/(R*C);
    end

    if pwm(t,T,d)==0 && x(1)>=0
    dx(1) = -x(2)/L+v_in/L;
    dx(2) = x(1)/C-x(2)/(C*R);
    end
    if pwm(t,T,d)==0 && x(1) <0
    dx(1) = 0;
    dx(2) = -x(2)/(C*R);
    end
end

function v = pwm(t,T,d)
for i = 1:length(t)
pos = mod(t(i),T);
    if(pos >= d*T) 
        v(i) = 0;
    else
        v(i) = 1;
    end
end
end