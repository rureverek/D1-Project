close all ; clear all; %close all figures and clear all variables 

Vin=1.5; %Input voltage
L=180e-6;
R=680;
C=200e-6;
Vout=0;
iL=0;
time=0;

Vout_target=5; %Target voltage
counter=1;

timeStep=1/1000e3;
switchTime=1/10e3;
dutyCycle=0.3;


while time<=1
    
    % Is the switch on or off?
    modulous=mod(time,switchTime);
    if(modulous/switchTime >dutyCycle)
        switchState=0;
    else
        switchState=1;
    end
      
    %If the switch is open, Figure 4 or 5 in notes
    if(switchState==0 && iL<0)
     %Case: Switch open and diode reverse biased (Figure 5 in notes)   
     case1=1;   
    else
     %Case: Switch open, diode forward biased  (Figure 4 in notes)           
     case1=2;       
    end
    
    %If the switch is closed, always case C
    if(switchState==1)
        %Case: Switch closed (Figure 3 in notes)          
        case1=3;
    end   
       
    switch case1
        case 1
            %Case: Switch open and diode reverse biased 
            dVout_dt= -Vout/(R*C);
            Vout=Vout+dVout_dt*timeStep; %Update Vout with new value            
            diL_dt=0;

        case 2           
           %Case: Switch open, diode forward biased

            diL_dt= -Vout/L+Vin/L;
            dVout_dt = iL/C-Vout/(C*R);
            iL=iL+diL_dt*timeStep;  %Update iL with new value
            Vout=Vout+dVout_dt*timeStep;  %Update Vout with new value

        case 3
            %Case: switch closed           
            dVout_dt= -Vout/(R*C);
            Vout=Vout+dVout_dt*timeStep;
            diL_dt = Vin/L;
            iL=iL+diL_dt*timeStep;
    end
    
    
    %Change duty cycle based on error
     if(mod(time,0.01)<timeStep)   %Change every 0.01 seconds      
         error=Vout-Vout_target;
          kP=0.005;
            dutyCycle=dutyCycle-error*kP;                  
     end
    
         
    %More to next step
    time=time+timeStep;
    
    %Store all variables each loop
    time_store(counter)=time;
    pwm(counter) = switchState;
    Vout_store(counter)=Vout;
    iL_store(counter)=iL;
    dutyCycle_store(counter)=dutyCycle;
    counter=counter+1; %Counter for storing variables
    
end

%Plot data
figure,plot(time_store,Vout_store,'k');
hold all,plot(time_store,iL_store,'c');  
hold all,plot(time_store,pwm,'y');  

xlabel('Time [s]');
ylabel('Voltage [V] , Current [A]');
legend('Current','Voltage','PWM');
%ylim([0 inf]);
xlim([0 1]);
title(['Dynamic Simulation d = ',num2str(dutyCycle),', V target = ', num2str(Vout_target)]);

 