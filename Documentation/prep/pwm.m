function v = pwm(t,T,d)
%figure(1);
v = t;
for i = 1:length(t)
pos = mod(t(i),T);
if(pos >= d) 
    v(i) = 0;
else
    v(i) = 1;
end
end
plot(t,v)
%ylabel('val')
%xlabel('t')
end
