Spill1 = importdata('Testemaior.txt');
velPitch =  Spill1(:,1);
angPitch =  Spill1(:,2);
t = (0.0004: 1/2500 : length(velPitch)/2500)';
figure(1)
plot(t,velPitch)
figure(2)
plot(t,angPitch)

t_zero = t(17200: 20100);
ang_zero = angPitch(17200: 20100);
plot(t_zero, ang_zero)
p = polyfit(t,vq2,7);
vel_est = polyval(p, t);






