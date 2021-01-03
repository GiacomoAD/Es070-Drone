Spill1 = importdata('dronefdp2.txt');
velPitch =  Spill1(:,1);
angPitch =  Spill1(:,2);
t = 0.004: 1/250 : length(velPitch)/250
plot(t,velPitch)
plot(t,angPitch)