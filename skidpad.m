%skidpad track width simulation
%assume the car is turning right
%basic convergence, assume a velocity of 10 m/s and iterate%
iter = 10; % number of iteration for velocity covnergence
v = zeros(1,iter); %velocity vector
v(1) = 10; %first guess of 10 m/s


cl = 3.5;
hs = 0.27033;
m = 294;
g = 9.81;
rho = 1.225;
s = 1.1;
%cdh = range of cg heights (added in later)
twf = 1:0.01:1.5;
twr = 1:0.01:1.5;
r = zeros(length(twf),length(twr));
V = zeros(length(twf),length(twr));



 for j = 1:length(twf)
   for k = 1:length(twr)
       for i = 1:length(v)
    

%calculating radius that the car will follow around the skidpad%
  if twf(j) > twr(k) %assume the car takes the smallest possible radius ie hugging the cones%
     r(j,k) = 7.625 + twf(j)/2; %7.625 is the radius of the cones from the centre of the circle%
    elseif twr(k) > twf(j)
      r(j,k) = 7.625 + twr(k)/2;
     elseif twr(j) == twf(k)
       r(j,k) = 7.625 + twf(j)/2;
  end

    skidpad_distance(j,k) = 2*pi*r(j,k); %distance of the skidpad
    ay = v(i)^2/(r(j,k)*g);

    %perpendicular distance funtion goes here when varying cg height comes

   load_transfer_front = (Ws*ay/twf(j))*(hs*((Kphif + Ws*hs*(1-weightdist))/(Kphif+Kphir-Ws*hs)) + (1-weightdist)*Zrcf) + (Wuf*UScg*ay)/twf(j);
   load_transfer_rear  = (Ws*ay/twr(k))*(hs*((Kphir + Ws*hs*weightdist)/(Kphif+Kphir-Ws*hs)) + weightdist*Zrcr) + (Wur*UScg*ay)/twr(k);

   Fz_fstatic = (m*g*weightdist)/2; %static normal loads
   Fz_rstatic = (m*g*(1-weightdist))/2;

   df(i) = 0.5*rho*s*cl*v(i)^2; %calculate down force

   Fz_fl = Fz_fstatic + load_transfer_front + df(i)/4; %loads (ask for aero balance)
   Fz_fr = Fz_fstatic - load_transfer_front + df(i)/4;
   Fz_rl = Fz_rstatic + load_transfer_rear + df(i)/4;
   Fz_rr = Fz_rstatic - load_transfer_rear + df(i)/4;

   Fy_fl(i) = (1.95 +0.0001*(675 - Fz_fl))*Fz_fl;
   Fy_fr(i) = (1.95 +0.0001*(675 - Fz_fr))*Fz_fr;
   Fy_rl(i) = (1.95 +0.0001*(675 - Fz_rl))*Fz_rl;
   Fy_rr(i) = (1.95 +0.0001*(675 - Fz_rr))*Fz_rr;
   
   v(i+1) = sqrt((r(j,k)*(Fy_fl(i) + Fy_fr(i) + Fy_rl(i) + Fy_rr(i)))/m);
   
   
   if i == 10
       V(j,k) = v(end); %place converged velcoity in an array
       v = zeros(1,iter); %reset velocity vector for next iteration
       v(1) = 10; %initial velocity of 10 m/s
   end
      
        end
    end
end
skidpad_time = skidpad_distance./V;

contourf(twf,twf,skidpad_time)
colorbar
xlabel('front track width')
ylabel('read track width')
title('skidpad time')
