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
tw = 1:0.001:1.5;
corner_radius = 3:0.02:50;
corner_radius = corner_radius(1:length(tw)); %janky fuckery to be able to plot need changed
r = zeros(length(tw),length(corner_radius));
distance = zeros(length(tw),length(corner_radius)); % array for storing lengths


 for j = 1:length(tw)
   for k = 1:length(corner_radius)
       for i = 1:length(v)
    

%calculating radius that the car will follow around the skidpad%
  r(j,k) = corner_radius(k) + tw(j)/2;

    skidpad_distance(j,k) = 2*pi*r(j,k); %distance of the skidpad
    ay = v(i)^2/(r(j,k)*g);

    %perpendicular distance funtion goes here when varying cg height comes
   %calculating load transfers
   load_transfer_front = (Ws*ay/tw(j))*(hs*((Kphif + Ws*hs*(1-weightdist))/(Kphif+Kphir-Ws*hs)) + (1-weightdist)*Zrcf) + (Wuf*UScg*ay)/tw(j);
   load_transfer_rear  = (Ws*ay/tw(j))*(hs*((Kphir + Ws*hs*weightdist)/(Kphif+Kphir-Ws*hs)) + weightdist*Zrcr) + (Wur*UScg*ay)/tw(j);

   Fz_fstatic = (m*g*weightdist)/2; %static normal loads
   Fz_rstatic = (m*g*(1-weightdist))/2;

   df = 0.5*rho*s*cl*v(i)^2; %calculate down force

   Fz_fl = Fz_fstatic + load_transfer_front + df/4; %loads (ask for aero balance)
   Fz_fr = Fz_fstatic - load_transfer_front + df/4;
   Fz_rl = Fz_rstatic + load_transfer_rear + df/4;
   Fz_rr = Fz_rstatic - load_transfer_rear + df/4;

   Fy_fl = (1.95 +0.0001*(675 - Fz_fl))*Fz_fl; %calculting tyre forces
   Fy_fr = (1.95 +0.0001*(675 - Fz_fr))*Fz_fr;
   Fy_rl = (1.95 +0.0001*(675 - Fz_rl))*Fz_rl;
   Fy_rr = (1.95 +0.0001*(675 - Fz_rr))*Fz_rr;
   
   v(i+1) = sqrt((r(j,k)*(Fy_fl + Fy_fr + Fy_rl + Fy_rr))/m); % calculating velocity
   %tyre forces = centripetal acc
   
   if i == iter
       V(j,k) = v(end); %place converged velcoity in an array
       v = zeros(1,iter); %reset velocity vector for next iteration
       v(1) = 10; %initial velocity of 10 m/s
   end
      
        end
    end
 end
 
skidpad_time = skidpad_distance./V;


contourf(corner_radius,tw,skidpad_time)
colorbar
xlabel('corner radius')
ylabel('track width')
title('skidpad time')