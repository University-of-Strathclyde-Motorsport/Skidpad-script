%skidpad track width simulation
%assume the car is turning right
%basic convergence, assume a velocity of 10 m/s and iterate%
load USMEV01Rates&Parameters.mat
iter = 100; % number of iteration for velocity covnergence
v = zeros(1,iter); %velocity vector
v(1) = 10; %first guess of 10 m/s

%BASE Cl
cl = 3.5;
hs = 0.27033;
m = 294;
g = 9.81;
rho = 1.225;
s = 1.1;

%cdh = range of cg heights (added in later)
twf = 1:0.01:2;
twr = 1:0.01:2;
twfmm = twf.*1000;
twrmm = twr.*1000;
r = zeros(length(twf),length(twr)); %corner radius array
V = zeros(length(twf),length(twr)); %final velocity array
skidpad_distance = zeros(length(twf),length(twr)); % array for storing lengths


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
			disp(v(i+1));
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


hold on

contourf(twrmm,twfmm,skidpad_time)
colorbar
xlabel('Rear Track Width (m)')
ylabel('Front Track Width (m)')
title('Skidpad Time (s)')

