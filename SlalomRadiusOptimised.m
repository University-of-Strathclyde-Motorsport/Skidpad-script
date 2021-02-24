%Slalom track width optimisation
%Runs a skidpad with the radius given from a slalom of y spacingand track width t

clear all;
load USMEV01Rates&Parameters.mat;

iter = 10; % number of iteration for velocity covnergence
v = zeros(1,iter); %velocity vector
v(1) = 10; %first guess of 10 m/s


BaseCl = 3.5;
TWScalingFactor = 1;
hs = 0.27033;
m = 294;
g = 9.81;
rho = 1.225;
s = 1.1;

%cone spacing
miny = 7.5;
maxy = 12;


tw = 1:0.001:2;
twmm = tw.*1000;
ntw = size(tw,2);

y = miny:(maxy-miny)/ntw:maxy;
%corner_radius = corner_radius(1:length(tw)); %janky fuckery to be able to plot, need changed

distance = zeros(length(tw),length(y)); % array for storing lengths


for j = 1:length(tw)
	cl = BaseCl + (tw(j)-1.2)*TWScalingFactor;
	for k = 1:length(y)
		for i = 1:length(v)


			%calculating radius that the car will follow around the skidpad
			r(j,k) = ((y(k)^2)/(4*tw(j)))+ (tw(j)/4);
			alpha = 2*asin(y(k)/(2*r(j,k)));


			skidpad_distance(j,k) = r(j,k)*alpha; %distance of the skidpad
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

tiledlayout('flow')
colormap('turbo')
nexttile
contourf(y,twmm,V,20)
colorbar
xlabel('Slalom Cone Spacing (m)')
ylabel('Track Width (mm)')
title('SS Slalom Velocity (m/s)')

nexttile
contourf(y,twmm,r,20)
colorbar
xlabel('Slalom Cone Spacing (m)')
ylabel('Track Width (mm)')
title('Slalom Corner Radius (m)')

nexttile
contourf(y,twmm,skidpad_distance,20)
colorbar
xlabel('Slalom Cone Spacing (m)')
ylabel('Track Width (mm)')
title('Distance Travelled over 1 cone (m)')
