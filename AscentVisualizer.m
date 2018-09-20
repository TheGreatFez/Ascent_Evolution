X = linspace(0,70000,1000)';
switch_alt = 100;
end_alt    = 70000;
alt_diff = end_alt - switch_alt;
for iX =1:length(X)
    
    Y(iX,1) = max(0,min(90,90*sqrt((X(iX) - switch_alt)/alt_diff)));    
end

plot(X,Y)
grid on
xlabel('Altitude (m)')
ylabel('Pitch Above Horizon (deg)')
title('Pitch vs Altitude')