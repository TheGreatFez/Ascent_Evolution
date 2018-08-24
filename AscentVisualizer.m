X = linspace(0,70000,1000)';
switch_alt = 5000;
end_alt    = 40000;
for iX =1:length(X)
    
    Y(iX,1) = max(0,min(90,90 - (90/(end_alt - switch_alt))*(X(iX) - switch_alt)));    
end

plot(X,Y)
grid on
xlabel('Altitude (m)')
ylabel('Pitch Above Horizon (deg)')
title('Pitch vs Altitude')