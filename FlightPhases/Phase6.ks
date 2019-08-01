@LAZYGLOBAL OFF.
// Phase 6: Landing
KUniverse:quicksaveto("Kerpollo 2 Phase 6 - Landing").
clearscreen.
SAS OFF.
RCS OFF.
if availablethrust = 0 {
  stage.
}

local target_latlng to LATLNG(12.69,-47.14).

run landing_orbit_new(target_latlng,3000).

run spot_land_eq(target_latlng).

clearscreen.
clearvecdraws().
print "Landing Complete".
LIGHTS ON.
