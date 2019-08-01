@LAZYGLOBAL OFF.
//Phase 4: Capture and circularize at Minmus at an altitude of 30km and 0 inclination
KUniverse:quicksaveto("Kerpollo 2 Phase 4 - Post-Capture at Minmus").
SAS ON.
clearscreen.
print "Stabilizing Craft in preparation for docking maneuver".
wait 10.
stage.
set target to "Saturn V + LEM [Kerpollo 2.0] Lander".
set target to target:partstagged("LanderDockingPort")[0].
run dock_to_target.
clearscreen.
print "Docking Complete".
wait 3.
clearscreen.
print "Decoupling Lander/Lunar Module Assembly".
AG9 ON.
wait 1.
RCS OFF.
SAS OFF.
lock steering to LOOKDIRUP(ship:body:position,facing:topvector).
clearscreen.
clearvecdraws().
for node in allnodes { remove node.}
local Lander_Capsule to ship:partstitled("Munar Excursion Module (M.E.M.)")[0].
Lander_Capsule:CONTROLFROM.
