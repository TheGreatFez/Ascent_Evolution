set config:ipu to 2000.
run launch.
set Target_body to Minmus.
set target to Target_body.
run rdz_body.
run lib_execnode.
ExecuteNode().
clearvecdraws().
for node in allnodes { remove node.}
SAS ON.
clearscreen.
print "Stabilizing Craft in preparation for docking maneuver".
// Write a better wait for stabilization
wait 10.
stage.
set target to "Saturn V + LEM (Ascent Studies) Lander".
set target to target:partstagged("LanderDockingPort")[0].
run dock_to_target.
clearscreen.
print "Docking Complete".
wait 3.
clearscreen.
print "Decoupling Lander/Lunar Module Assembly".
AG1 ON.
wait 3.
clearscreen.
print "Warping to Munar Insertion".
warpto(time:seconds + eta:transition + 10).
wait until ship:body = Target_body.
// Capture logic
run capture.
run CircularizePE.
