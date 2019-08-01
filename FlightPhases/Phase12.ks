@LAZYGLOBAL OFF.
clearscreen.
for n in allnodes { remove n.}
clearvecdraws().
KUniverse:quicksaveto("Kerpollo 2 Phase 12 - Returning To Kerbin").

print "Exiting Minmus SOI".

warpto(time:seconds + ship:orbit:nextpatcheta).
wait 0.5.
wait until warp = 0 AND KUNIVERSE:timewarp:issettled.
wait 3.
clearscreen.
print "Returning to Kerbin".

warpto(time:seconds + eta:periapsis).
wait 0.5.
wait until warp = 0 AND KUNIVERSE:timewarp:issettled.

lock steering to srfretrograde.
stage.
wait until stage:ready.
stage.
wait until altitude < 10000.
stage.
