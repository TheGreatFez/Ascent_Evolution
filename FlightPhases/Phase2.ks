@LAZYGLOBAL OFF.
//Phase 2: Match orbital plane of Minmus and intercept
KUniverse:quicksaveto("Kerpollo 2 Phase 2 - Launch Complete").
set target to Minmus.
run rdz_body.
clearscreen.
print "Warping to Munar Insertion".
wait 3.
warpto(time:seconds + eta:transition + 10).
wait until ship:body = BODY("Minmus").
