@LAZYGLOBAL OFF.
// Phase 8: Rendezvous with Command Module
KUniverse:quicksaveto("Kerpollo 2 Phase 8 - Lander Ascent Complete").
set target to VESSEL("Saturn V + LEM [Kerpollo 2.0]").
clearscreen.
run rdz_vessel.
lock steering to target:position.

wait until KUNIVERSE:CANQUICKSAVE.
