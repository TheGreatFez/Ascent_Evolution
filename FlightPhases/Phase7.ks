@LAZYGLOBAL OFF.
// Phase 7: Lander Ascent
KUniverse:quicksaveto("Kerpollo 2 Phase 7 - Lander Landed").
clearscreen.
SAS OFF.
LIGHTS OFF.
wait 2.
stage.
wait until stage:ready.
stage.
wait until stage:ready.
run launch_airless(60000).
