@LAZYGLOBAL OFF.
// Phase 8: Rendezvous with Command Module
clearvecdraws().
KUniverse:quicksaveto("Kerpollo 2 Phase 9 - Rendezvous Complete").
clearscreen.
print "Sending Pre-Docking Alignment to Main Command Module".
local main_pod to VESSEL("Saturn V + LEM [Kerpollo 2.0]").
main_pod:connection:sendmessage("Align For Docking").
print "Waiting for Command Module to stabilize".
wait 5.
run dock_to_target.
clearvecdraws().
