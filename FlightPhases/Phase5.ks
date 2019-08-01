@LAZYGLOBAL OFF.
// Phase 5: Crew Transfer
KUniverse:quicksaveto("Kerpollo 2 Phase 5 - Crew Transfer Pre-Landing").
clearscreen.
local uid_command to ship:crew[0]:part:uid.
print "Awaiting Crew Transfer".
local transfer_complete to false.

until transfer_complete {
  local crew_count_command to 0.
  local crew_count_lander to 0.

  for member in ship:crew {
    if member:part:uid = uid_command {
      set crew_count_command to crew_count_command + 1.
    } else {
      set crew_count_lander to crew_count_lander + 1.
    }
  }

  if crew_count_lander = 2 {
    set transfer_complete to true.
  }

  print "Crew Count Command = " + crew_count_command + "    " at(0,1).
  print "Crew Count Lander  = " + crew_count_lander + "    " at(0,2).

  wait 0.
}
clearscreen.
print "Crew Transfer Complete".
print "Ready for Undocking".
wait 3.
AG3 ON.
ship:partstagged("LanderDockingPort")[0]:UNDOCK.
clearscreen.
print "Send Landing Message".
local lander to VESSEL("Saturn V + LEM [Kerpollo 2.0] Lander").
lander:connection:sendmessage("Begin Landing").
