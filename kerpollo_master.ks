@LAZYGLOBAL OFF.
set config:ipu to 2000.
KUniverse:quicksaveto("Kerpollo 2 Phase 1 - Launch").
run launch.

KUniverse:quicksaveto("Kerpollo 2 Phase 2 - Launch Complete").
local Target_body to Minmus.
set target to Target_body.
run rdz_body.

clearscreen.
print "Warping to Munar Insertion".
wait 3.
warpto(time:seconds + eta:transition + 10).
wait until ship:body = BODY("Minmus").

KUniverse:quicksaveto("Kerpollo 2 Phase 3 - Pre-Capture at Minmus").
run capture.
run CircularizePE.
local SMA_desired to ship:orbit:semimajoraxis - ship:body:radius.
local INC_desired to 0.0.
local LAN_desired to ship:orbit:LAN.
local DesiredOrbit to lexicon("INC",INC_desired,"LAN",LAN_desired,"APO",SMA_desired,"PER",SMA_desired).
run change_orbit(DesiredOrbit).

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

KUniverse:quicksaveto("Kerpollo 2 Phase 5 - Docked Pre-Landing").
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

KUniverse:quicksaveto("Kerpollo 2 Phase 6 - Docked Pre-Landing, Crew Transfer Complete").

AG3 ON.
ship:partstagged("LanderDockingPort")[0]:UNDOCK.
clearscreen.
print "Send Landing Message".
local lander to VESSEL("Saturn V + LEM [Kerpollo 2.0] Lander").
lander:connection:sendmessage("Begin Landing").
