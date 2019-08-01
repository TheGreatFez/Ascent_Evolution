@LAZYGLOBAL OFF.
// Phase 10: Command Module Wait for Docking and Transfer
AG1 ON.
clearscreen.
local lander to ship:body.
local lander_pos to lander:position.
local docking_port to ship:dockingports[0].
SAS OFF.
lock steering to ship:facing.
local align_for_docking to false.
local ship_state to "Waiting for Message".
local aquiring to docking_port:state:split(" ")[0] = "Acquire".
local docked to docking_port:state:split(" ")[0] = "Docked".

until docked {
  wait 0.
  set aquiring to docking_port:state:split(" ")[0] = "Acquire".
  set docked to docking_port:state:split(" ")[0] = "Docked".
  if docked {
    set ship_state to "Docked".
  }
  if not(ship:messages:empty) {
    local RECEIVED to SHIP:MESSAGES:POP.
    IF RECEIVED:CONTENT = "Align For Docking" {
      set lander to VESSEL("Saturn V + LEM [Kerpollo 2.0] Lander").
      set align_for_docking to true.
      set lander_pos to lander:position.
      lock steering to lander_pos.
      SAS OFF.
    }
  }
  if align_for_docking AND aquiring {
    set align_for_docking to false.
    unlock steering.
    SAS ON.
    set ship_state to "Aquiring".
  }
    print ship_state + "            " at(0,0).
}

print ship_state + "            " at(0,0).
local transfer_complete to false.
local uid_command to ship:crew[0]:part:uid.

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
  if crew_count_lander = 0 {
    set transfer_complete to true.
  }
  print "Crew Count Command = " + crew_count_command + "    " at(0,1).
  print "Crew Count Lander  = " + crew_count_lander + "    " at(0,2).
  wait 0.
}

clearscreen.
wait until KUNIVERSE:CANQUICKSAVE.
KUniverse:quicksaveto("Kerpollo 2 Phase 10 - Ready for Return, Crew Transfer Complete").
