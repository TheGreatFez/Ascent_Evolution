@LAZYGLOBAL OFF.


if ship:dockingports[0]:state:split(" ")[0] = "Docked" {
  print "Crew Transfer Complete, Undocking".
  wait 2.
  ship:partstagged("LanderDockingPort")[0]:UNDOCK.
}
wait until KUNIVERSE:CANQUICKSAVE.
KUniverse:quicksaveto("Kerpollo 2 Phase 11 - Escape from Minmus").
clearscreen.

RUNONCEPATH("Library/lib_BisectionSolver.ks").
RUNONCEPATH("Library/lib_execnode.ks").
run escape_traj.

local LAN to ship:body:orbit:LAN.
local INC to ship:body:orbit:inclination.
local AOP to ship:orbit:argumentofperiapsis.
local PER to apoapsis.
local APO to apoapsis.
local DesiredOrbit to lexicon("LAN",LAN,"INC",INC,"AOP",AOP,"PER",PER,"APO",APO).

run change_orbit(DesiredOrbit).

local escape_set to false.

function escape_score {
  parameter delta_speed.
  local target_peri is 30000.

  return target_peri - getEscapeTraj_Peri(delta_speed).

}

local min_speed is -ship:body:orbit:velocity:orbit:mag.
local peri_solver is makeBiSectSolver(escape_score@,-10,min_speed).
clearscreen.
local iteration is 0.

until escape_set {
  local peri_testpoints is peri_solver().
  local diff is abs(peri_testpoints[0][0] - peri_testpoints[1][0]).

  if diff < 0.1 {
    set escape_set to true.
  }
  local line_count is 0.
  print "Exit Speed 1   : " + round(peri_testpoints[0][0],1) + "     " at(0,line_count).
  set line_count to line_count + 1.
  print "Exit Speed 2   : " + round(peri_testpoints[1][0],1) + "     " at(0,line_count).
  set line_count to line_count + 1.
  print "Exit Speed Avg : " + round(peri_testpoints[2][0],1) + "     " at(0,line_count).
  set line_count to line_count + 1.
  print "Score          : " + round(peri_testpoints[2][1],1) + "     " at(0,line_count).
  set line_count to line_count + 1.
  print "Iteration      : " + iteration + "     " at(0,line_count).
  set iteration to iteration + 1.
  wait 1.
}

ExecuteNode().

wait until KUNIVERSE:CANQUICKSAVE.
