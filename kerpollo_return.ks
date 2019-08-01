@LAZYGLOBAL OFF.

parameter Phase is 10, continue is true..

function runPhase {
  parameter Phase.
  local PhasePath is "FlightPhases/Phase" + Phase:tostring + ".ks".
  RUNPATH(PhasePath).
}

if continue {
  local count is Phase.
  until count = 13 {
    runPhase(count).
    set count to count + 1.
  }
} else {
  runPhase(Phase).
}
