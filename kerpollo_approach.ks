@LAZYGLOBAL OFF.

parameter Phase is 1, continue is true.

function runPhase {
  parameter Phase.
  local PhasePath is "FlightPhases/Phase" + Phase:tostring + ".ks".
  RUNPATH(PhasePath).
}

if continue {
  local count is Phase.
  until count = 6 {
    runPhase(count).
    set count to count + 1.
  }
} else {
  runPhase(Phase).
}
