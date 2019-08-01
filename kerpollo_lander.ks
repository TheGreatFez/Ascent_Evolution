@LAZYGLOBAL OFF.

parameter Phase is 6, continue is false.

function runPhase {
  parameter Phase.
  local PhasePath is "FlightPhases/Phase" + Phase:tostring + ".ks".
  RUNPATH(PhasePath).
}

if continue {
  local count is Phase.
  until count = 10 {
    runPhase(count).
    set count to count + 1.
  }
} else {
  runPhase(Phase).
}
