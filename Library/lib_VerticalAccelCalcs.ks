function g {
  local R to ship:body:position:mag.
  local GM to ship:body:mu.

  return GM/(R)^2.
}

function V_accel_inertial {
  local V is ship:velocity:orbit.
  local R is -ship:body:position.
  local tmp_vec is VCRS(VCRS(R,V),R):normalized.
  local centri_acc to VDOT(tmp_vec,V)^2/R:mag.

  return centri_acc - g().
}
