clearscreen.
SAS OFF.
RCS OFF.
function CaptureVelVec {

  parameter R2 is ship:body:radius*1.5.

  local R1 to ship:body:position:mag.
  local a_cap to (R1+R2)/2.
  local ecc_cap to abs(R1-R2)/(R1+R2).
  local vel_speed to sqrt(ship:body:MU*(2/R1 - 1/a_cap)).

  local temp_vec to VCRS(-1*ship:body:position,ship:velocity:orbit).
  // Prograde Check
  if VDOT(V(0,-1,0),temp_vec) < 0 {
    set temp_vec to -temp_vec.
  }
  local vel_dir to VCRS(temp_vec,-1*ship:body:position):normalized.

  return vel_dir*vel_speed.
}

local vec_diff to CaptureVelVec() - ship:velocity:orbit.
local Delta_V to vec_diff:mag.
local thr to 0.
local max_acc to maxthrust/mass.

lock throttle to thr.
lock steering to vec_diff.
clearscreen.
print "Aligning to Burn Vector" at(0,0).
until VANG(vec_diff,ship:facing:vector) < 1 {
  print "Angle Difference = " + round(VANG(vec_diff,ship:facing:vector),1) at(0,1).
}

clearscreen.
print "Burning to Capture" at(0,0).
until Delta_V < 0.1 {

  set vec_diff to CaptureVelVec() - ship:velocity:orbit.
  set Delta_V to vec_diff:mag.

  set max_acc to max(0.001,maxthrust/mass).
  set thr to Delta_V/max_acc.

  print "Delta V remaining = " + round(Delta_V,1) at(0,1).
}
