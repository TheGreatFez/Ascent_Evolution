// This script calculates a Maneuver Node to intercept a target body. It is assumed both target and ship are in near circular orbits
@LAZYGLOBAL OFF.
parameter target_periapsis is target:radius/2.

clearscreen.
for node in allnodes {remove node.}
RUNONCEPATH("Library/lib_execnode.ks").

function ETA_to_theta {

	parameter theta_test.

	local T_orbit to ship:orbit:period.
	local theta_ship to ship:orbit:trueanomaly.
	local e to ship:orbit:eccentricity.
	local GM to ship:body:mu.
	local a to ship:orbit:semimajoraxis.
	clearscreen.

	local EA_ship to 2*ARCTAN((TAN(theta_ship/2))/sqrt((1+e)/(1-e))).
	local MA_ship to EA_ship*constant:pi/180 - e*SIN(EA_ship).
	local EA_test to 2*ARCTAN((TAN(theta_test/2))/sqrt((1+e)/(1-e))).
	local MA_test to EA_test*constant:pi/180 - e*SIN(EA_test).
	local n to sqrt(GM/(a)^3).
	local eta_to_testpoint to (MA_test - MA_ship)/n.
	if eta_to_testpoint < 0 {
		set eta_to_testpoint to T_orbit + eta_to_testpoint.
	}

  //	print "ETA to " + round(theta_test,2) + " degrees True Anomaly is " + round(eta_to_testpoint,2) + " seconds".
  //	wait 2.
	return eta_to_testpoint.
}

function Phase_angFunc_target {
  //Assume target is set and both in same orbital plane
  parameter time_input is time:seconds.
  local ship_pos is -ship:body:position.
  local target_pos is target:position-ship:body:position.
  local Ship_angvel to VCRS(ship_pos,ship:velocity:orbit):normalized.
  local Phase_ang to VANG(ship_pos,target_pos).
  local Target_X_Ship to VCRS(target_pos,ship_pos).
  local dir_check to VDOT(Target_X_Ship,Ship_angvel).

  if dir_check > 0 {
  	set Phase_ang to 360-Phase_ang.
  }

  return Phase_ang.
}

function SetNode_BurnVector {
	parameter timeat,V_New.

	local V_timeat to velocityat(ship,timeat):orbit.

	local node_normal_vec to vcrs(ship:body:position,ship:velocity:orbit):normalized.
	local node_prograde_vec to V_timeat:normalized.
	local node_radial_vec to VCRS(node_normal_vec,node_prograde_vec).

	local burn_vector to (V_New - V_timeat).
	local burn_prograde to VDOT(node_prograde_vec,burn_vector).
	local burn_normal to VDOT(node_normal_vec,burn_vector).
	local burn_radial to VDOT(node_radial_vec,burn_vector).

	return NODE(timeat,burn_radial,burn_normal,burn_prograde).
}

function New_Vec_Mag {
  parameter New_Mag,vec1.

  local Old_Mag to vec1:mag.
  local New_Vec to (New_Mag/Old_Mag)*vec1.

  return vec1.
}

local R1 to ship:orbit:semimajoraxis.
local R2 to target:orbit:semimajoraxis.
local Warp_P_or_A to "TEMP".
if R1 < R2 {
	set Warp_P_or_A to "Apoapsis".
} else {
	set Warp_P_or_A to "Periapsis".
}

local Orbit_Match to abs(ship:orbit:lan - target:orbit:lan) < 0.1 AND abs(ship:orbit:inclination - target:orbit:inclination) < 0.01.
local Orbit_Match to Orbit_Match AND ship:orbit:eccentricity < 0.005.

if Orbit_Match {
	clearscreen.
	print "Orbits Match Inc and LAN".
	print "Moving to Periapsis Solver".
	wait 1.
} else {
	clearscreen.
	print "WARNING: Inclination Difference greater than 0.01 degrees!".
	print "Running Orbit Set Script".
	wait 3.
  local SMA_desired to ship:orbit:semimajoraxis - ship:body:radius.
  local INC_desired to target:orbit:inclination.
  local LAN_desired to target:orbit:LAN.
  if INC_desired < 0.01 {
    set LAN_desired to ship:orbit:LAN.
  }
	local desired_orbit to lexicon("INC",INC_desired,"LAN",LAN_desired,"APO",SMA_desired,"PER",SMA_desired).
	run change_orbit(desired_orbit).
}
clearscreen.
local line is 0.

local a_trans to (R1+R2)/2.
local ecc_trans to abs(R2-R1)/(R1+R2).
local T_ship to ship:orbit:period.
local T_target to target:orbit:period.
local T_trans to 2*constant:pi*sqrt((a_trans^3)/ship:body:MU).
local Phase_ang_desired to 180-(T_trans/T_target)*180.
set line to line + 1.
print "Phase_ang_desired = " + round(Phase_ang_desired,2) + "    " at(0,line).
local Phase_ang_current to Phase_angFunc_target().
set line to line + 1.
print "Phase_ang_current = " + round(Phase_ang_current,2) + "    " at(0,line).
local Phase_ang_dot to abs(360/T_ship - 360/T_target).
local Phase_ang_diff is 0.

if T_ship > T_target {
  set Phase_ang_diff to Phase_ang_desired - Phase_ang_current.
} else {
  set Phase_ang_diff to Phase_ang_current - Phase_ang_desired.
}
if Phase_ang_diff < 0 {
  set Phase_ang_diff to Phase_ang_current + 360-Phase_ang_desired.
}
local Phase_Time to Phase_ang_diff/Phase_ang_dot.
if Phase_Time < 0 {
  set Phase_Time to T_ship - Phase_Time.
}
set line to line + 1.
print "Phase_ang_diff = " + round(Phase_ang_diff,2) + "    " at(0,line).
local Phase_Time_Total to time:seconds + Phase_Time.
set line to line + 1.
print "Phase_Time_Total = " + round(Phase_Time_Total) + "    " at(0,line).
local Burn_Position to (positionat(ship,Phase_Time_Total) - ship:body:position).
local Burn_Velocity to velocityat(ship,Phase_Time_Total):orbit.
local Burn_R to Burn_Position:mag.
local Ship_angvel to VCRS(Burn_Position,Burn_Velocity).
local Burn_SMA to (R2 + Burn_R)/2.
local desired_speed to sqrt(ship:body:mu*(2/Burn_R - 1/Burn_SMA)).
local desired_burn_dir to VCRS(Ship_angvel,Burn_Position):normalized.
local desired_velocity to desired_speed*desired_burn_dir.

local test_node to SetNode_BurnVector(Phase_Time_Total,desired_velocity).
add test_node.

local bad_intercept to false.
local dont_start to false.
local orbit_check is 1.
local test_periapsis is 1.
if not(nextnode:orbit:hasnextpatch) {
  set bad_intercept to true.
  set dont_start to true.
} else {
  set orbit_check to nextnode:orbit:nextpatch.
  set test_periapsis to orbit_check:periapsis.
}

local Delta_V is desired_speed.
local delta is 0.5.
local delta_dir is 1.
local tollerance to 100.
local i is 1.

clearscreen.

until i >= 100 OR dont_start {

  local diff1 to abs(test_periapsis - target_periapsis).
	set Delta_V to Delta_V + delta*delta_dir.
  set desired_velocity to (Delta_V/desired_velocity:mag)*desired_velocity.

  for n in allnodes { remove n.}
  local test_node to SetNode_BurnVector(Phase_Time_Total,desired_velocity).
  add test_node.
  wait 0.
  set orbit_check to nextnode:orbit:nextpatch.

	set test_periapsis to orbit_check:periapsis.
	local diff2 to abs(test_periapsis - target_periapsis).

	if diff2 > diff1 {
		set delta to delta/2.
		set delta_dir to -1*delta_dir.
	}

	if diff2 < tollerance {
		set bad_intercept to false.
		break.
		}

	set i to i+1.
	if i = 100 {
		if diff2 > 10*tollerance {
			set bad_intercept to true.
		} else {
			set bad_intercept to false.
		}
	}
	print "Iteration = " + i at(0,3).
	print "delta = " + delta at (0,4).
	print "delta_dir = " + delta_dir at (0,5).
	print "test_periapsis = " + round(test_periapsis,2) at (0,6).
	print "target_periapsis = " + target_periapsis at (0,7).
  // Uncomment to slow down iteration process for visualization
  //wait 1.

}
clearscreen.

if bad_intercept {
	print "Bad Intercept".
} else {
	print "Successful Intercept".
	print "Iterations " + i.
	print "Periapsis within " + round(abs(test_periapsis - target_periapsis),2) + " meters".
  wait 2.
  clearvecdraws().
  ExecuteNode().
	}
