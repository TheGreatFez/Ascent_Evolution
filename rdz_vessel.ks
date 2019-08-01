// This script calculates a Maneuver Node to intercept a target body. It is assumed both target and ship are in near circular orbits
@LAZYGLOBAL OFF.

clearscreen.
for node in allnodes {remove node.}
RUNONCEPATH("Library/lib_execnode.ks").
RUNONCEPATH("Library/lib_Hysteresis.ks").

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

function Vmax_target_relative {
  parameter buffer_dist.

  local R_rel to target:position.
	local max_acc to availablethrust/mass.
	local Vmax to sqrt(MAX(0,2*(R_rel:mag - buffer_dist)*max_acc)).

  return Vmax.
}

function Amax_target_relative {
  parameter buffer_dist.

  local R_rel to target:position.
	local max_acc to availablethrust/mass.
  local V1 to ship:velocity:orbit - target:velocity:orbit.
  local Vi to VDOT(R_rel:normalized,V1).
  local Amax to 0.
  if Vi > 0 {
    set Amax to (Vi^2)/(2*(R_rel:mag - buffer_dist)).
  }

  return Amax/max_acc.
}

function PN_throttle {
  parameter desired_position,throttle_available is 1.

  local Gain to 4.

  local R1 to desired_position.
  local V1 to ship:velocity:orbit - target:velocity:orbit.
  local PN_acc_vec to Gain*VCRS(V1,VCRS(R1,V1)/VDOT(R1,R1)).

  //- VDOT(Inertial_acc,PN_acc_vec_norm)*PN_acc_vec_norm.

  return (PN_acc_vec*mass)/(max(0.001,availablethrust*throttle_available)).
}

function Retro_throttle {
  parameter buffer_dist is 50.

  local Vmax to Vmax_target_relative(buffer_dist).
  local V_rel to ship:velocity:orbit - target:velocity:orbit.
  local R_rel to target:position.
  local Speed_rel to VDOT(R_rel:normalized,V_rel).
  local Speed_diff to Vmax - Speed_rel.
  local t_0 is 5.
  local max_acc to max(0.001,availablethrust/mass).
  local throttle_needed is max(0,1 - Speed_diff/(t_0*max_acc)).
  if Speed_diff < 0 {
    set throttle_needed to Speed_rel/max_acc.
  }


  //local throttle_needed is Amax_target_relative(buffer_dist).

  return -V_rel:normalized*throttle_needed.
}

function Approach_throttle {
  parameter buffer_dist is 50.
  local max_speed is min(25,target:position:mag/30).
  local V_des_vec is target:position:normalized*max_speed.
  local V_rel to ship:velocity:orbit - target:velocity:orbit.
  local Delta_V_vec is V_des_vec - V_rel.
  local max_acc to max(0.001,availablethrust/mass).

  return Delta_V_vec/max_acc.
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
	print "Moving to Rendezvous".
	wait 1.
} else {
	clearscreen.
	print "Running Orbit Set Script".
	wait 1.
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

KUniverse:quicksaveto("Rdz Vessel - TestSave").
local line is 0.

local a_trans to (R1+R2)/2.
local ecc_trans to abs(R2-R1)/(R1+R2).
local T_ship to ship:orbit:period.
local T_target to target:orbit:period.
local T_trans to 2*constant:pi*sqrt((a_trans^3)/ship:body:MU).
local tmp_ang to 180 -(T_trans/T_target)*180..
if tmp_ang < 0 {
  set tmp_ang to 360 + tmp_ang.
}
local Phase_ang_desired to tmp_ang.
set line to line + 1.
print "Phase_ang_desired = " + round(Phase_ang_desired,2) + "    " at(0,line).
local Phase_ang_current to Phase_angFunc_target().
set line to line + 1.
print "Phase_ang_current = " + round(Phase_ang_current,2) + "    " at(0,line).
local Phase_ang_dot to abs(360/T_ship - 360/T_target).
local Phase_ang_diff is Phase_ang_desired - Phase_ang_current.

if Phase_ang_diff < 0 {
  set Phase_ang_diff to 360 + Phase_ang_diff.
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

local rdz_node to SetNode_BurnVector(Phase_Time_Total,desired_velocity).
add rdz_node.
ExecuteNode().
clearscreen.

local zero_out_time to 0.

if Warp_P_or_A = "Apoapsis" {
  set zero_out_time to time:seconds + eta:apoapsis.
  warpto(zero_out_time).
} else {
  set zero_out_time to time:seconds + eta:periapsis.
  warpto(zero_out_time).
}
wait until warp <= 0 AND KUniverse:timewarp:issettled.

// Catch Up Mechanics
local right_hand_limit is .05.
local left_hand_limit is .01.
local init_output is false.
local throttle_hyst to makeHysteresis(right_hand_limit, left_hand_limit, init_output).

local App_right_hand_limit is min(25,target:position:mag/30)-1.
local App_left_hand_limit is 0.
local target_relative_velocity to ship:velocity:orbit - target:velocity:orbit.
local relative_speed to VDOT(target:position:normalized,target_relative_velocity).
local init_output is false.
if relative_speed < right_hand_limit {
  set init_output to true.
}
local right_hand_output is false.
local approach_hyst to makeHysteresis(App_right_hand_limit, App_left_hand_limit, init_output, right_hand_output).

local throttle_val to 0.
lock throttle to throttle_val.
local throttle_vec to PROGRADE:vector:normalized/1000.
local steering_vec to throttle_vec.
lock steering to LOOKDIRUP(steering_vec,facing:topvector).

local target_dist to 25.

local Throttle_Vec_draw to vecdraw().
set Throttle_Vec_draw:startupdater to {return V(0,0,0).}.
set Throttle_Vec_draw:vecupdater to {return throttle_vec*10.}.
set Throttle_Vec_draw:show to true.

local Relative_Position_draw to vecdraw().
set Relative_Position_draw:startupdater to {return V(0,0,0).}.
set Relative_Position_draw:vecupdater to {return target:position.}.
set Relative_Position_draw:show to true.
set Relative_Position_draw:color to GREEN.

local Relative_Velocity_draw to vecdraw().
set Relative_Velocity_draw:startupdater to {return V(0,0,0).}.
set Relative_Velocity_draw:vecupdater to {return ship:velocity:orbit - target:velocity:orbit.}.
set Relative_Velocity_draw:show to true.
set Relative_Velocity_draw:color to RED.

local target_reached is false.
local approaching is false.

until target_reached {
  local target_pos to target:position.
  local throttle_ang to VANG(throttle_vec,ship:facing:vector).
  set target_relative_velocity to ship:velocity:orbit - target:velocity:orbit.
  set relative_speed to VDOT(target:position:normalized,target_relative_velocity).

  if target_relative_velocity:mag < 0.1 {
    set target_reached to true.
  }

  local Retro_vec to Retro_throttle(target_dist).
  local throttle_available to sqrt(1 - min(1,Retro_vec:mag^2)).
  local PN_vec to PN_throttle(target_pos,throttle_available).
  local Approach_vec to Approach_throttle(target_dist).

  if approach_hyst(relative_speed + 1) AND not(approaching) {
    set throttle_vec to Approach_vec.
  } else {
    set throttle_vec to Retro_vec + PN_vec.
    set approaching to true.
  }

  if throttle_hyst(throttle_vec:mag) {
    set steering_vec to throttle_vec.
  } else {
    set steering_vec to -target_relative_velocity.
  }

  if throttle_hyst(throttle_vec:mag) {
    set throttle_val to throttle_vec:mag.
  } else {
    set throttle_val to 0.
  }

  local line_count to 0.

  print "Throttle Hyst   = " + throttle_hyst(throttle_vec:mag) + "     " at(0,line_count).
  set line_count to line_count + 1.
  print "Approach Hyst   = " + approach_hyst(relative_speed) + "     " at(0,line_count).
  set line_count to line_count + 1.
  print "Retro Throttle  = " + round(100*Retro_vec:mag,1) + "%     " at(0,line_count).
  set line_count to line_count + 1.
  print "PN Throttle     = " + round(100*PN_vec:mag,1) + "%     " at(0,line_count).
  set line_count to line_count + 1.
  print "Throttle        = " + round(100*throttle_vec:mag,1) + "%     " at(0,line_count).
  set line_count to line_count + 1.
  print "Throttle Angle  = " + round(throttle_ang,2) + "     " at(0,line_count).
  set line_count to line_count + 1.
  print "Relative Speed  = " + round(relative_speed,2) + "     " at(0,line_count).
  set line_count to line_count + 1.
  print "Amax            = " + round(Amax_target_relative(100),2) + "     " at(0,line_count).
  wait 0.
}

lock throttle to 0.
