// *********************************************** //
//    Ascent Program Airless: V1.0   //
// *********************************************** //
@LAZYGLOBAL OFF.
SAS OFF.
// Clean up work area
clearscreen.
set ship:control:pilotmainthrottle to 0.

// Main Inputs
parameter TargetOrbit is 250000. // The target altitude of our parking orbit.
RUNONCEPATH("Library/lib_execnode.ks").
RUNONCEPATH("Library/lib_OrbitalMechanics_Functions.ks").
// Functions

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

function PitchProgram_Airless {
	parameter Pitch_Data.

  local thrust to max(0.0001,availablethrust).
  local V_throttle to 0.

  local V_speed_des to airspeed*sin(Pitch_Data["Gamma_des"]).
  local V_speed_diff to max(0,V_speed_des - verticalspeed).
  local t_0 to 1.

  if verticalspeed < Pitch_Data["Speed_switch"] {
    set V_throttle to 1.
  } else {
    set V_throttle to min(1,max(0,(-V_accel_inertial() + (V_speed_diff/t_0))*mass/thrust)).
  }

  set Pitch_Data["Pitch"] to arcsin(V_throttle).

  local line to 15.
	print "V_throttle         = " + round(V_throttle,2) + "     " at(0,line).
  set line to line + 1.
  print "V_speed_des        = " + round(V_speed_des,2) + "     " at(0,line).
  set line to line + 1.
  print "V_speed_diff       = " + round(V_speed_diff,2) + "     " at(0,line).
  set line to line + 1.
  print "V_accel_inertial   = " + round(V_accel_inertial(),2) + "     " at(0,line).


	return Pitch_Data.
}

function Calculate_DeltaV {
	parameter DeltaV_Data.

	local thrust_accel_1 to DeltaV_Data["Thrust_Accel"].
	local thrust_accel_2 to throttle*availablethrust/mass.
	local a_vec1 to DeltaV_Data["Accel_Vec"].
	local a_vec2 to throttle*ship:sensors:acc.
	local time1 to DeltaV_Data["Time"].
	local time2 to time:seconds.
	local dt to max(0.0001,time2 - time1).
	local thrust_accel to (thrust_accel_1 + thrust_accel_2)/2.
	local a_vec to (a_vec1 + a_vec2)/2.
	local thrust_vec to thrust_accel*ship:facing:vector.
	set DeltaV_Data["Total"] to DeltaV_Data["Total"] + thrust_accel*dt.
	local obt_vel_norm to ship:velocity:orbit:normalized.
	set DeltaV_Data["Gain"] to DeltaV_Data["Gain"] + dt*(VDOT(obt_vel_norm,a_vec)).

	set DeltaV_Data["Time"] to time2.
	set DeltaV_Data["Accel_Vec"] to a_vec2.
	set DeltaV_Data["Thrust_Accel"] to thrust_accel_2.

	return DeltaV_Data.
}

function circ_speed {
	parameter R.
	local R_val to ship:body:radius + R.
	return sqrt(ship:body:mu/R_val).
}

function Circularize_DV_Calc{
	local Vapo_cir to circ_speed(apoapsis).
	local Delta_V to  Vapo_cir - vis_via_speed(apoapsis + ship:body:radius).
	local CirPer to NODE(TIME:seconds + eta:apoapsis, 0, 0, Delta_V).
	ADD CirPer.
	return CirPer:deltav:mag.
}

function inst_az {
	parameter	inc. // target inclination

	// find orbital velocity for a circular orbit at the current altitude.
	local V_orb is max(ship:velocity:orbit:mag + 1,sqrt( body:mu / ( ship:altitude + body:radius))).

	// Use the current orbital velocity
	//local V_orb is ship:velocity:orbit:mag.

	// project desired orbit onto surface heading
	local az_orb is arcsin ( max(-1,min(1,cos(inc) / cos(ship:latitude)))).
	if (inc < 0) {
		set az_orb to 180 - az_orb.
	}

	// create desired orbit velocity vector
	local V_star is heading(az_orb, 0)*v(0, 0, V_orb).

	// find horizontal component of current orbital velocity vector
	local V_ship_h is ship:velocity:orbit - vdot(ship:velocity:orbit, up:vector:normalized)*up:vector:normalized.

	// calculate difference between desired orbital vector and current (this is the direction we go)
	local V_corr is V_star - V_ship_h.

	// project the velocity correction vector onto north and east directions
	local vel_n is vdot(V_corr, ship:north:vector).
	local vel_e is vdot(V_corr, heading(90,0):vector).

	// calculate compass heading
	local az_corr is arctan2(vel_e, vel_n).
	return az_corr.
}

// Ignition
local pitch_ang to 0.
local inc_des   to 0. // Desired inclination
if abs(inc_des) < abs(ship:geoposition:LAT) {
  set inc_des to ship:geoposition:LAT.
}
local compass to inst_az(inc_des).
lock throttle to 1.
lock steering to lookdirup(heading(compass,pitch_ang):vector,ship:facing:upvector).
if availablethrust = 0 {
  stage.
}

// Basic Staging:
local current_max to maxthrust.
when maxthrust < current_max OR availablethrust = 0 then {
	lock throttle to 0.
	stage.
	lock throttle to 1.
	set current_max to maxthrust.
	preserve.
}

// Pitch Program Parameters
local min_VS is 10.
local Pitch_Data to lexicon().
Pitch_Data:ADD("Time",time:seconds).
Pitch_Data:ADD("Pitch",0).
Pitch_Data:ADD("Gamma_des",10).
Pitch_Data:ADD("Speed_switch",3).

local switch_alt is 0.

// Run Mode Variables
local AscentStage is 1.
local ThrottleStage is 1.

local line is 1.

until AscentStage = 2 {
	// Run Mode Logic

	if apoapsis > TargetOrbit AND ThrottleStage = 1 {
		lock throttle to 0.
		set ThrottleStage to 2.
		set AscentStage to 2.
	}

	set Pitch_Data to PitchProgram_Airless(Pitch_Data).
	set pitch_ang to Pitch_Data["Pitch"].
	set compass to inst_az(inc_des).

	// Variable Printout
	set line to 1.
	print "ThrottleStage = " + ThrottleStage + "   " at(0,line).
	set line to line + 1.
	print "AscentStage   = " + AscentStage + "   " at(0,line).
	set line to line + 1.
	print "pitch_ang     = " + round(pitch_ang,2) + "   " at(0,line).
	set line to line + 1.
	print "compass      = " + round(compass,2) + "   " at(0,line).
	set line to line + 1.
	print "altitude      = " + round(altitude) + "   " at(0,line).
	set line to line + 1.
	print "switch_alt    = " + round(switch_alt) + "   " at(0,line).
	set line to line + 1.
	print "apoapsis      = " + round(apoapsis) + "   " at(0,line).
	set line to line + 1.
	print "TargetOrbit   = " + TargetOrbit + "   " at(0,line).

	wait 0.
}

local DV_Circ to Circularize_DV_Calc().
set line to line + 3.

print "Ascent Complete, Proceeding to Circularize".
wait 3.
wait until altitude > 3000.
ExecuteNode().
