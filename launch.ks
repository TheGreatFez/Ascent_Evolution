// *********************************************** //
//    Ascent Program Evolution: V3.0   //
// *********************************************** //

// Clean up work area
clearscreen.
set ship:control:pilotmainthrottle to 0.

// Main Inputs
local TargetOrbit is 250000. // The target altitude of our parking orbit.

// Functions

function PitchProgram_Sqrt {
	parameter InputData, switch_alt is 250.
	local pitch_ang to 90.
	local FollowPitch to true.
	local scale_factor to 1.0.
	local alt_diff is scale_factor*ship:body:atm:height - switch_alt.
	
	local SrfProg_pitch to 90 - VANG(UP:vector,srfprograde:vector).
	local ObtProg_pitch to 90 - VANG(UP:vector,prograde:vector).
	
	if AscentStage = 2 {
		set pitch_ang to 90 - max(0,min(90,90*sqrt((altitude - switch_alt)/alt_diff))).
		if ObtProg_pitch > pitch_ang AND FollowPitch{
			set pitch_ang to ObtProg_pitch.
		}
	} else if AscentStage = 3 {
		set pitch_ang to SrfProg_pitch. 
	}
	
	return pitch_ang.
}

function PitchProgram_Rate {
	parameter Pitch_Data.
	local v_speed1 to Pitch_Data["V_Speed"].
	local t_1 to Pitch_Data["Time"].
	local v_speed2 to verticalspeed.
	local t_2 to time:seconds.
	local dt to max(0.0001,t_2 - t_1).
	local v_accel to max(0.001,(v_speed2 - v_speed2)/dt).
	local alt_final is Pitch_Data["Alt_Final"].
	local alt_diff is alt_final - altitude.
	
	local a to .5*v_accel.
	local b to verticalspeed.
	local c to -alt_diff.
	
	local time_to_alt to ((-b) + sqrt(max(0,b^2 - 4*a*c)))/(2*a).
	local pitch_des to Pitch_Data["Pitch"].
	local pitch_final to Pitch_Data["Pitch_Final"].
	local pitch_rate to max(0,(pitch_final - pitch_des)/time_to_alt).
	
	local pitch_des to min(pitch_final,max(0,pitch_des + dt*pitch_rate)).
	
	set Pitch_Data["Pitch"] to pitch_des.
	set Pitch_Data["Time"] to t_2.
	set Pitch_Data["V_Speed"] to v_speed2.
	
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

function vis_via_speed {
	parameter R, a is ship:orbit:semimajoraxis.
	local R_val to ship:body:radius + R.
	return sqrt(ship:body:mu*(2/R_val - 1/a)).
}

function Circularize_DV_Calc{
	local Vapo_cir to circ_speed(apoapsis).
	local Delta_V to  Vapo_cir - vis_via_speed(apoapsis).
	local CirPer to NODE(TIME:seconds + eta:apoapsis, 0, 0, Delta_V).
	ADD CirPer.
	return CirPer:deltav:mag.
}

function inst_az {
	parameter
		inc. // target inclination
	
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
local inc_des   to 10. // Desired inclination
local compass to inst_az(inc_des).
lock throttle to 1.
lock steering to lookdirup(heading(compass,90-pitch_ang):vector,ship:facing:upvector).
stage.
AG3 on. // Open the Terminal

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
set Pitch_Data to lexicon().
Pitch_Data:ADD("Time",time:seconds).
Pitch_Data:ADD("Pitch",0).
Pitch_Data:ADD("Pitch_Final",85).
Pitch_Data:ADD("V_Speed",verticalspeed).
Pitch_Data:ADD("Alt_Final",0.7*ship:body:atm:height).

local switch_alt is 0.

// Run Mode Variables
local AscentStage is 1.
local ThrottleStage is 1.

// Delta V Variables
set DeltaV_Data to lexicon().
DeltaV_Data:ADD("Total",0).
DeltaV_Data:ADD("Gain",0).
DeltaV_Data:ADD("Time",time:seconds).
DeltaV_Data:ADD("Thrust_Accel",throttle*availablethrust/mass).
DeltaV_Data:ADD("Accel_Vec",throttle*ship:sensors:acc).

local line is 1.

until AscentStage = 2 AND altitude > ship:body:ATM:height {
	// Run Mode Logic
	
	if apoapsis > TargetOrbit AND ThrottleStage = 1 {
		lock throttle to 0.
		set ThrottleStage to 2.
		set AscentStage to 2.
	}
	
	set Pitch_Data to PitchProgram_Rate(Pitch_Data).
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
	
	
	// Delta V Calculations
	set DeltaV_Data to Calculate_DeltaV(DeltaV_Data).
	
	// Delta V Printout
	set line to line + 3.
	print "DeltaV_total  = " + round(DeltaV_Data["Total"]) + "   " at(0,line).
	set line to line + 1.
	print "DeltaV_gain   = " + round(DeltaV_Data["Gain"]) + "   " at(0,line).
	set line to line + 1.
	print "DeltaV_Losses = " + round(DeltaV_Data["Total"] - DeltaV_Data["Gain"]) + "   " at(0,line).
	set line to line + 1.
	print "DeltaV_Eff    = " + round(100*DeltaV_Data["Gain"]/DeltaV_Data["Total"]) + "%   " at(0,line).
	
	wait 0.
}

local DV_Circ to Circularize_DV_Calc().
set line to line + 3.
print "Total Delta V for Circularization " + round(DeltaV_Data["Total"] + DV_Circ) + "    " at(0,line).