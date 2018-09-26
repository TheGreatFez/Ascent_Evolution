// *********************************************** //
//    Ascent Program Evolution: Gravity Turn 2.0   //
// *********************************************** //

// Clean up work area
clearscreen.
set ship:control:pilotmainthrottle to 0.

// Main Inputs
local TargetOrbit is 150000. // The target altitude of our parking orbit.

// Functions

function PitchProgram {
	parameter AscentStage, switch_alt is 250.
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

declare function circ_speed {
	parameter R.
	local R_val to ship:body:radius + R.
	return sqrt(ship:body:mu/R_val).
}

declare function vis_via_speed {
	parameter R, a is ship:orbit:semimajoraxis.
	local R_val to ship:body:radius + R.
	return sqrt(ship:body:mu*(2/R_val - 1/a)).
}

declare function Circularize_DV_Calc{
	local Vapo_cir to circ_speed(apoapsis).
	local Delta_V to  Vapo_cir - vis_via_speed(apoapsis).
	local CirPer to NODE(TIME:seconds + eta:apoapsis, 0, 0, Delta_V).
	ADD CirPer.
	return CirPer:deltav:mag.
}

// Ignition
local pitch_ang to 90.
lock throttle to 1.
lock steering to heading(90,pitch_ang).
stage.
AG1 on.

// Basic Staging:
local current_max to maxthrust.
when maxthrust < current_max OR availablethrust = 0 then {
	lock throttle to 0.
	stage.
	lock throttle to 1.
	set current_max to maxthrust.
	preserve.
}

// Gravity Turn Parameters
local min_VS is 30.
local pitch_over_ang is 2.
local switch_alt is 0.

// Run Mode Variables
local AscentStage is 1.
local ThrottleStage is 1.

// Delta V Variables
local DeltaV_total is 0.
local DeltaV_gain is 0.
local time1 is time:seconds.
local time2 is time:seconds.
local dt is 0.00001.
local thrust_accel_1 is throttle*availablethrust/mass.
local thrust_accel_2 is throttle*availablethrust/mass.
local a_vec1 is throttle*ship:sensors:acc.
local a_vec2 is throttle*ship:sensors:acc.
local line is 1.

until AscentStage = 3 AND altitude > ship:body:ATM:height {
	// Run Mode Logic
	if verticalspeed > min_VS AND AscentStage = 1 {
		set AscentStage to 2.
		set switch_alt to altitude.
	}
	
	if apoapsis > TargetOrbit AND ThrottleStage = 1 {
		lock throttle to 0.
		set ThrottleStage to 2.
		set AscentStage to 3.
	}
	
	// Pitch Program 
	set pitch_ang to PitchProgram(AscentStage,switch_alt).
	
	
	// Variable Printout
	set line to 1.
	print "ThrottleStage = " + ThrottleStage + "   " at(0,line).
	set line to line + 1.
	print "AscentStage   = " + AscentStage + "   " at(0,line).
	set line to line + 1.
	print "pitch_ang     = " + round(pitch_ang) + "   " at(0,line).
	set line to line + 1.
	print "altitude      = " + round(altitude) + "   " at(0,line).
	set line to line + 1.
	print "switch_alt    = " + round(switch_alt) + "   " at(0,line).
	set line to line + 1.
	print "apoapsis      = " + round(apoapsis) + "   " at(0,line).
	set line to line + 1.
	print "TargetOrbit   = " + TargetOrbit + "   " at(0,line).
	
	
	// Delta V Calculations
	set thrust_accel_2 to throttle*availablethrust/mass.
	set a_vec2 to throttle*ship:sensors:acc.
	set time2 to time:seconds.
	set dt to time2 - time1.
	local thrust_accel to (thrust_accel_1 + thrust_accel_2)/2.
	local a_vec to (a_vec1 + a_vec2)/2.
	set thrust_vec to thrust_accel*ship:facing:vector.
	set DeltaV_total to DeltaV_total + thrust_accel*dt.
	local obt_vel_norm to ship:velocity:orbit:normalized.
	set DeltaV_Gain to DeltaV_gain + dt*(VDOT(obt_vel_norm,a_vec)).
	local DeltaV_Losses to DeltaV_gain - DeltaV_total.
	local DeltaV_Eff to 100*DeltaV_Gain/DeltaV_total.
	
	set thrust_accel_1 to thrust_accel_2.
	set a_vec1 to a_vec2.
	set time1 to time2.
	
	// Delta V Printout
	set line to line + 3.
	print "DeltaV_total  = " + round(DeltaV_total) + "   " at(0,line).
	set line to line + 1.
	print "DeltaV_gain   = " + round(DeltaV_gain) + "   " at(0,line).
	set line to line + 1.
	print "DeltaV_Losses = " + round(DeltaV_Losses) + "   " at(0,line).
	set line to line + 1.
	print "DeltaV_Eff    = " + round(DeltaV_Eff) + "%   " at(0,line).
	
	wait 0.
}

local DV_Circ to Circularize_DV_Calc().
set line to line + 3.
print "Total Delta V for Circularization " + round(DeltaV_total + DV_Circ) + "    " at(0,line).