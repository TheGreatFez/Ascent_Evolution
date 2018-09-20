// *********************************************** //
//    Ascent Program Evolution: Gravity Turn 2.0   //
// *********************************************** //

// Clean up work area
clearscreen.
set ship:control:pilotmainthrottle to 0.

// Main Inputs
local TargetOrbit is 100000. // The target altitude of our parking orbit.

// Functions

function PitchProgram {
	parameter AscentStage, switch_alt is 250.
	local pitch_ang to 90.
	local scale_factor to 0.75.
	local alt_diff is scale_factor*ship:body:atm:height - switch_alt.
	
	local SrfProg_pitch to VANG(UP:vector,srfprograde:vector).
	if AscentStage = 2 {
		set pitch_ang to 90 - max(0,min(90,90*sqrt((altitude - switch_alt)/alt_diff))).
	} else if AscentStage = 3 {
		set pitch_ang to 90 - SrfProg_pitch. 
	}
	
	return pitch_ang.
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
local grav_loss is 0.
local turn_loss is 0.
local DeltaV_gain is 0.
local obt_vec1 is ship:velocity:orbit.
local accel_1 is 0.
local obt_vec2 is ship:velocity:orbit.
local accel_2 is 0.
local time1 is 0.
local time2 is 0.
local dt is 0.00001.
local g is body:mu/(body:position:mag^2).
local g_vec to UP:vector.


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
	
	// Delta V Calculations Step 1
	set accel_1 to throttle*availablethrust/mass.
	set obt_vec1 to ship:velocity:orbit.
	set time1 to time:seconds.
	
	// Variable Printout
	local line is 1.
	print "ThrottleStage = " + ThrottleStage + "   " at(0,line).
	set line to line + 1.
	print "AscentStage   = " + AscentStage + "   " at(0,line).
	set line to line + 1.
	print "pitch_ang     = " + round(pitch_ang) + "   " at(0,line).
	set line to line + 1.
	print "altitude      = " + round(altitude) + "   " at(0,line).
	set line to line + 1.
	print "switch_alt    = " + switch_alt + "   " at(0,line).
	set line to line + 1.
	print "apoapsis      = " + round(apoapsis) + "   " at(0,line).
	set line to line + 1.
	print "TargetOrbit   = " + TargetOrbit + "   " at(0,line).
	wait 0.
	
	// Delta V Calculations Step 2
	set accel_2 to throttle*availablethrust/mass.
	set obt_vec2 to ship:velocity:orbit.
	set time2 to time:seconds.
	set dt to time2 - time1.
	set accel to (accel_1 + accel_2)/2.
	set DeltaV_total to DeltaV_total + accel*dt.
	set accel_vec to (obt_vec2 - obt_vec1)/dt.
	set g to body:mu/(body:position:mag^2).
	set g_vec to -UP:vector*g.
	set thrust_vec to accel*ship:facing:vector.
	set obt_vel_norm to obt_vec2:normalized.
	set grav_loss to grav_loss + dt*(VDOT(obt_vel_norm,g_vec))*throttle.
	local turn_vec to VCRS(VCRS(obt_vel_norm,ship:facing:vector),obt_vel_norm):normalized.
	set turn_loss to turn_loss - dt*(VDOT(turn_vec,thrust_vec + g_vec)).
	set DeltaV_gain to DeltaV_gain + dt*(VDOT(obt_vel_norm,accel_vec)).
	set aero_loss to -1*(DeltaV_total - DeltaV_gain + turn_loss + grav_loss).	
	
	// Delta V Printout
	set line to line + 3.
	print "DeltaV_gain   = " + round(DeltaV_gain) + "   " at(0,line).
	set line to line + 1.
	print "turn_loss     = " + round(turn_loss) + "   " at(0,line).
	set line to line + 1.
	print "aero_loss     = " + round(aero_loss) + "   " at(0,line).
	set line to line + 1.
	print "grav_loss     = " + round(grav_loss) + "   " at(0,line).
	set line to line + 1.
	print "DeltaV_total  = " + round(DeltaV_total) + "   " at(0,line).
	set line to line + 1.
	print "g             = " + round(g,2) + "   " at(0,line).
	set line to line + 1.
	print "acceleration  = " + round(accel,2) + "   " at(0,line).
	set line to line + 1.
	print "dt            = " + round(dt,4) + "   " at(0,line).
	
}