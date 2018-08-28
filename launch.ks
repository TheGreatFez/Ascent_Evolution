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
	parameter AscentStage, switch_alt is 250, pitch_over_ang is 3.
	local pitch_ang to 90.
	if AscentStage = 2 {
		set pitch_ang to 90 - pitch_over_ang.
	} else if AscentStage = 3{
		set pitch_ang to 90 - VANG(UP:vector,srfprograde:vector).
	}
	
	return pitch_ang.
}

// Ignition
local pitch_ang to 90.
lock throttle to 1.
lock steering to heading(90,pitch_ang).
stage.

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
local switch_alt is 250.
local pitch_over_ang is 2.

// Run Mode Variables
local AscentStage is 1.
local ThrottleStage is 1.

until apoapsis > TargetOrbit AND altitude > ship:body:ATM:height {
	if altitude > switch_alt AND AscentStage = 1 {
		set AscentStage to 2.
	}
	if AscentStage = 2 {
		local AngCheck to VANG(srfprograde:vector,steering:vector).
		if AngCheck <= 0.1 {
			set AscentStage to 3.
		}
	}
	if apoapsis > TargetOrbit AND ThrottleStage = 1 {
		lock throttle to 0.
		set ThrottleStage to 2.
	}
	
	set pitch_ang to PitchProgram(AscentStage,switch_alt,pitch_over_ang).
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
}