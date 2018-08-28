// *********************************************** //
//    Ascent Program Evolution: Gravity Turn 1.0   //
// *********************************************** //

// Clean up work area
clearscreen.
set ship:control:pilotmainthrottle to 0.

// Main Inputs
local TargetOrbit is 100000. // The target altitude of our parking orbit.

// Ignition
local pitch to 90.
lock throttle to 1.
lock steering to heading(90,pitch).
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
local pitch_over_ang is 3.
local pitch_hold_time is 2.
local gravity_turn is false.

until apoapsis > TargetOrbit {
	if altitude > switch_alt AND not(gravity_turn) {
		set pitch to pitch - pitch_over_ang.
		wait pitch_hold_time.
		lock steering to srfprograde.
		set gravity_turn to true.
	}
	local line is 1.
	print "Pitch        = " + pitch + "   " at(0,line).
	set line to line + 1.
	print "gravity_turn = " + gravity_turn + "   " at(0,line).
	set line to line + 1.
	print "altitude     = " + round(altitude,2) + "   " at(0,line).
	set line to line + 1.
	print "switch_alt   = " + switch_alt + "   " at(0,line).
	set line to line + 1.
	print "apoapsis     = " + round(apoapsis,2) + "   " at(0,line).
	set line to line + 1.
	print "TargetOrbit  = " + TargetOrbit + "   " at(0,line).
}