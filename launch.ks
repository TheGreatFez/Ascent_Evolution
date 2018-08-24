// ****************************************** //
//    Ascent Program Evolution: Version 1.0   |
// ****************************************** //

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
local MAX to maxthrust.
when maxthrust < MAX OR availablethrust = 0 then {
	lock throttle to 0.
	stage.
	lock throttle to 1.
	set MAX to maxthrust.
	preserve.
}

// Pitch Loop
local switch_alt is 5000.
local end_alt is 30000.

until apoapsis > TargetOrbit {
	set pitch to max(0,min(90,90 - (90/(end_alt - switch_alt))*(altitude - switch_alt))).
	print "Pitch = " + pitch + "   " at(0,1).
}