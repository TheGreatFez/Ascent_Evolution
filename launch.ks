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

until apoapsis > TargetOrbit {
	if pitch > 0 {
		if altitude > switch_alt {
			set switch_alt to switch_alt + 5000.
			set pitch to pitch - 15.
		}
	}
	print "Pitch = " + pitch + "   " at(0,1).
}