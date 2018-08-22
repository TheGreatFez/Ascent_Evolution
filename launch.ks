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
print "Pitch is now " + pitch.

// Basic Staging:
local MAX to maxthrust.
when maxthrust < MAX OR availablethrust = 0 then {
	lock throttle to 0.
	stage.
	lock throttle to 1.
	set MAX to maxthrust.
	preserve.
}

// Pitch Steps
wait until altitude > 100.
set pitch to 75.
print "Pitch is now " + pitch.

wait until altitude > 10000.
set pitch to 60.
print "Pitch is now " + pitch.

wait until altitude > 15000.
set pitch to 45.
print "Pitch is now " + pitch.

wait until altitude > 20000.
set pitch to 30.
print "Pitch is now " + pitch.

wait until altitude > 25000.
set pitch to 15.
print "Pitch is now " + pitch.

wait until altitude > 30000.
set pitch to 0.
print "Pitch is now " + pitch.

wait until apoapsis >= TargetOrbit.