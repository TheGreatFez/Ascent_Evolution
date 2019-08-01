// ASSUME EQUATORIAL CIRCULAR ORBIT
@LAZYGLOBAL OFF.
parameter landing_pos is LATLNG(12.69,-47.14), landing_peri is 0.05*ship:body:radius.
clearscreen.
for n in ALLNODES { remove n.}
clearvecdraws().

RUNONCEPATH("Library/lib_BisectionSolver.ks").
RUNONCEPATH("Library/lib_OrbitalMechanics_Functions.ks").

function ETA_to_theta {

	parameter theta_test.

	local orbit_test to ship:orbit.
	local mnv_time to 0.

	if HASNODE {
		set orbit_test to nextnode:orbit.
		set mnv_time to nextnode:eta.
	}

	local T_orbit to orbit_test:period.
	local theta_ship to orbit_test:trueanomaly.
	local e to orbit_test:eccentricity.
	local GM to ship:body:mu.
	local a to orbit_test:semimajoraxis.
	//clearscreen.

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
	return eta_to_testpoint + mnv_time.
}

function Vec_To_Node {
	parameter des_vec, mnv_time.

	local vel_vec to velocityat(ship,time:seconds + mnv_time):orbit.

	local norm_vec to vcrs(ship:body:position,ship:velocity:orbit):normalized.
	local prog_vec to vel_vec:normalized.
	local radi_vec to VCRS(norm_vec,prog_vec).

	local burn_vec to des_vec - vel_vec.

	local norm_comp to VDOT(norm_vec,burn_vec).
	local prog_comp to VDOT(prog_vec,burn_vec).
	local radi_comp to VDOT(radi_vec,burn_vec).

	local mynode to NODE(time:seconds + mnv_time,radi_comp,norm_comp,prog_comp).
	add mynode.
}

function Incline_Orbit {
  parameter inc_angle, mnv_time.

	local vel_vec to velocityat(ship,time:seconds + mnv_time):orbit.
	local pos_vec to positionat(ship,time:seconds + mnv_time).
	local body_vec to pos_vec - ship:body:position.

	local angle_rotate_inc to ANGLEAXIS(inc_angle,-body_vec).
	local new_vel_vec to vel_vec*angle_rotate_inc.

  Vec_To_Node(new_vel_vec,mnv_time).
}

function Set_Landing_Orbit_vec {
  parameter landing_pos, mnv_time.

  Incline_Orbit(-landing_pos:LAT,mnv_time).

  local trueanomaly_desired to nextnode:orbit:trueanomaly + 90.
  local deorbit_node_time to ETA_to_theta(trueanomaly_desired).
  local deorbit_node_position to positionat(ship,time:seconds + deorbit_node_time) - ship:body:position.
  local deorbit_node_velocity to velocityat(ship,time:seconds + deorbit_node_time):orbit.
  local vel_norm_deorbit to VCRS(VCRS(deorbit_node_position,deorbit_node_velocity),deorbit_node_position):normalized.
  local new_sma to (deorbit_node_position:mag + landing_peri + ship:body:radius)/2.
  local new_deorbit_vel to vis_via_speed(deorbit_node_position:mag,new_sma)*vel_norm_deorbit.
  Vec_To_Node(new_deorbit_vel,deorbit_node_time).
  //print new_sma.
  //print landing_peri.
  //print deorbit_node_position:mag.
  //print new_deorbit_vel:mag.

  //wait until false.
  local new_deorbit_timeperiod to 2*constant:pi*sqrt((new_sma^3)/ship:body:mu).
  local total_traverse_time to 0.5*new_deorbit_timeperiod + deorbit_node_time.

  local peri_pos to positionat(ship,time:seconds + total_traverse_time).
  local long_offset to 360*(total_traverse_time)/ship:body:rotationperiod.
  //print "Long Offset = " + round(long_offset,2).
  local peri_ltlng_pre to ship:body:GEOPOSITIONOF(peri_pos).
  local peri_ltlng to LATLNG(peri_ltlng_pre:LAT, peri_ltlng_pre:LNG - long_offset).
  local long_diff to landing_pos:LNG - peri_ltlng:LNG.

  return peri_ltlng:LNG.
}

function makeLandingSpot_Score {
  parameter landing_pos.

  return {
    parameter mnv_time.
    for n in allnodes {remove n.}
    wait 0.
    Incline_Orbit(-landing_pos:LAT,mnv_time).

    local trueanomaly_desired to nextnode:orbit:trueanomaly + 90.
    local deorbit_node_time to ETA_to_theta(trueanomaly_desired).
    local deorbit_node_position to positionat(ship,time:seconds + deorbit_node_time) - ship:body:position.
    local deorbit_node_velocity to velocityat(ship,time:seconds + deorbit_node_time):orbit.
    local vel_norm_deorbit to VCRS(VCRS(deorbit_node_position,deorbit_node_velocity),deorbit_node_position):normalized.
    local new_sma to (deorbit_node_position:mag + landing_peri + ship:body:radius)/2.
    local new_deorbit_vel to vis_via_speed(deorbit_node_position:mag,new_sma)*vel_norm_deorbit.
    Vec_To_Node(new_deorbit_vel,deorbit_node_time).
    //print new_sma.
    //print landing_peri.
    //print deorbit_node_position:mag.
    //print new_deorbit_vel:mag.

    //wait until false.
    local new_deorbit_timeperiod to 2*constant:pi*sqrt((new_sma^3)/ship:body:mu).
    local total_traverse_time to 0.5*new_deorbit_timeperiod + deorbit_node_time.

    local peri_pos to positionat(ship,time:seconds + total_traverse_time).
    local long_offset to 360*(total_traverse_time)/ship:body:rotationperiod.
    //print "Long Offset = " + round(long_offset,2).
    local peri_ltlng_pre to ship:body:GEOPOSITIONOF(peri_pos).
    local peri_ltlng to LATLNG(peri_ltlng_pre:LAT, peri_ltlng_pre:LNG - long_offset).
    local long_diff to landing_pos:LNG - peri_ltlng:LNG.

    return long_diff.
  }.
}

// Main Loop

local LandingSpot_Score to makeLandingSpot_Score(landing_pos).
local time_start is 100.
local lng_check is LandingSpot_Score(time_start).
local ready_to_start is false.
clearscreen.

until ready_to_start {
  local time_start2 is time_start + 100.
  local lng_check2 is LandingSpot_Score(time_start2).
  local line_count is 0.
  print "Time 1 : " + round(time_start) + "     " at(0,line_count).
  set line_count to line_count + 1.
  print "Lng 1  : " + round(lng_check,2) + "     " at(0,line_count).
  set line_count to line_count + 1.
  print "Time 2 : " + round(time_start2) + "     " at(0,line_count).
  set line_count to line_count + 1.
  print "Lng 2  : " + round(lng_check2,2) + "     " at(0,line_count).

  if not(lng_check > 0 AND lng_check2 > 0) {
    set time_start to time_start2.
    set lng_check to lng_check2.
  } else {
    set ready_to_start to true.
  }
  wait 1.
}

local iteration to 0.
local LandingSpot_Solver to makeBiSectSolver(LandingSpot_Score,time_start,time_start + 100).
local MnvTimes to LandingSpot_Solver().
clearscreen.

until abs(MnvTimes[0][0] - MnvTimes[1][0]) < 1 OR iteration > 100 {
  local MnvTimes to LandingSpot_Solver().
  local line_count is 0.
  print "Time 1 : " + round(MnvTimes[0][0],1) + "     " at(0,line_count).
  set line_count to line_count + 1.
  print "Time 2 : " + round(MnvTimes[1][0],1) + "     " at(0,line_count).
  set line_count to line_count + 1.
  print "Time 3 : " + round(MnvTimes[2][0],1) + "     " at(0,line_count).
  set line_count to line_count + 1.
  print "Score  : " + round(MnvTimes[2][1],1) + "     " at(0,line_count).
  set iteration to iteration + 1.
  wait 1.
}
