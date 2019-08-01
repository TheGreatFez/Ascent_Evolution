@LAZYGLOBAL OFF.
// All radii are absolute, not altitude above body surface
function vis_via_speed {
	parameter R_input, a is ship:orbit:semimajoraxis.

	local V_output is sqrt(ship:body:mu*(2/R_input - 1/a)).

  return V_output.
}

function gamma_from_R {
  parameter R_input, a is ship:orbit:semimajoraxis.
  local ship_AngVel is 0.
  if HASNODE {
    set a to nextnode:orbit:semimajoraxis.
    local node_time to nextnode:eta + 1 + time:seconds.
    set ship_AngVel to VCRS(velocityat(ship,node_time):orbit,positionat(ship,node_time)-ship:body:position):mag.
  } else {
    set ship_AngVel to VCRS(ship:velocity:orbit,-ship:body:position):mag.
  }
  local V_input is vis_via_speed(R_input, a).
  local gamma is arccos(ship_AngVel/(V_input*R_input)).

  return gamma.
}

function FindTheta_Vec {

	parameter test_vector is -ship:body:position.

	local body_pos to ship:body:position.
	local R to -body_pos.
	local AngVel_ship to VCRS(R,ship:velocity:orbit):normalized.
	local theta_test to VANG(test_vector,R).
	local cross_test to VCRS(R,test_vector):normalized.

	local check_vec to cross_test + AngVel_ship.
	local theta_ship is ship:orbit:trueanomaly.
	local theta is theta_ship.

	if check_vec:mag > 1 {
		set theta to theta_ship + theta_test.
	} else {
		set theta to theta_ship - theta_test.
	}

	if theta < 0 {
		set theta to 360 + theta.
	}

	if theta > 360 {
		set theta to theta - 360.
	}

	return theta.
}

function ETA_to_theta {

	parameter theta_test.

	local T_orbit to ship:orbit:period.
	local theta_ship to ship:orbit:trueanomaly.
	local e to ship:orbit:eccentricity.
	local GM to ship:body:mu.
	local a to ship:orbit:semimajoraxis.

	local EA_ship to 2*ARCTAN((TAN(theta_ship/2))/sqrt((1+e)/(1-e))).
	local MA_ship to EA_ship*constant:pi/180 - e*SIN(EA_ship).
	local EA_test to 2*ARCTAN((TAN(theta_test/2))/sqrt((1+e)/(1-e))).
	local MA_test to EA_test*constant:pi/180 - e*SIN(EA_test).
	local n to sqrt(GM/(a)^3).
	local eta_to_testpoint to (MA_test - MA_ship)/n.
	if eta_to_testpoint < 0 {
		set eta_to_testpoint to T_orbit + eta_to_testpoint.
	}

	return eta_to_testpoint.
}
