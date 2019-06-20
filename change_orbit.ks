RUNONCEPATH("Library/lib_execnode.ks").

function Change_LAN_Inc {

	parameter DesiredOrbit.
	local body_pos to ship:body:position.
	local INC_ship to ship:orbit:inclination.
	local R to -body_pos.
	local SMA_ship to ship:orbit:semimajoraxis.
	local LAN_des to DesiredOrbit["LAN"].
	local LAN_VEC to  solarprimevector*(SMA_ship)*R(0,-LAN_des,0).
	local Inc_Rotate to ANGLEAXIS(-1*DesiredOrbit["INC"],LAN_VEC).
	local Inc_Normal to Inc_Rotate*(V(0,-1,0):direction).
	local Inc_Normal to SMA_ship*Inc_Normal:vector.

	local AngVel_ship to SMA_ship*VCRS(R,ship:velocity:orbit):normalized.

	local LAN_relative_vec to SMA_ship*VCRS(AngVel_ship,Inc_Normal):normalized.

	local LAN_relative_theta to FindTheta_Vec(LAN_relative_vec).
	local LAN_eta to ETA_to_theta(LAN_relative_theta).

	local delta_inc to VANG(AngVel_ship,Inc_Normal).
	local Vel_at_LAN to velocityat(ship,time:seconds + LAN_eta):orbit.
	local temp_dir to Vel_at_LAN:direction.
	local rotate_dir to ANGLEAXIS(delta_inc,LAN_relative_vec).
	local vel_rotated to rotate_dir*temp_dir.
	local New_Vel_at_LAN to (Vel_at_LAN:mag)*vel_rotated:vector.

	local LAN_node to SetNode_BurnVector(time:seconds + LAN_eta,New_Vel_at_LAN).
	add LAN_node.

	// Debugging Vecdraws
	//set LAN_VEC_Draw to vecdraw().
	//set LAN_VEC_Draw:startupdater to { return ship:body:position. }.
	//set LAN_VEC_Draw:vecupdater to { return LAN_VEC. }.
	//set LAN_VEC_Draw:show to true.
	//set LAN_VEC_Draw:color to RGB(255,0,0).

	//set INC_VEC_Draw to vecdraw().
	//set INC_VEC_Draw:startupdater to { return ship:body:position. }.
	//set INC_VEC_Draw:vecupdater to { return Inc_Normal. }.
	//set INC_VEC_Draw:show to true.
	//set INC_VEC_Draw:color to RGB(0,255,0).

	//set ANG_VEC_Draw to vecdraw().
	//set ANG_VEC_Draw:startupdater to { return ship:body:position. }.
	//set ANG_VEC_Draw:vecupdater to { return AngVel_ship. }.
	//set ANG_VEC_Draw:show to true.
	//set ANG_VEC_Draw:color to RGB(0,0,255).

	//set Rel_LAN_VEC_Draw to vecdraw().
	//set Rel_LAN_VEC_Draw:startupdater to { return ship:body:position. }.
	//set Rel_LAN_VEC_Draw:vecupdater to { return LAN_relative_vec. }.
	//set Rel_LAN_VEC_Draw:show to true.
	//set Rel_LAN_VEC_Draw:color to RGB(255,255,0).

	//set LAN_VEL_VEC_Draw to vecdraw().
	//set LAN_VEL_VEC_Draw:startupdater to { return V(0,0,0). }.
	//set LAN_VEL_VEC_Draw:vecupdater to { return Vel_at_LAN/50. }.
	//set LAN_VEL_VEC_Draw:show to true.
	//set LAN_VEL_VEC_Draw:color to RGB(255,0,0).

	//set LAN_VEL_VEC_Draw2 to vecdraw().
	//set LAN_VEL_VEC_Draw2:startupdater to { return V(0,0,0). }.
	//set LAN_VEL_VEC_Draw2:vecupdater to { return New_Vel_at_LAN/50. }.
	//set LAN_VEL_VEC_Draw2:show to true.
	//set LAN_VEL_VEC_Draw2:color to RGB(0,255,0).

	//wait 1.

	if nextnode:burnvector:mag > 0.05 {
    ExecuteNode().
	} else {
    remove nextnode.
    wait 0.
  }
}

function Change_AoP_PerApo {

	parameter DesiredOrbit.
	local body_pos to ship:body:position.
	local INC_ship to ship:orbit:inclination.
	local R to -body_pos.
	local SMA_ship to ship:orbit:semimajoraxis.
	local LAN_ship to ship:orbit:LAN.
	local LAN_VEC to  solarprimevector*(SMA_ship)*R(0,-LAN_ship,0).
	local AngVel_ship to SMA_ship*VCRS(R,ship:velocity:orbit):normalized.
	local AOP_ship to ship:orbit:argumentofperiapsis.
	local AoP_Rotate to ANGLEAXIS(DesiredOrbit["AOP"],AngVel_ship).
	local AoP_VEC to AoP_Rotate*(LAN_VEC:direction).
	local AoP_VEC to SMA_ship*AoP_VEC:vector.

	local AoP_theta to FindTheta_Vec(AoP_VEC).
	local AoP_eta to ETA_to_theta(AoP_theta).
	local AoP_timeat to time:seconds + AoP_eta.

	Apoapsis_Set_TimeAt(AoP_timeat,DesiredOrbit).
	local New_Apo_time to time:seconds + eta:apoapsis.
	Periapsis_Set_TimeAt(New_Apo_time,DesiredOrbit).
	// Debugging Vecdraws
	//set LAN_VEC_Draw to vecdraw().
	//set LAN_VEC_Draw:startupdater to { return ship:body:position. }.
	//set LAN_VEC_Draw:vecupdater to { return LAN_VEC. }.
	//set LAN_VEC_Draw:show to true.
	//set LAN_VEC_Draw:color to RGB(255,0,0).
	//
	//set AoP_VEC_Draw to vecdraw().
	//set AoP_VEC_Draw:startupdater to { return ship:body:position. }.
	//set AoP_VEC_Draw:vecupdater to { return AoP_VEC. }.
	//set AoP_VEC_Draw:show to true.
	//set AoP_VEC_Draw:color to RGB(0,255,0).

	wait 1.
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

function Apoapsis_Set_TimeAt {
	parameter AoP_timeat, DesiredOrbit.

	local body_pos to ship:body:position.
	local body_radius to ship:body:radius.
	local R to -body_pos.
	local SMA_ship to ship:orbit:semimajoraxis.
	local AngVel_ship to SMA_ship*VCRS(R,ship:velocity:orbit):normalized.
	local R_ap to body_radius + DesiredOrbit["APO"].
	local R_aop_vec to positionat(ship,AoP_timeat) - body_pos.
	local R_aop to R_aop_vec:mag.
	local R_ap_vec to -1*R_ap*R_aop_vec:normalized.

	local SMA_new to (R_ap + R_aop)/2.
	local V_aop_speed to vis_via_speed(R_aop-body_radius,SMA_new).
	local temp_vec to VCRS(AngVel_ship,R_aop_vec):normalized.
	local V_aop_new_vec to V_aop_speed*temp_vec.

	local APO_node to SetNode_BurnVector(AoP_timeat,V_aop_new_vec).
	add APO_node.

	local delta_v_current to APO_node:burnvector:mag.
	local max_acc to availablethrust/mass.
	local burn_time to delta_v_current/max_acc.

	if nextnode:burnvector:mag > 0.05 {
    ExecuteNode().
	} else {
    remove nextnode.
    wait 0.
  }
	// Debugging Vecdraws
	//set R_VEC_Draw to vecdraw().
	//set R_VEC_Draw:startupdater to { return ship:body:position. }.
	//set R_VEC_Draw:vecupdater to { return R_ap_vec. }.
	//set R_VEC_Draw:show to true.
	//set R_VEC_Draw:color to RGB(0,0,255).

	//wait 1.
}

function Periapsis_Set_TimeAt {
	parameter New_Apo_time, DesiredOrbit.

	local body_pos to ship:body:position.
	local R to -body_pos.
	local body_radius to ship:body:radius.
	local R_per_new to DesiredOrbit["PER"] + body_radius.
	local R_ap_new to positionat(ship,New_Apo_time) + R.
	local SMA_new to (R_ap_new:mag + R_per_new)/2.
	local V_ap_new_speed to vis_via_speed(R_ap_new:mag-body_radius,SMA_new).
	local V_ap_current_speed to velocityat(ship,New_Apo_time):orbit:mag.

	local delta_v_node to V_ap_new_speed - V_ap_current_speed.
	local PER_node to node(New_Apo_time,0,0,delta_v_node).
	add PER_node.

	if nextnode:burnvector:mag > 0.05 {
    ExecuteNode().
	} else {
    remove nextnode.
    wait 0.
  }
}

function vis_via_speed {
	parameter R, a is ship:orbit:semimajoraxis.
	local R_val to ship:body:radius + R.
	return sqrt(ship:body:mu*(2/R_val - 1/a)).
}

function SetNode_BurnVector {
	parameter timeat,V_New.

	local V_timeat to velocityat(ship,timeat):orbit.

	local node_normal_vec to vcrs(ship:body:position,ship:velocity:orbit):normalized.
	local node_prograde_vec to V_timeat:normalized.
	local node_radial_vec to VCRS(node_normal_vec,node_prograde_vec).

	local burn_vector to (V_New - V_timeat).
	local burn_prograde to VDOT(node_prograde_vec,burn_vector).
	local burn_normal to VDOT(node_normal_vec,burn_vector).
	local burn_radial to VDOT(node_radial_vec,burn_vector).

	return NODE(timeat,burn_radial,burn_normal,burn_prograde).
}

local LAN_ship to ship:orbit:LAN.
local INC_ship to ship:orbit:inclination.
local AOP_ship to ship:orbit:argumentofperiapsis.
local PER_ship to ship:orbit:periapsis.
local APO_ship to ship:orbit:apoapsis.
local default_DesiredOrbit to lexicon("LAN",LAN_ship,"INC",INC_ship,"AOP",AOP_ship,"PER",PER_ship,"APO",APO_ship).

parameter DesiredOrbit is lexicon("LAN",LAN_ship,"INC",INC_ship,"AOP",AOP_ship,"PER",PER_ship,"APO",APO_ship).

for key in default_DesiredOrbit:keys {
    if not DesiredOrbit:haskey(key) { set DesiredOrbit[key] to default_DesiredOrbit[key]. }
}
clearscreen.
print "Changing Orbit to:".
print DesiredOrbit.
wait 2.

clearscreen.
clearvecdraws().
for n in allnodes { remove n.}
local tolerance_angle to 0.01.
local LAN_diff to abs(DesiredOrbit["LAN"] - LAN_ship).
local INC_diff to abs(DesiredOrbit["INC"] - INC_ship).

if  not(LAN_diff < tolerance_angle AND INC_diff < tolerance_angle) {
  Change_LAN_Inc(DesiredOrbit).
}
local AOP_ship to ship:orbit:argumentofperiapsis.
local PER_ship to ship:orbit:periapsis.
local APO_ship to ship:orbit:apoapsis.

local AOP_diff to abs(AOP_ship - DesiredOrbit["AOP"]).
local APO_diff to abs(APO_ship - DesiredOrbit["APO"]).
local PER_diff to abs(PER_ship - DesiredOrbit["PER"]).

local AOP_diff_percent to 100*(1 - AOP_diff/DesiredOrbit["AOP"]).
local APO_diff_percent to 100*(APO_diff/DesiredOrbit["APO"]).
local PER_diff_percent to 100*(PER_diff/DesiredOrbit["PER"]).

local tolerance_percent to 0.05.

if not(AOP_diff_percent < tolerance_percent AND APO_diff_percent < tolerance_percent AND PER_diff_percent < tolerance_percent) {
	Change_AoP_PerApo(DesiredOrbit).
}
clearvecdraws().