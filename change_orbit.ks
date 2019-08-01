RUNONCEPATH("Library/lib_execnode.ks").
RUNONCEPATH("Library/lib_OrbitalMechanics_Functions.ks").
RUNONCEPATH("Library/lib_SetManNode_Velocity.ks").

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

	AoP_Set_TimeAt(AoP_timeat,DesiredOrbit).
  ApoPeri_Set(DesiredOrbit).
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

	wait 0.
}

function AoP_Set_TimeAt {
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
	local V_aop_speed to vis_via_speed(R_aop,SMA_new).
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

function ApoPeri_Set {
	parameter DesiredOrbit.
  local burn_at_apo is true.
  local New_Apo_time to time:seconds + eta:apoapsis.
  if ship:orbit:trueanomaly > 90 AND ship:orbit:trueanomaly < 270 {
    set New_Apo_time to time:seconds + eta:periapsis.
    set burn_at_apo to false.
  }

	local body_pos to ship:body:position.
	local body_radius to ship:body:radius.
	local R_per_new to DesiredOrbit["PER"] + body_radius.
	local R_apo_new to DesiredOrbit["APO"] + body_radius.
  local R_per_old to periapsis + body_radius.
	local R_apo_old to apoapsis + body_radius.
  local eta_to_apo to time:seconds + eta:apoapsis.
	local V_apo_current_speed to velocityat(ship,eta_to_apo):orbit:mag.
  local eta_to_per to time:seconds + eta:periapsis.
	local V_per_current_speed to velocityat(ship,eta_to_per):orbit:mag.

  local Burn_Node_time is time:seconds.
  local delta_v_node to 0.
  if burn_at_apo {
    local SMA_new to (R_apo_old + R_per_new)/2.
    local V_apo_new_speed to vis_via_speed(R_apo_old,SMA_new).
    set delta_v_node to V_apo_new_speed - V_apo_current_speed.
    set Burn_Node_time to eta_to_apo.
  } else {
    local SMA_new to (R_apo_new + R_per_old)/2.
    local V_per_new_speed to vis_via_speed(R_per_old,SMA_new).
    set delta_v_node to V_per_new_speed - V_per_current_speed.
    set Burn_Node_time to eta_to_per.
  }

	local PER_node to node(Burn_Node_time,0,0,delta_v_node).
	add PER_node.

	if nextnode:burnvector:mag > 0.05 {
    ExecuteNode().
	} else {
    remove nextnode.
    wait 0.
  }
}

// Main Loop
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
Change_LAN_Inc(DesiredOrbit).
Change_AoP_PerApo(DesiredOrbit).
clearvecdraws().
