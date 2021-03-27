// Script to do a spot landing on equatorial trajectory.
@LAZYGLOBAL OFF.
parameter landing_pos is LATLNG(12.69,-47.14).
//set landing_pos to LATLNG(-0.09720767,-74.557677).

clearscreen.
clearvecdraws().

SAS OFF.
if HASNODE {
  RUNONCEPATH("Library/lib_execnode.ks").
	for node in ALLNODES { ExecuteNode(). }
  local max_acc to maxthrust/mass.
  local peri_v to velocityat(ship,time:seconds + eta:periapsis):orbit:mag.
  local delta_time to peri_v/max_acc.
  warpto(time:seconds + eta:periapsis - 1.5*delta_time).
  print "Warping closer to Periapsis".
  wait until warp <= 0.
  clearscreen.
  KUniverse:quicksaveto("Spot_Land Test").
}
GEAR ON.



function true_alt {
  return altitude - ship:geoposition:terrainheight.
}

function g {
  local R to ship:body:position:mag.
  local GM to ship:body:mu.

  return GM/(R)^2.
}

function Hysteresis {
	declare parameter input,prev_output, right_hand_limit, left_hand_limit,right_hand_output is true.
	local output to prev_output.
	if prev_output = right_hand_output {
		if input <= left_hand_limit {
			set output to not(right_hand_output).
		}
	} else {
		if input >= right_hand_limit {
			set output to right_hand_output.
		}
	}
	return output.
}

function Vmax_v {
	declare parameter buffer_terrain is 0, TouchDownSpeed is 1.
	local V to ship:velocity:orbit.
	local R to ship:body:position.
	local Vper to VDOT(VCRS(R,VCRS(V,R)):normalized,V).
	local AccelCent to (Vper^2)/R:mag.
	local MaxThrustAccUp to availablethrust/mass.
	local MaxAccUp to MaxThrustAccUp - g() + AccelCent.
	local FPAsurf to 90 - VANG(UP:vector,ship:velocity:surface).
	local Vmax to sqrt(MAX(0,2*(true_alt() - buffer_terrain)*MaxAccUp + TouchDownSpeed^2)).
	return -Vmax.
}

function V_accel_inertial {
  local V is ship:velocity:orbit.
  local R is -ship:body:position.
  local tmp_vec is VCRS(VCRS(R,V),R):normalized.
  local centri_acc to VDOT(tmp_vec,V)^2/R:mag.

  return centri_acc - g().
}

local function V_throttle {
  local Vmax is Vmax_v(-13,0).
  local error is Vmax - verticalspeed.
  local throttle_needed to 0.
  local max_acc is maxthrust/mass.

  local throttle_needed is 1 + (0.25*error - V_accel_inertial)/max_acc.

  return min(1,max(0,throttle_needed)).
}

function Vmax_h {
	declare parameter  buffer_dist is 1000.
	local R to ship:body:position.
	local V to ship:velocity:orbit.
	local MaxThrustAccHor to availablethrust/mass.
	local angle_diff_h to VANG(-R, landing_pos:position - R).
	local dist_diff_h to (angle_diff_h/360)*2*(constant:pi)*R:mag.
	local Vmax to sqrt(MAX(0.001,2*(dist_diff_h - buffer_dist)*MaxThrustAccHor)).

	local dir_check_vel to VCRS(V,R).
	local dir_check_pos to VCRS(-R,landing_pos:position-R).
	local dir_check to 1.
	if VDOT(dir_check_vel,dir_check_pos) > 0 {
		set dir_check to 1.
	} else {
		set dir_check to -1.
	}

	return dir_check*Vmax.
}

local function PN_InterceptCorrection {
  parameter target_IntVec, target_pos.

  local R1 to target_pos:position.
  local corrRotation to ROTATEFROMTO(target_IntVec,R1).

  return R1*corrRotation.
}

local function PN_throttle {
  parameter target_pos.

  local Gain to 7.5.

  local R1 to target_pos.
  local V1 to ship:velocity:surface.
  local PN_acc_vec_pre to Gain*VCRS(V1,VCRS(R1,V1)/VDOT(R1,R1)).
  local PN_acc_vec_norm to PN_acc_vec_pre:normalized.
  local Inertial_acc to V_accel_inertial*UP:vector.

  local PN_acc_vec to PN_acc_vec_pre - VDOT(Inertial_acc,PN_acc_vec_norm)*PN_acc_vec_norm..

  //- VDOT(Inertial_acc,PN_acc_vec_norm)*PN_acc_vec_norm.

  return (PN_acc_vec*mass)/availablethrust.
}

local function surface_normal {
    PARAMETER p1.
    LOCAL localBody IS p1:BODY.
    LOCAL basePos IS p1:POSITION.

    LOCAL upVec IS (basePos - localBody:POSITION):NORMALIZED.
    LOCAL northVec IS VXCL(upVec,localBody:GEOPOSITIONLATLNG(90,0):POSITION - basePos):NORMALIZED * 3.
    LOCAL sideVec IS VCRS(upVec,northVec):NORMALIZED * 3.//is east

    LOCAL aPos IS localBody:GEOPOSITIONOF(basePos - northVec + sideVec):POSITION - basePos.
    LOCAL bPos IS localBody:GEOPOSITIONOF(basePos - northVec - sideVec):POSITION - basePos.
    LOCAL cPos IS localBody:GEOPOSITIONOF(basePos + northVec):POSITION - basePos.
    RETURN VCRS((aPos - cPos),(bPos - cPos)):NORMALIZED.
}

local desiredInterceptVector to -surface_normal(landing_pos).
local NewMethod to true.

local PN_landing_pos to V(1,0,0).
if NewMethod {
  local PN_landing_pos to PN_InterceptCorrection(desiredInterceptVector,landing_pos).
} else {
  local PN_landing_pos to landing_pos.
}


local throttle_vec to UP:vector.
local throttle_var to throttle_vec:mag.

lock throttle to throttle_var.

local steering_var to LOOKDIRUP(throttle_vec,facing:topvector).
lock steering to steering_var.

clearscreen.

local touchdown_speed to -1.
local alt_cutoff to 100.

local throttle_hyst to false.
local throttle_hyst_UL to 15.
local throttle_hyst_LL to 1.

local V_throttle_val to 0.

local ang_hyst to false.
local ang_hyst_UL to 50.
local ang_hyst_LL to 10.

local left_over_flag to false.
local Follow_Mode to false.
local TouchDown_Mode to false.

local LandingVector to VECDRAW((alt:radar)*(landing_pos:position - ship:body:position):normalized,landing_pos:position,GREEN,"Landing Intercept Vector",0.5,TRUE,.5).
set LandingVector:vectorupdater to { return (altitude-landing_pos:terrainheight)*(desiredInterceptVector).}.
set LandingVector:startupdater to { return landing_pos:position.}.

local LandingPositionVector to VECDRAW(V(0,0,0),landing_pos:position,RED,"Landing Vector",1.0,TRUE,.5).
set LandingPositionVector:vectorupdater to { return landing_pos:position.}.
set LandingPositionVector:startupdater to { return V(0,0,0).}.

local SurfaceVelocity to VECDRAW(V(0,0,0),ship:velocity:surface,BLUE,"Surface Velocity",1.0,TRUE,.5).
set SurfaceVelocity:vectorupdater to { return ship:velocity:surface.}.
set SurfaceVelocity:startupdater to { return V(0,0,0).}.

local PN_Throttle_Vec to VECDRAW(V(0,0,0),10*PN_throttle(PN_landing_pos),RED,"PN",1.0,TRUE,.5).
set PN_Throttle_Vec:vectorupdater to { return 10*PN_throttle(PN_landing_pos).}.
set PN_Throttle_Vec:startupdater to { return V(0,0,0).}.

//local Throttle_Vec_draw to VECDRAW(V(0,0,0),10*throttle_vec,BLUE,"Throttle Vec",1.0,TRUE,.5).
//set Throttle_Vec_draw:vectorupdater to { return 10*throttle_vec.}.
//set Throttle_Vec_draw:startupdater to { return V(0,0,0).}.

local CorrectedTargetPos to VECDRAW(V(0,0,0),PN_InterceptCorrection(desiredInterceptVector,landing_pos),GREEN,"Corrected Target Pos",1.0,TRUE,.5).
set CorrectedTargetPos:vectorupdater to { return PN_InterceptCorrection(desiredInterceptVector,landing_pos). }.
set CorrectedTargetPos:startupdater to { return V(0,0,0).}.

local V_speed_check to true.
local left_over_flag to True.

// Main Loop
until ship:status = "LANDED" {

  if verticalspeed > 0 {
    set V_speed_check to false.
  } else {
    set V_speed_check to true.
  }

	if verticalspeed > touchdown_speed AND true_alt() < alt_cutoff AND not(TouchDown_Mode){
		set TouchDown_Mode to True.
	}

  local V_throttle_val to V_throttle().

	if TouchDown_Mode {
		set V_throttle_val to (1-(touchdown_speed-verticalspeed)/touchdown_speed)*mass*g()/availablethrust.
    lock steering to LOOKDIRUP(UP:vector,facing:topvector).
	} else {
    if not(V_speed_check) {
      set V_throttle_val to 0.
    }
	}

  if NewMethod {
    set PN_landing_pos to PN_InterceptCorrection(desiredInterceptVector,landing_pos).
  } else {
    set PN_landing_pos to landing_pos.
  }
  set throttle_vec to UP:vector*V_throttle_val + PN_throttle(PN_landing_pos).

	local throttle_hyst_test to throttle_vec:mag.
	local ang_diff to VANG(throttle_vec,ship:facing:vector).
	set throttle_hyst to Hysteresis(100*throttle_hyst_test,throttle_hyst, throttle_hyst_UL, throttle_hyst_LL).
	set ang_hyst to Hysteresis(ang_diff,ang_hyst,ang_hyst_UL,ang_hyst_LL,False).

	if throttle_hyst {
		if ang_hyst {
			set throttle_var to throttle_vec:mag.
			set steering_var to LOOKDIRUP(throttle_vec,facing:topvector).
		} else {
			set throttle_var to 0.
      set steering_var to LOOKDIRUP(throttle_vec,facing:topvector).
		}
	} else {
		set throttle_var to 0.
		set steering_var to LOOKDIRUP(srfretrograde:vector,facing:topvector).
	}

  local line_count to 0.
	print "V_throttle_val = " + round(100*(VDOT(UP:vector,throttle_vec)),1) + "%   " at(0,line_count).
  set line_count to line_count + 1.
	print "F_throttle_val = " +round(100*sqrt(max(0,throttle_vec:mag^2 - V_throttle_val^2)),1) + "%   " at(0,line_count).
  set line_count to line_count + 1.
	print "PN_throttle_val= " +round(100*PN_throttle(PN_landing_pos):mag) + "%   " at(0,line_count).
  set line_count to line_count + 1.
	print "Vmax_v         = " +round(Vmax_v,2) at(0,line_count).
  set line_count to line_count + 1.
	print "Speed_v        = " +round(verticalspeed,2) at(0,line_count).
  set line_count to line_count + 1.
	print "Vmax_h         = " +round(Vmax_h,2) at(0,line_count).
  set line_count to line_count + 1.
	print "Longitude      = " +round(ship:geoposition:lng,2) at(0,line_count).
  set line_count to line_count + 1.
	print "Throttle       = " + round(100*throttle,0) + "%   " at(0,line_count).
  set line_count to line_count + 1.
	print "throttle_hyst  = " + throttle_hyst + "   " at(0,line_count).
  set line_count to line_count + 1.
	print "left_over_flag = " + left_over_flag + "   " at(0,line_count).
  set line_count to line_count + 1.
	print "ang_diff       = " + round(ang_diff,1) + "   " at(0,line_count).
  set line_count to line_count + 1.
	print "ang_hyst       = " + ang_hyst + "   " at(0,line_count).
  set line_count to line_count + 1.
	print "groundspeed    = " + round(groundspeed,2) + "   " at(0,line_count).
  set line_count to line_count + 1.
	print "Follow_Mode    = " + Follow_Mode + "   " at(0,line_count).
  set line_count to line_count + 1.
	print "TouchDown_Mode     = " + TouchDown_Mode + "   " at(0,line_count).
  set line_count to line_count + 1.
	print "throttle_hyst_test = " + round(throttle_hyst_test,2) + "   " at(0,line_count).
  set line_count to line_count + 1.
  print "V_speed_check      = " + V_speed_check + "   " at(0,line_count).
  set line_count to line_count + 1.
  print "true_alt           = " + round(true_alt(),2) + "   " at(0,line_count).
  set line_count to line_count + 1.
  print "NewMethod          = " + NewMethod + "   " at(0,line_count).

	wait 0.
}
lock throttle to 0.
unlock steering.
SAS ON.
clearscreen.
print "Waiting for Settling".
local settled to False.
local t_settle_start to time:seconds.
local t_settle to 0.

local settle_time_min to 3.

until settled {
  if not(KUNIVERSE:CANQUICKSAVE) {
    local t_settle_start to time:seconds.
  }
  set t_settle to time:seconds - t_settle_start.

  if t_settle > settle_time_min AND KUNIVERSE:CANQUICKSAVE {
    set settled to true.
  }
  local line_count to 1.
	print "Settled Time Min is " + round(settle_time_min,1) + "sec   " at(0,line_count).
  set line_count to line_count + 1.
  print "Settled Time : " + round(t_settle,1) + "sec    " at(0,line_count).
}
