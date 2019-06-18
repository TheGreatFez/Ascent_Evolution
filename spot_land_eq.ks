// Script to do a spot landing on equatorial trajectory.
@LAZYGLOBAL OFF.
parameter landing_pos is LATLNG(12.69,-47.14).
//set landing_pos to LATLNG(-0.09720767,-74.557677).

clearscreen.
clearvecdraws().

SAS OFF.
if HASNODE {
  run lib_execnode.
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

function H_vec {
  local R to ship:body:position.
	local V to ship:velocity:surface.
  return VCRS(R,VCRS(V,R)):normalized.
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

function Speed_h {
  local R to ship:body:position.
  local V_surf to ship:velocity:surface.
  local Velocity_h_norm to VCRS(VCRS(R,landing_pos:position),R):normalized.
  local Speed_h_val to VDOT(Velocity_h_norm,ship:velocity:surface).

  return Speed_h_val.
}

function H_throttle {
  local R to ship:body:position.
  local V_surf to ship:velocity:surface.
  local Velocity_h_norm to VCRS(VCRS(R,landing_pos:position),R):normalized.
  local Speed_h to VDOT(Velocity_h_norm,ship:velocity:surface).
  local error to Vmax_h() - Speed_h.
  local max_acc is maxthrust/mass.

  local throttle_needed is 1 - (0.25*error)/max_acc.

  return min(1,max(0,throttle_needed)).
}

function PN_throttle {

  local Gain to 4.

  local R1 to landing_pos:position.
  local V1 to ship:velocity:surface.
  local PN_acc_vec_pre to Gain*VCRS(V1,VCRS(R1,V1)/VDOT(R1,R1)).
  local PN_acc_vec_norm to PN_acc_vec_pre:normalized.
  local Inertial_acc to V_accel_inertial*UP:vector.

  local PN_acc_vec to PN_acc_vec_pre - VDOT(Inertial_acc,PN_acc_vec_norm)*PN_acc_vec_norm..

  //- VDOT(Inertial_acc,PN_acc_vec_norm)*PN_acc_vec_norm.

  return (PN_acc_vec*mass)/availablethrust.
}

function Follow_throttle_func {
	local R to ship:body:position.
	local V to ship:velocity:surface.
	local V_ref to (V:mag)*(landing_pos:position:normalized).
	local h to altitude - landing_pos:terrainheight. // used to adjust the V_ref later
	local V_diff to V_ref - V.
	local throttle_sel to (V_diff*mass)/availablethrust.

	return throttle_sel.
}

function S_vec {
  local R is ship:body:position.
  local V is ship:velocity:surface.
  return -1*VCRS(V,R):normalized.
}

function S_throttle_func {
	declare parameter t_0 is 1.
	local R to ship:body:position.
	local V to ship:velocity:surface.
	local S to V:mag.
	local V_side to VCRS(V,R):normalized.
	local V_per to VCRS(R,V_side):normalized.
	local T_vec to VCRS(R,VCRS(landing_pos:position,R)):normalized.
	local delta_v to -1*VDOT(V_side,(T_vec*S - V_per*S)).

	return delta_v.
}

local S_throttle to S_throttle_func().

local throttle_vec to UP:vector*V_throttle() + H_vec()*H_throttle() + S_vec()*S_throttle.

lock steering to LOOKDIRUP(throttle_vec,facing:topvector).

lock land_surf to VANG(landing_pos:position,ship:velocity:surface).

clearscreen.

local touchdown_speed to -1.
local alt_cutoff to 100.

local throttle_hyst to false.
local throttle_hyst_UL to 5.
local throttle_hyst_LL to 1.

local H_throttle_val to 0.
local V_throttle_val to 0.

local ang_hyst to false.
local ang_hyst_UL to 50.
local ang_hyst_LL to 10.

local left_over_flag to false.
local Follow_Mode to false.
local TouchDown_Mode to false.

local LandingVector to VECDRAW((alt:radar)*(landing_pos:position - ship:body:position):normalized,landing_pos:position,GREEN,"Landing Position",1.0,TRUE,.5).
set LandingVector:vectorupdater to { return (altitude-landing_pos:terrainheight)*(landing_pos:position - ship:body:position):normalized.}.
set LandingVector:startupdater to { return landing_pos:position.}.

local LandingPositionVector to VECDRAW(V(0,0,0),landing_pos:position,RED,"Landing Vector",1.0,TRUE,.5).
set LandingPositionVector:vectorupdater to { return landing_pos:position.}.
set LandingPositionVector:startupdater to { return V(0,0,0).}.

local SurfaceVelocity to VECDRAW(V(0,0,0),ship:velocity:surface,BLUE,"Surface Velocity",1.0,TRUE,.5).
set SurfaceVelocity:vectorupdater to { return ship:velocity:surface.}.
set SurfaceVelocity:startupdater to { return V(0,0,0).}.

local PN_Throttle_Vec to VECDRAW(V(0,0,0),10*PN_throttle(),RED,"PN",1.0,TRUE,.5).
set PN_Throttle_Vec:vectorupdater to { return 10*PN_throttle().}.
set PN_Throttle_Vec:startupdater to { return V(0,0,0).}.

local Throttle_Vec_draw to VECDRAW(V(0,0,0),10*throttle_vec,BLUE,"Throttle Vec",1.0,TRUE,.5).
set Throttle_Vec_draw:vectorupdater to { return 10*throttle_vec.}.
set Throttle_Vec_draw:startupdater to { return V(0,0,0).}.

local Inertial_Acc_Vec to VECDRAW(V(0,0,0),10*V_accel_inertial()*UP:vector,GREEN,"Inertial Vec",1.0,TRUE,.5).
set Inertial_Acc_Vec:vectorupdater to { return 10*V_accel_inertial()*UP:vector.}.
set Inertial_Acc_Vec:startupdater to { return V(0,0,0).}.

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
  local H_throttle_test to H_throttle().

	if TouchDown_Mode {
		set V_throttle_val to (1-(touchdown_speed-verticalspeed)/touchdown_speed)*mass*g()/availablethrust.
    lock steering to LOOKDIRUP(UP:vector,facing:topvector).
	} else {
    if not(V_speed_check) {
      set V_throttle_val to 0.
    }
	}

	local S_deltaV to S_throttle_func(2).
	local S_throttle_enable to true.
	local S_throttle_test to (S_deltaV*mass)/(availablethrust*1).

	if sqrt(V_throttle_val^2 + H_throttle_test^2 + S_throttle_test^2) > 1 {
		set left_over_flag to True.
		local left_over to 1- V_throttle_val^2.
		if H_throttle_test > sqrt(left_over) {
			set H_throttle_val to MAX(0,MIN(H_throttle_test,sqrt(left_over))).
			set S_throttle to 0.
		} else {
			set H_throttle_val to H_throttle_test.
			set S_throttle to MAX(0,MIN(S_throttle_test,sqrt(left_over - H_throttle_test^2))).
		}
	} else {
		set left_over_flag to False.
		set S_throttle to S_throttle_test.
		set H_throttle_val to H_throttle_test.
	}

	local Follow_Mode_Ang to VANG(landing_pos:position,ship:velocity:surface).
	local Follow_Mode_Dist to VXCL(UP:vector,landing_pos:position):mag/true_alt().
	//if Follow_Mode_Ang < 15 AND Follow_Mode_Dist < 0.1 AND not(Follow_Mode) {
	//	set Follow_Mode to True.
  //  set throttle_hyst to False.
	//}
  if PN_throttle():mag < throttle_hyst_LL/100 AND not(Follow_Mode) {
		set Follow_Mode to True.
    set throttle_hyst to False.
	}

	if groundspeed < 10 AND not(Follow_Mode) {
		set Follow_Mode to True.
	}

	if Follow_Mode {
    //set throttle_vec to UP:vector*V_throttle_val + Follow_throttle_func().
    set throttle_vec to UP:vector*V_throttle_val + PN_throttle().
	} else {
		set throttle_vec to UP:vector*V_throttle_val - H_vec()*H_throttle_val + S_vec()*S_throttle.
	}

	local throttle_hyst_test to throttle_vec:mag.
	local ang_diff to VANG(throttle_vec,ship:facing:vector).
	set throttle_hyst to Hysteresis(100*throttle_hyst_test,throttle_hyst, throttle_hyst_UL, throttle_hyst_LL).
	set ang_hyst to Hysteresis(ang_diff,ang_hyst,ang_hyst_UL,ang_hyst_LL,False).

	if throttle_hyst {
		if ang_hyst {
			lock throttle to throttle_vec:mag.
			lock steering to LOOKDIRUP(throttle_vec,facing:topvector).
		} else {
			lock throttle to 0.
      lock steering to LOOKDIRUP(throttle_vec,facing:topvector).
		}
	} else {
		lock throttle to 0.
		lock steering to LOOKDIRUP(srfretrograde:vector,facing:topvector).
	}

  local line_count to 0.
	print "V_throttle_val = " + round(100*(VDOT(UP:vector,throttle_vec)),1) + "%   " at(0,line_count).
  if Follow_Mode {
    set line_count to line_count + 1.
  	print "F_throttle_val = " +round(100*sqrt(max(0,throttle_vec:mag^2 - V_throttle_val^2)),1) + "%   " at(0,line_count).
    set line_count to line_count + 1.
  	print "PN_throttle_val= " +round(100*PN_throttle():mag) + "%   " at(0,line_count).
  } else {
    set line_count to line_count + 1.
  	print "H_throttle_val = " +round(100*(VDOT(H_vec(),throttle_vec)),1) + "%   " at(0,line_count).
    set line_count to line_count + 1.
  	print "S_throttle     = " +round(100*(VDOT(S_vec(),throttle_vec)),1) + "%   " at(0,line_count).
  }
  set line_count to line_count + 1.
	print "Vmax_v         = " +round(Vmax_v,2) at(0,line_count).
  set line_count to line_count + 1.
	print "Speed_v         = " +round(verticalspeed,2) at(0,line_count).
  set line_count to line_count + 1.
	print "Vmax_h         = " +round(Vmax_h,2) at(0,line_count).
  set line_count to line_count + 1.
	print "Speed_h       = " +round(Speed_h,2) at(0,line_count).
  set line_count to line_count + 1.
	print "Longitude      = " +round(ship:geoposition:lng,2) at(0,line_count).
  set line_count to line_count + 1.
	print "Throttle       = " + round(100*throttle_vec:mag,0) + "%   " at(0,line_count).
  set line_count to line_count + 1.
	print "throttle_hyst  = " + throttle_hyst + "   " at(0,line_count).
  set line_count to line_count + 1.
	print "left_over_flag = " + left_over_flag + "   " at(0,line_count).
  set line_count to line_count + 1.
	print "ang_diff       = " + round(ang_diff,1) + "   " at(0,line_count).
  set line_count to line_count + 1.
	print "ang_hyst       = " + ang_hyst + "   " at(0,line_count).
  set line_count to line_count + 1.
	print "S_deltaV       = " + round(S_deltaV,2) + "   " at(0,line_count).
  set line_count to line_count + 1.
	print "groundspeed    = " + round(groundspeed,2) + "   " at(0,line_count).
  set line_count to line_count + 1.
	print "land_surf      = " + round(land_surf,2) + "   " at(0,line_count).
  set line_count to line_count + 1.
	print "Follow_Mode    = " + Follow_Mode + "   " at(0,line_count).
  set line_count to line_count + 1.
	print "TouchDown_Mode     = " + TouchDown_Mode + "   " at(0,line_count).
  set line_count to line_count + 1.
	print "S_throttle_enable  = " + S_throttle_enable + "   " at(0,line_count).
  set line_count to line_count + 1.
	print "S_throttle_test    = " + round(S_throttle_test,2) + "   " at(0,line_count).
  set line_count to line_count + 1.
	print "throttle_hyst_test = " + round(throttle_hyst_test,2) + "   " at(0,line_count).
  set line_count to line_count + 1.
  print "V_speed_check      = " + V_speed_check + "   " at(0,line_count).
  set line_count to line_count + 1.
  print "true_alt           = " + round(true_alt(),2) + "   " at(0,line_count).

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
