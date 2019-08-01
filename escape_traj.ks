@LAZYGLOBAL OFF.

clearscreen.
clearvecdraws().

// Begin in a circular orbit
RUNONCEPATH("Library/lib_SetManNode_Velocity.ks").
RUNONCEPATH("Library/lib_OrbitalMechanics_Functions.ks").


function getEscapeTraj_Peri {
  parameter Exit_Speed.
  // Visualization
  local debug_vecdraws is false.
  local Body_Vel_draw is VECDRAW().
  local Burn_Node_draw is VECDRAW().
  local Exit_Vel_draw is VECDRAW().
  local Exit_Body_Vel_draw is VECDRAW().
  if debug_vecdraws {

    set Body_Vel_draw:startupdater to {return ship:body:position.}.
    set Body_Vel_draw:vecupdater to {return ship:body:orbit:velocity:orbit:normalized*ship:orbit:semimajoraxis.}.
    set Body_Vel_draw:show to true.

    set Burn_Node_draw:startupdater to {return ship:body:position.}.
    set Burn_Node_draw:vecupdater to {return ship:body:orbit:velocity:orbit:normalized*ship:orbit:semimajoraxis.}.
    set Burn_Node_draw:show to true.
    set Burn_Node_draw:color to GREEN.

    set Exit_Vel_draw:startupdater to {return ship:body:position.}.
    set Exit_Vel_draw:vecupdater to {return ship:body:orbit:velocity:orbit:normalized*ship:orbit:semimajoraxis.}.
    set Exit_Vel_draw:show to true.
    set Exit_Vel_draw:color to RED.

    set Exit_Body_Vel_draw:startupdater to {return ship:body:position.}.
    set Exit_Body_Vel_draw:vecupdater to {return ship:body:orbit:velocity:orbit:normalized*ship:orbit:semimajoraxis.}.
    set Exit_Body_Vel_draw:show to true.
    set Exit_Body_Vel_draw:color to BLUE.
  }

  for n in allnodes { remove n.}
  wait 0.
  local test_time is time:seconds + 100.
  local Exit_peri is ship:orbit:semimajoraxis.
  local V1 is velocityat(ship,test_time):orbit.
  local GM is ship:body:mu.
  local R_soi is ship:body:SOIradius.
  local Min_SMA is (Exit_peri + R_soi)/2.
  local Min_OrbitEnergy is -GM/(2*Min_SMA).
  local Delta_Speed_Limit is sqrt(2*(Min_OrbitEnergy + GM/R_soi)).

  if Exit_Speed < Delta_Speed_Limit AND Exit_Speed > -Delta_Speed_Limit {

    if Exit_Speed >= 0 {
      set Exit_Speed to Delta_Speed_Limit.
    } else {
      set Exit_Speed to -Delta_Speed_Limit.
    }
  }

  local Exit_OrbitEnergy is Exit_Speed^2/2 - GM/R_soi.
  local Final_Periapsis is 0.
  local Exit_ecc is 0.
  local Exit_h is 0.
  local Exit_peri_speed is 0.

  if Exit_OrbitEnergy > 0 { // Hyperbolic
    local Exit_SMA is GM/(-2*Exit_OrbitEnergy).
    set Exit_peri_speed to vis_via_speed(Exit_peri,Exit_SMA).
    set Exit_h to Exit_peri_speed*Exit_peri.
    set Exit_ecc to (Exit_h^2/GM)/Exit_peri - 1.
    local Exit_theta_exit is arccos((Exit_h^2/(GM*R_soi) - 1)/Exit_ecc).
    local Exit_gamma is arccos(Exit_h/(abs(Exit_Speed)*R_soi)).
    local Exit_theta_phase is Exit_theta_exit - ( 90 + Exit_gamma).
    if Exit_Speed > 0 {
      set Exit_theta_phase to Exit_theta_exit - ( 270 + Exit_gamma).
    }
  } else {

    local Exit_SMA is GM/(-2*Exit_OrbitEnergy).
    set Exit_peri_speed to vis_via_speed(Exit_peri,Exit_SMA).
    set Exit_h to Exit_peri_speed*Exit_peri.
    local Exit_apo is 2*Exit_SMA - Exit_peri.
    set Exit_ecc to (Exit_apo - Exit_peri)/(Exit_apo + Exit_peri).
  }

  local Exit_theta_exit is arccos((Exit_h^2/(GM*R_soi) - 1)/Exit_ecc).
  local Exit_gamma is arccos(Exit_h/(abs(Exit_Speed)*R_soi)).
  local Exit_theta_phase is Exit_theta_exit - ( 90 + Exit_gamma).
  if Exit_Speed > 0 {
    set Exit_theta_phase to Exit_theta_exit - ( 270 + Exit_gamma).
  }

  local body_orbVel is ship:body:orbit:velocity:orbit.
  local body_orbPos is ship:body:position - ship:body:body:position.
  local body_angVel is VCRS(body_orbPos,body_orbVel).

  local Burn_Node_Relative_vec is body_orbVel*AngleAxis(-Exit_theta_phase,body_angVel).
  local Burn_Node_Relative_theta is FindTheta_Vec(Burn_Node_Relative_vec).
  local Burn_Node_eta to ETA_to_theta(Burn_Node_Relative_theta).
  local Burn_Node_time to time:seconds + Burn_Node_eta.

  local body_pos_atTime is positionat(ship,Burn_Node_time) - ship:body:position.
  local temp_vec is VCRS(body_pos_atTime,velocityat(ship,Burn_Node_time):orbit).
  local Burn_Node_vec_norm is VCRS(temp_vec,body_pos_atTime):normalized.

  local Exit_Node to SetNode_BurnVector(Burn_Node_time,Burn_Node_vec_norm*Exit_peri_speed).
  add Exit_Node.
  wait 0.
  if debug_vecdraws {
    set Burn_Node_draw:vecupdater to {
      local body_orbVel_show is ship:orbit:semimajoraxis*ship:body:orbit:velocity:orbit:normalized.
      return body_orbVel_show*AngleAxis(-Exit_theta_phase,body_angVel).
    }.

    set Exit_Vel_draw:vecupdater to {
      local temp_vec_draw is velocityat(ship,nextnode:orbit:nextpatcheta + time:seconds):orbit:normalized.
      return temp_vec_draw*2*ship:orbit:semimajoraxis.
    }.

    set Exit_Body_Vel_draw:vecupdater to {return velocityat(ship:body,nextnode:orbit:nextpatcheta + time:seconds):orbit:normalized*2*ship:orbit:semimajoraxis.}.

    wait until false.
  }
  set Final_Periapsis to nextnode:orbit:nextpatch:periapsis.
  return Final_Periapsis.

}
