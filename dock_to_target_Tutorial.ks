SAS OFF.
RCS ON.
clearscreen.
clearvecdraws().

function target_relative_velocity {
  if HASTARGET {
    if target:istype("Part") {
      return target:ship:velocity:orbit - ship:velocity:orbit.
    } else {
      return target:velocity:orbit - ship:velocity:orbit.
    }
  } else {
    return V(0,0,0).
  }
}

function target_relative_position {
  parameter checker is ship.
  if HASTARGET {
    return -(checker:position - target:position).
  } else {
    return V(0,0,0).
  }

}

function vector_to_translation {
  parameter input_vec.

  local top_vec to VDOT(ship:facing:topvector,input_vec).
  local star_vec to VDOT(ship:facing:starvector,input_vec).
  local fore_vec to VDOT(ship:facing:forevector,input_vec).
  local output_vec to V(star_vec,top_vec,fore_vec).

  set ship:control:translation to output_vec.
}

function AxisAlignManeuver_vec {
  parameter checker is ship.
  if HASTARGET {
    return target_relative_position(checker) - VDOT(target:facing:forevector,target_relative_position(checker))*target:facing:forevector.
  } else {
    return V(0,0,0).
  }

}

function position_correction {
    parameter target_position,checker.

    // Translation V(STAR,TOP,FORE)
    local max_acc to 0.5/mass.

    local dist to target_position:mag.
    local desired_speed to sqrt(max(0,2*dist*max_acc)).
    local desired_velocity to desired_speed*target_position:normalized.
    set DesiredVel_draw:vecupdater to {return 5*desired_velocity.}.
    set TargetRelVel_draw:vecupdater to {return -5*target_relative_velocity.}.
    local diff_vel to desired_velocity + target_relative_velocity.
    local output to (1/max_acc)*diff_vel.

    return output.
}

local DockingPort to ship:partstagged("MainDockingPort")[0].
// Vecdraws --------------------------------------------------------------------
// Vector from Docking Port to Docking Port
set TargetRelPos_draw to vecdraw().
set TargetRelPos_draw:startupdater to {return DockingPort:position.}.
set TargetRelPos_draw:vecupdater to {return target_relative_position(DockingPort).}.
set TargetRelPos_draw:show to true.
set TargetRelPos_draw:color to RGB(1,1,0).
set TargetRelPos_draw:label to "Target Relative Position".

// Stage 1 vector
set AxisAlignMan_draw to vecdraw().
set AxisAlignMan_draw:startupdater to {return DockingPort:position.}.
set AxisAlignMan_draw:vecupdater to {return AxisAlignManeuver_vec(DockingPort).}.
set AxisAlignMan_draw:show to true.
set AxisAlignMan_draw:color to RGB(1,0,1).
set AxisAlignMan_draw:label to "Off Axis Maneuver Vector".

// Vectors for Guidance and Autopilot
set DesiredVel_draw to vecdraw().
set DesiredVel_draw:startupdater to {return DockingPort:position.}.
set DesiredVel_draw:show to true.
set DesiredVel_draw:color to RGB(1,0,0).
set DesiredVel_draw:label to "Des Vel".

set TargetRelVel_draw to vecdraw().
set TargetRelVel_draw:startupdater to {return DockingPort:position.}.
set TargetRelVel_draw:show to true.
set TargetRelVel_draw:color to RGB(0,1,0).
set TargetRelVel_draw:label to "Targ Rel Vel".

// Body Reference Vectors
set bodyX to vecdraw().
set bodyX:startupdater to {return DockingPort:position.}.
set bodyX:vecupdater to {return 10*DockingPort:facing:starvector.}.
set bodyX:show to true.
set bodyX:color to RGB(1,0,0).
set bodyX:label to "Star Vector".

set bodyY to vecdraw().
set bodyY:startupdater to {return DockingPort:position.}.
set bodyY:vecupdater to {return 10*DockingPort:facing:upvector.}.
set bodyY:show to true.
set bodyY:color to RGB(0,1,0).
set bodyY:label to "Up Vector".

set bodyZ to vecdraw().
set bodyZ:startupdater to {return DockingPort:position.}.
set bodyZ:vecupdater to {return 10*DockingPort:facing:forevector.}.
set bodyZ:show to true.
set bodyZ:color to RGB(0,0,1).
set bodyZ:label to "Fore Vector".

// -----------------------------------------------------------------------------



local capture to false.
when DockingPort:state:split(" ")[0] = "Acquire" then {
  set capture to true.
  unlock steering.
  SAS on.
  set ship:control:neutralize to true.
  set target to "".
  wait 0.
}
clearscreen.
local docking_stage to 0.
until DockingPort:state:split(" ")[0] = "Docked" {
  // Align to face target docking port
  if not(capture) AND docking_stage = 1 {
    lock steering to (-1*target:facing:vector):direction.
    until VANG(ship:facing:vector,(-1*target:facing:vector)) < 1 {
      print "Direction Angle Error = " + round(VANG(ship:facing:vector,(-1*target:facing:vector)),1) + "     "at(0,1).
    }
    //set docking_stage to 2.
  }
  // Reduce relative velocity to 0
  if not(capture) AND docking_stage = 2 {
    vector_to_translation(10*target_relative_velocity).
    if target_relative_velocity:mag < 0.01 {
      //set docking_stage to 3.
      //set ship:control:neutralize to true.
    }
  }
  // Align docking port facing axes
  if not(capture) AND docking_stage = 3 {
    local output to position_correction(AxisAlignManeuver_vec,DockingPort).
    vector_to_translation(output).
    if AxisAlignManeuver_vec:mag < 0.1 AND target_relative_velocity:mag < 0.1{
      //set docking_stage to 4.
      //set ship:control:neutralize to true.
    }
  }
  // Move forward to dock
  if not(capture) AND docking_stage >=  4 {
    local output to position_correction(target_relative_position,DockingPort).
    vector_to_translation(output).
  }

  if LIGHTS {
    LIGHTS OFF.
    set docking_stage to docking_stage + 1.
    wait 0.
  }

  local line to 1.
  print "docking_stage = " + docking_stage + "   " at(0,line).
  set line to line + 1.
  print "capture        = " + capture + "   "at(0,line).

  if not(capture){
    set line to line + 1.
    print "v_relative    = " + round(target_relative_velocity:mag,3) + "   " at(0,line).
    set line to line + 1.
    print "AxisAlignManeuver_vec:mag= " + round(AxisAlignManeuver_vec:mag,2) + "   " at(0,line).
    set line to line + 1.
    print "DP:state      = " + DockingPort:state + "   " at(0,line).
  } else {
    set line to line + 1.
    print "DP:state      = " + DockingPort:state + "   " at(0,line).
  }

  wait 0.

}
