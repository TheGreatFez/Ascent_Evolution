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

function vector_to_translation {
  parameter input_vec.

  local top_vec to VDOT(ship:facing:topvector,input_vec).
  local star_vec to VDOT(ship:facing:starvector,input_vec).
  local fore_vec to VDOT(ship:facing:forevector,input_vec).
  local output_vec to V(star_vec,top_vec,fore_vec).

  set ship:control:translation to output_vec.
}

function target_relative_position {
  parameter checker is ship.
  if HASTARGET {
    return -(checker:position - target:position).
  } else {
    return V(0,0,0).
  }

}

function stage1_vec {
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
    local max_acc to 1/mass.
    local dist to target_position:mag.
    //print dist at(0,10).
    local desired_speed to sqrt(max(0,2*dist*max_acc)).
    //print "desired_speed = " + round(desired_speed,2) at(0,11).
    local desired_velocity to desired_speed*target_position:normalized.
    set vec4:vecupdater to {return 5*desired_velocity.}.
    set vec5:vecupdater to {return -5*target_relative_velocity.}.
    //print "desired_velocity = " + desired_velocity at(0,12).
    local diff_vel to desired_velocity + target_relative_velocity.
    local output to (mass/max_acc)*diff_vel.

    return output.
}

local DockingPort to ship:partstagged("MainDockingPort")[0].

local maxacc_RCS to 2/mass.

// Vector from Docking Port to Docking Port
set vec1 to vecdraw().
set vec1:startupdater to {return DockingPort:position.}.
set vec1:vecupdater to {return target_relative_position(DockingPort).}.
set vec1:show to true.

// Fore of Target
set vec2 to vecdraw().
set vec2:startupdater to {return DockingPort:position.}.
set vec2:vecupdater to {return 10*DockingPort:facing:forevector.}.
set vec2:show to true.

// Stage 1 vector
set vec3 to vecdraw().
set vec3:startupdater to {return DockingPort:position.}.
set vec3:vecupdater to {return stage1_vec(DockingPort).}.
set vec3:show to true.

set vec4 to vecdraw().
set vec4:startupdater to {return DockingPort:position.}.
set vec4:show to true.
set vec4:color to RGB(1,0,0).

set vec5 to vecdraw().
set vec5:startupdater to {return DockingPort:position.}.
set vec5:show to true.
set vec5:color to RGB(0,1,0).

set vec6 to vecdraw().
set vec6:startupdater to {return DockingPort:position.}.
set vec6:show to true.
set vec6:color to RGB(0,0,1).

lock steering to (-1*target:facing:vector):direction.
until VANG(ship:facing:vector,(-1*target:facing:vector)) < 1 {
  print "Direction Angle Error = " + round(VANG(ship:facing:vector,(-1*target:facing:vector)),1) + "   "at(0,1).
}

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

  if not(capture) AND docking_stage = 0 {
    vector_to_translation(10*target_relative_velocity).
    if target_relative_velocity:mag < 0.01 {
      set docking_stage to 1.
      set ship:control:neutralize to true.
    }
  }

  if not(capture) AND docking_stage = 1 {
    local output to position_correction(stage1_vec,DockingPort).
    vector_to_translation(output).
    if stage1_vec:mag < 0.1 AND target_relative_velocity:mag < 0.1{
      set docking_stage to 2.
      set ship:control:neutralize to true.
    }
  }

  if not(capture) AND docking_stage =  2 {
    local output to position_correction(target_relative_position,DockingPort).
    vector_to_translation(output).
  }


  local line to 1.
  print "docking_stage = " + docking_stage + "   " at(0,line).
  set line to line + 1.
  print "capture        = " + capture + "   "at(0,line).

  if not(capture){
    set line to line + 1.
    print "v_relative    = " + round(target_relative_velocity:mag,3) + "   " at(0,line).
    set line to line + 1.
    print "stage1_vec:mag= " + round(stage1_vec:mag,2) + "   " at(0,line).
    set line to line + 1.
    print "DP:state      = " + DockingPort:state + "   " at(0,line).
  } else {
    set line to line + 1.
    print "DP:state      = " + DockingPort:state + "   " at(0,line).
  }

  wait 0.

}
