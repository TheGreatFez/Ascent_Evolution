function makeDerivator_N {
    parameter init_value, N_count is 0.
    local tLast is time:seconds.
    local der is 0.
    local derLast to 0.
    local inputLast is init_value.
    if init_value:istype("Vector") {
        set der to v(0,0,0).
    }
    return {
      parameter getInput.
      local now is time:seconds.
      local dt is now - tLast.
      if dt > 0 {
        set der_next to (getInput - inputLast)/dt.
        set der to (N_count*der + der_next)/(N_count + 1).
        set inputLast to getInput.
        set tLast to now.
      }

      return der.
    }.
}

function makeDerivator_dt {
    parameter init_value, time_interval is 0.
    local tLast is time:seconds.
    local der is 0.
    local inputLast is init_value.
    if init_value:istype("Vector") {
        set der to v(0,0,0).
    }
    return {
      parameter getInput.
      local now is time:seconds.
      local dt is now - tLast.
      if dt > time_interval {
        set der to (getInput - inputLast)/dt.
        set inputLast to getInput.
        set tLast to now.
      }

      return der.
    }.
}
