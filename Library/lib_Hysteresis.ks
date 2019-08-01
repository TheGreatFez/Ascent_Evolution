function makeHysteresis {
	parameter right_hand_limit, left_hand_limit, init_output is true, right_hand_output is true.

  local output to init_output.

	return {
    parameter input.

    if output = right_hand_output {
      if input <= left_hand_limit {
  			set output to not(right_hand_output).
  		}
    } else {
  		if input >= right_hand_limit {
  			set output to right_hand_output.
  		}
  	}
    return output.
  }.
}
