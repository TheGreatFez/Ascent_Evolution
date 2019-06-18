function makeIntegrator_func {
    parameter getInput.
    local tLast is time:seconds.
    local int is 0.
    if getInput:call():istype("Vector") {
        set int to v(0,0,0).
    }
    return {
        local now is time:seconds.
        local dt is now - tLast.
        set int to int + getInput() * (now - tLast).
        set tLast to now.
        return int.
    }.
}

function makeIntegrator_val {
    parameter init_value.
    local tLast is time:seconds.
    local int is init_value.
    if init_value:istype("Vector") {
        set int to v(0,0,0).
    }
    return {
        parameter getInput.
        local now is time:seconds.
        local dt is now - tLast.
        set int to int + getInput * (now - tLast).
        set tLast to now.
        return int.
    }.
}
