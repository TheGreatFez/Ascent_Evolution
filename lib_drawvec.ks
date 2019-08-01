@LAZYGLOBAL OFF.

function DrawVecNow {
    parameter vec is V(10,0,0),start_updater is V(0,0,0).

    local vec_draw is VECDRAW().
    set vec_draw:startupdater to {return start_updater.}.
    set vec_draw:vecupdater to {return vec.}.
    set vec_draw:show to true.

}