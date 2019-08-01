function SetNode_BurnVector {
	parameter timeat,V_New.

	local V_timeat to velocityat(ship,timeat):orbit.

	local node_normal_vec to vcrs(ship:body:position,ship:velocity:orbit):normalized.
	local node_prograde_vec to V_timeat:normalized.
	local node_radial_vec to VCRS(node_normal_vec,node_prograde_vec).

	local burn_vector to (V_New - V_timeat).
	local burn_prograde to VDOT(node_prograde_vec,burn_vector).
	local burn_normal to VDOT(node_normal_vec,burn_vector).
	local burn_radial to VDOT(node_radial_vec,burn_vector).

	return NODE(timeat,burn_radial,burn_normal,burn_prograde).
}
