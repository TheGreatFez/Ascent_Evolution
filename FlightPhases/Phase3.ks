@LAZYGLOBAL OFF.
//Phase 3: Capture and circularize at Minmus at an altitude of 30km and 0 inclination
KUniverse:quicksaveto("Kerpollo 2 Phase 3 - Pre-Capture at Minmus").
run capture.
local SMA_desired to 30000.
local INC_desired to 0.0.
// Check for Prograde or Retrograde
if ship:orbit:inclination > 90 OR ship:orbit:inclination < -90 {
  if ship:orbit:inclination < 0 {
    set INC_desired to -180.0.
  } else {
    set INC_desired to 180.0.
  }
}
local LAN_desired to ship:orbit:LAN.
local DesiredOrbit to lexicon("INC",INC_desired,"LAN",LAN_desired,"APO",SMA_desired,"PER",SMA_desired).
run change_orbit(DesiredOrbit).
