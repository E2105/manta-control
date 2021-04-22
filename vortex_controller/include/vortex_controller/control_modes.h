#pragma once

#include <string>

namespace ControlModes
{
enum ControlMode
{
  OPEN_LOOP           = 0,
  DEPTH_HOLD          = 1,
  HEADING_HOLD        = 2,
  DEPTH_HEADING_HOLD  = 3,
  STAY_LEVEL          = 4,
  FEEDBACK_CONTROL    = 5, // Added for semi autonomy. Funker kanskje ikke. Sjekk
  BRIEFCASE_MODE      = 6,
  CONTROL_MODE_END    = 7

};
}

typedef ControlModes::ControlMode ControlMode;

inline std::string controlModeString(ControlMode control_mode)
{
  std::string s;
  switch (control_mode)
  {
    case ControlModes::OPEN_LOOP:
    s = "OPEN LOOP";
    break;

    case ControlModes::DEPTH_HOLD:
    s = "DEPTH HOLD";
    break;

    case ControlModes::HEADING_HOLD:
    s = "HEADING HOLD";
    break;

    case ControlModes::DEPTH_HEADING_HOLD:
    s = "DEPTH HEADING HOLD";
    break;

    case ControlModes::STAY_LEVEL:
    s = "STAY LEVEL";
    break;
      
    case ControlModes::FEEDBACK_CONTROL:    // Added for semi autonomy
    s = "COMPLETE FEEBACK CONTROL";
    break;
      
    case ControlModes::BRIEFCASE_MODE:
    s = "BRIEFCASE MODE";
    break;

    default:
    s = "INVALID CONTROL MODE";
    break;
  }
  return s;
}
