#include "vex.h"
#include "../include/robot-config.h"
#include "../include/subsystems.h"
#include "../../include/motor-control.h"
#include <cmath>

using namespace vex;
//TUNE THIS
double lift_inches_per_output_revolution = 28.0;
double lift_gear_reduction = 60.0 / 12.0;
double lift_max_height_inches = 50.0;
double lift_raise_rate_inches_per_second = 8.0;
double lift_kp = 5.0;
double lift_ki = 0.0;
double lift_kd = 0.25;
double lift_hold_voltage = 1.5;

double claw_intake_voltage = 12.0;
double claw_outtake_voltage = 2.5;
double claw_detect_distance_mm = 60.0;
double claw_rotator_retract_degrees = 10.0;
double claw_rotator_open_degrees = 60.0;

double toggle_motor_voltage = 12.0;



namespace {

enum class LiftState {
  Intaking,
  ManualTarget,
  DriverRaising,
  DriverScoring,
  AutonomousRaising,
  AutonomousScoring
};

constexpr double loop_period_seconds = 0.01;
constexpr double lift_position_tolerance_inches = 0.25;
constexpr double release_lead_inches = 5.0;

volatile bool driver_raise_requested = false;
volatile bool manual_lift_target_requested = false;
volatile double requested_lift_target_inches = 0.0;
volatile bool autonomous_score_requested = false;
volatile bool autonomous_score_finished = false;
volatile int roller_target_color = static_cast<int>(RollerColor::Red);

double lift_target_inches = 0.0;
double scoring_height_inches = 0.0;
double lift_integral = 0.0;
double previous_lift_error = 0.0;
double game_element_release_started_msec = 0.0;
bool lift_pid_initialized = false;
volatile bool has_game_element = false;
bool game_element_released = false;
bool subsystem_task_started = false;
bool autonomous_sequence_active = false;
volatile int autonomous_score_count = 0;
LiftState lift_state = LiftState::Intaking;

double liftHeightInches() {
  const double motor_degrees =
      (liftL.position(degrees) + liftR.position(degrees)) / 2.0;
  return motor_degrees / 360.0 / lift_gear_reduction *
         lift_inches_per_output_revolution;
}

void logLiftCommand(const char* command, double target_height,
                    double current_height) {
  logger.info("Lift command (%s): target %.2f in, current height %.2f in",
              command, target_height, current_height);
}

bool targetColorAtSensor() {
  const double hue = toggleOptical.hue();
  switch (static_cast<RollerColor>(roller_target_color)) {
    case RollerColor::Red:
      return hue < 25.0 || hue >= 335.0;
    case RollerColor::Yellow:
      return hue >= 35.0 && hue <= 75.0;
    case RollerColor::Blue:
      return hue >= 190.0 && hue <= 260.0;
  }
  return false;
}

void updateLift(double current_height) {
  const double error = lift_target_inches - current_height;
  if (!lift_pid_initialized) {
    previous_lift_error = error;
    lift_pid_initialized = true;
  }

  if (std::fabs(error) > 2.0) {
    lift_integral = 0.0;
  } else {
    lift_integral += error * loop_period_seconds;
    if (lift_integral > 10.0) lift_integral = 10.0;
    if (lift_integral < -10.0) lift_integral = -10.0;
  }

  const double derivative = (error - previous_lift_error) / loop_period_seconds;
  previous_lift_error = error;

  if (lift_target_inches <= lift_position_tolerance_inches &&
      current_height <= lift_position_tolerance_inches) {
    lift.stop(hold);
    lift_integral = 0.0;
    return;
  }

  double output = lift_kp * error + lift_ki * lift_integral +
                 lift_kd * derivative + lift_hold_voltage;
  if (output > 12.0) output = 12.0;
  if (output < -12.0) output = -12.0;
  if (current_height <= 0.0 && output < 0.0) output = 0.0;
  if (current_height >= lift_max_height_inches && output > 0.0) output = 0.0;

  lift.spin(fwd, output, volt);
}

void setScoringState(LiftState state) {
  scoring_height_inches = lift_target_inches;
  lift_target_inches = 0.0;
  game_element_released = false;
  lift_state = state;
  logLiftCommand("lower", lift_target_inches, liftHeightInches());
}

void updateClaw(double current_height) {
  if (lift_state == LiftState::Intaking) {
    if (!has_game_element && clawDetect.isObjectDetected() &&
        clawDetect.objectDistance(mm) <= claw_detect_distance_mm) {
      has_game_element = true;
      claw.stop(hold);
    } else if (!has_game_element) {
      claw.spin(fwd, claw_intake_voltage, volt);
    } else {
      claw.stop(hold);
    }
    return;
  }

  if (lift_state == LiftState::DriverRaising ||
      lift_state == LiftState::AutonomousRaising ||
      lift_state == LiftState::ManualTarget) {
    claw.stop(hold);
    return;
  }

  if (!game_element_released &&
      current_height <=
          std::fmax(0.0, scoring_height_inches - release_lead_inches)) {
    game_element_released = true;
    game_element_release_started_msec = Brain.timer(msec);
    clawRotator.spinToPosition(claw_rotator_open_degrees, degrees, false);
  }

  if (game_element_released) {
    claw.spin(reverse, claw_outtake_voltage, volt);
  } else {
    claw.stop(hold);
  }
}

void updateToggleRoller() {
  if (!toggleOptical.isNearObject()) {
    toggle.stop(hold);
    return;
  }

  if (targetColorAtSensor()) {
    toggle.stop(hold);
  } else {
    toggle.spin(fwd, toggle_motor_voltage, volt);
  }
}

void subsystemLoop() {
  bool previous_driver_request = false;

  while (true) {
    const double current_height = liftHeightInches();
    const bool driver_request = driver_raise_requested;

    if (manual_lift_target_requested) {
      manual_lift_target_requested = false;
      lift_target_inches = requested_lift_target_inches;
      lift_state = LiftState::ManualTarget;
      lift_integral = 0.0;
      logLiftCommand("manual target", lift_target_inches, current_height);
    }

    if (autonomous_score_requested &&
        (lift_state == LiftState::Intaking ||
         lift_state == LiftState::ManualTarget)) {
      autonomous_score_requested = false;
      autonomous_score_finished = false;
      autonomous_sequence_active = true;
      scoring_height_inches =
          (autonomous_score_count + 1) * 10.0;
      if (scoring_height_inches > lift_max_height_inches) {
        scoring_height_inches = lift_max_height_inches;
      }
      lift_target_inches = scoring_height_inches;
      logLiftCommand("autonomous raise", lift_target_inches, current_height);
      game_element_released = false;
      lift_state = LiftState::AutonomousRaising;
    }

    if (lift_state == LiftState::DriverRaising) {
      if (driver_request) {
        lift_target_inches +=
            lift_raise_rate_inches_per_second * loop_period_seconds;
        if (lift_target_inches > lift_max_height_inches) {
          lift_target_inches = lift_max_height_inches;
        }
      } else if (previous_driver_request) {
        setScoringState(LiftState::DriverScoring);
      }
    } else if (lift_state == LiftState::Intaking && driver_request) {
      lift_target_inches = current_height;
      logLiftCommand("driver raise", lift_target_inches, current_height);
      lift_state = LiftState::DriverRaising;
    }

    if (lift_state == LiftState::AutonomousRaising &&
        std::fabs(scoring_height_inches - current_height) <=
            lift_position_tolerance_inches) {
      setScoringState(LiftState::AutonomousScoring);
    }

    if (lift_state == LiftState::ManualTarget &&
        lift_target_inches <= lift_position_tolerance_inches &&
        current_height <= lift_position_tolerance_inches) {
      lift_state = LiftState::Intaking;
    }

    if ((lift_state == LiftState::DriverScoring ||
         lift_state == LiftState::AutonomousScoring) &&
        current_height <= lift_position_tolerance_inches &&
        game_element_released &&
        Brain.timer(msec) - game_element_release_started_msec >= 300.0) {
      lift_target_inches = 0.0;
      lift_state = LiftState::Intaking;
      has_game_element = false;
      clawRotator.spinToPosition(claw_rotator_retract_degrees, degrees, false);
      claw.stop(hold);
      if (autonomous_sequence_active) {
        ++autonomous_score_count;
        autonomous_score_finished = true;
        autonomous_sequence_active = false;
      }
    }

    updateLift(current_height);
    updateClaw(current_height);
    updateToggleRoller();

    previous_driver_request = driver_request;
    wait(10, msec);
  }
}

}  // namespace

void startSubsystems() {
  if (subsystem_task_started) return;

  liftL.setPosition(0.0, degrees);
  liftR.setPosition(0.0, degrees);
  clawRotator.setPosition(0.0, degrees);
  toggleOptical.setLight(ledState::on);
  toggleOptical.setLightPower(100.0);
  clawRotator.spinToPosition(claw_rotator_retract_degrees, degrees, false);

  subsystem_task_started = true;
  thread subsystem_task = thread(subsystemLoop);
}

void updateDriverSubsystems(bool raise_lift) {
  driver_raise_requested = raise_lift;
}

void setLiftTargetHeight(double height_inches) {
  if (!subsystem_task_started) startSubsystems();

  if (height_inches < 0.0) height_inches = 0.0;
  if (height_inches > lift_max_height_inches) {
    height_inches = lift_max_height_inches;
  }

  requested_lift_target_inches = height_inches;
  manual_lift_target_requested = true;
}

void setRollerTargetColor(RollerColor color) {
  roller_target_color = static_cast<int>(color);
}

bool isGamePieceAcquired() {
  return has_game_element;
}

void scoreLiftAutonomous() {
  if (!subsystem_task_started) startSubsystems();

  autonomous_score_finished = false;
  autonomous_score_requested = true;
  while (!autonomous_score_finished) {
    wait(10, msec);
  }
}

int getAutonomousScoreCount() {
  return autonomous_score_count;
}
