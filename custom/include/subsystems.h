#pragma once

enum class RollerColor {
  Red,
  Yellow,
  Blue
};

extern double lift_inches_per_output_revolution;
extern double lift_gear_reduction;
extern double lift_max_height_inches;
extern double lift_raise_rate_inches_per_second;
extern double lift_kp;
extern double lift_ki;
extern double lift_kd;
extern double lift_hold_voltage;

extern double claw_intake_voltage;
extern double claw_outtake_voltage;
extern double claw_detect_distance_mm;
extern double claw_rotator_retract_degrees;
extern double claw_rotator_open_degrees;

extern double toggle_motor_voltage;

void startSubsystems();
void updateDriverSubsystems(bool raise_lift);
void setLiftTargetHeight(double height_inches);
void setRollerTargetColor(RollerColor color);
bool isGamePieceAcquired();
void scoreLiftAutonomous();
int getAutonomousScoreCount();