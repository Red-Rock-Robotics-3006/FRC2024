// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
  public static class Intake {
    public static final int INTAKE_MOTOR_ID = 17;
  }
  public static class Index {
    public static final int INDEX_TOF_SENSOR_ID = 0;
    public static final int INDEX_MOTOR_ID = 60;
    public static final int ROLLER_MOTOR_ID = 51;
    public static final int ROLLER_CURRENT_LIMIT = 60;
  }
  public static class LED {
    public static final int NUM_LEDS = 30;
    public static final int LED_CHANNEL_ID = 9;
  }
  public static class Shooter {
    public static final int TOP_MOTOR_ID = 42;
    public static final int BOTTOM_MOTOR_ID = 59;
    public static final int LEFT_ANGLE_MOTOR_ID = 45;
    public static final int RIGHT_ANGLE_MOTOR_ID = 13;
    public static final int SHOOT_CURRENT_LIMIT = 80;
  }
  public static class Settings {
    public static final boolean INTAKE_HOMING_ENABLED = false;
    public static final boolean SHOOTER_HOMING_ENABLED = false;
    public static final boolean POLICE_MODE_ENABLED = true;
  }
}
