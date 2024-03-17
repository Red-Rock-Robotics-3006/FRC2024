// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class AprilTags
  {
    public static final double[][] locations = { // Rotation is in DEGREES
      {0,      0,      0,     0}, // Limelight offset on Robot, temp place? {x, y, z, angle up}
      {593.68, 9.68,   53.38, 120}, // AprilTag 1,  {x, y, z, rotation}
      {637.21, 34.79,  53.38, 120}, // AprilTag 2,  {x, y, z, rotation}
      {652.73, 196.17, 57.13, 180}, // AprilTag 3,  {x, y, z, rotation}
      {652.73, 218.42, 57.13, 180}, // AprilTag 4,  {x, y, z, rotation}
      {578.77, 323.00, 53.38, 270}, // AprilTag 5,  {x, y, z, rotation}
      {72.5,   323.00, 53.38, 270}, // AprilTag 6,  {x, y, z, rotation}
      {-1.50,  218.42, 57.13, 0},   // AprilTag 7,  {x, y, z, rotation}
      {-1.50,  196.17, 57.13, 0},   // AprilTag 8,  {x, y, z, rotation}
      {14.02,  34.79,  53.38, 60},  // AprilTag 9,  {x, y, z, rotation}
      {57.54,  9.68,   53.38, 60},  // AprilTag 10, {x, y, z, rotation}
      {468.69, 146.19, 52.00, 300}, // AprilTag 11, {x, y, z, rotation}
      {468.69, 177.10, 52.00, 60},  // AprilTag 12, {x, y, z, rotation}
      {441.74, 161.62, 52.00, 180}, // AprilTag 13, {x, y, z, rotation}
      {209.48, 161.62, 52.00, 0},   // AprilTag 14, {x, y, z, rotation}
      {182.73, 177.10, 52.00, 120}, // AprilTag 15, {x, y, z, rotation}
      {182.73, 146.19, 52.00, 240}  // AprilTag 16, {x, y, z, rotation}
    };
  }


  public static class Shooter
  {
    public static final int LEFT_MOTOR_ID = 42;
    public static final int RIGHT_MOTOR_ID = 59;
    public static final int LEFT_ANGLE_MOTOR_ID = 13;
    public static final int RIGHT_ANGLE_MOTOR_ID = 45;

    public static final double CENTER_ANGLE = 45.0; // TODO FILLER
    public static final double LEFT_ANGLE = 0.0; // TODO FILLER
    public static final double RIGHT_ANGLE = 0.0; // TODO FILLER
    public static final double PODIUM_ANGLE = 0.0; // TODO FILLER
    public static final double RED_PODIUM_HEADING = 0.0; // TODO FILLER
    public static final double BLUE_PODIUM_HEADING = 0.0; // TODO FILLER
    public static final double AMP_ANGLE = 0.0; // TODO FILLER
  }
  
  public static class Transfer {
    public static final int TOP_MOTOR_ID = 1001;//FILLER
    public static final int BOTTOM_MOTOR_ID = 1002;//FILLER
  }
  public static class Intake {
    public static final int INTAKE_MOTOR_ID = 1000;//FILLER
  }
  public static class Index {
    public static final int SWITCH_CHANNEL_ID = 1;//FILLER
  }
}
// AprilTag Coordinates
// The XYZ Origin is established in the bottom left corner of
// the field (as viewed in the image above). An x
// coordinate of 0 is aligned with the Blue Alliance Station
// diamond plate. A y coordinate of 0 is aligned with the
// side border polycarbonate on the Scoring Table side of
// the field. A z coordinate of 0 is on the carpet.
// +Z is up into the air from the carpet, +X is horizontal to the
// right (in this image above) toward the opposing alliance
// stations, and +Y runs from the Field Border towards the SPEAKERS.
// The face-pose of the tags is denoted with 1 degree
// representation, the Z-rotation. 0° faces the red alliance
// station, 90° faces the non- scoring table side, and 180°
// faces the blue alliance station. Distances are measured to the
// center of the tag.

// ID X Y Z Rotation
// 1 593.68 9.68 53.38 120°
// 2 637.21 34.79 53.38 120°
// 3 652.73 196.17 57.13 180°
// 4 652.73 218.42 57.13 180°
// 5 578.77 323.00 53.38 270°
// 6 72.5 323.00 53.38 270°
// 7 -1.50 218.42 57.13 0°
// 8 -1.50 196.17 57.13 0°
// 9 14.02 34.79 53.38 60°
// 10 57.54 9.68 53.38 60°
// 11 468.69 146.19 52.00 300°
// 12 468.69 177.10 52.00 60°
// 13 441.74 161.62 52.00 180°
// 14 209.48 161.62 52.00 0°
// 15 182.73 177.10 52.00 120°
// 16 182.73 146.19 52.00 240°