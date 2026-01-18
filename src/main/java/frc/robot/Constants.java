// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
//import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity (length, complexity).
 */
public final class Constants {
  public static final class DriveConstants{ 
    //allowed max speeds
    public static final double MAX_SPEED_METERS_PER_SECOND = 4.8;
    public static final double MAX_ANGULAR_SPEED = 2 * Math.PI; //rad/s 

    //Chassis config - width, depth, CAN IDS and angular offset values in Designdoc.md
  
    public static final double WHEEL_CENTER_WIDTH = Units.inchesToMeters(7.25);
    // Distance between centers of right and left wheels on robot
    public static final double WHEEL_CENTER_DEPTH = Units.inchesToMeters(7.25);
    // Distance between front and back wheels on robot
    //depth/frontback distance from robot center to each wheel 

    public static final SwerveDriveKinematics DriveKinematics = new SwerveDriveKinematics(
        new Translation2d(WHEEL_CENTER_DEPTH, WHEEL_CENTER_WIDTH),
        new Translation2d(WHEEL_CENTER_DEPTH, -WHEEL_CENTER_WIDTH),
        new Translation2d(-WHEEL_CENTER_DEPTH, WHEEL_CENTER_WIDTH),
        new Translation2d(-WHEEL_CENTER_DEPTH, -WHEEL_CENTER_WIDTH));
      
    //angular offsets of module relative to chassis (rad)
    public static final double FRONT_LEFT_CHASSIS_ANGULAR_OFFSET = Math.PI/2;
    public static final double FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET = 0;
    public static final double BACK_LEFT_CHASSIS_ANGULAR_OFFSET = -Math.PI/2;
    public static final double BACK_RIGHT_CHASSIS_ANGULAR_OFFSET = -Math.PI;
    
    //SPARK MAX CAN IDs 
    public static final int FRONT_LEFT_DRIVING_CAN_ID = 5;
    public static final int FRONT_LEFT_TURNING_CAN_ID = 6;

    
    public static final int FRONT_RIGHT_DRIVING_CAN_ID = 3;
    public static final int FRONT_RIGHT_TURNING_CAN_ID = 4;
    
    public static final int BACK_LEFT_DRIVING_CAN_ID = 7;
    public static final int BACK_LEFT_TURNING_CAN_ID = 8;
   
    public static final int BACK_RIGHT_DRIVING_CAN_ID = 1;
    public static final int BACK_RIGHT_TURNING_CAN_ID = 2;

    public static final boolean GYRO_REVERSED = false;
  }

  public static final class ModuleConstants{
    // The MAXSwerve module 3 pinion ggears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (more teeth = faster)
    public static final int DRIVING_MOTOR_PINION_TEETH = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double DRIVING_MOTOR_FREE_SPEED_RPS = MotorConstants.FREE_SPEED_RPM / 60;
    public static final double WHEEL_DIAMETER_METERS = 0.0762;
    public static final double WHEEL_CIRCUMFERENCE_METERS = WHEEL_DIAMETER_METERS * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double DRIVING_MOTOR_REDUCTION = (45.0 * 22) / (DRIVING_MOTOR_PINION_TEETH * 15);
    public static final double DRIVE_WHEEL_FREE_SPEED_RPS = (DRIVING_MOTOR_FREE_SPEED_RPS * WHEEL_CIRCUMFERENCE_METERS)
        / DRIVING_MOTOR_REDUCTION;

  }
  public static class OperatorConstants {
    public static final int DRIVER_CONTROLLER = 0;
    public static final double DRIVE_DEADBAND = 0.05; 
  }

  public static final class AutoConstants {
    //add constants here that are not in pathplanner/limelight if needed
    public static final double X_TAG_ALIGNMENT_P = 0.1; //TODO: tune limelight pid  
    public static final double Y_TAG_ALIGNMENT_P = 0.1; 
    public static final double ROT_TAG_ALIGNMENT_P = 0.1; 

    public static final double ROT_SETPOINT_TAG_ALIGNMENT = 0;  // Rotation
    public static final double ROT_TOLERANCE_TAG_ALIGNMENT = 1;
    
    public static final double X_SETPOINT_TAG_ALIGNMENT = -0.34;  // Vertical pose and tolerance 
    public static final double X_TOLERANCE_TAG_ALIGNMENT = 0.02;

    public static final double Y_SETPOINT_TAG_ALIGNMENT = 0.16;  // Horizontal pose (- for diff sides of tag)
    public static final double Y_TOLERANCE_TAG_ALIGNMENT = 0.02; 

    public static final double DONT_SEE_TAG_WAIT_TIME = 1;
	  public static final double POSE_VALIDATION_TIME = 0.3;
  }

  public static final class MotorConstants {
    public static final double FREE_SPEED_RPM = 5676;
  }

}
