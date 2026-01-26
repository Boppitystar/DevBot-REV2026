// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.AutoConstants;

import frc.robot.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
  //create 4 MAXSwerveModules 
  
  private final MaxSwerveModule mFrontLeft = new MaxSwerveModule(
    DriveConstants.FRONT_LEFT_DRIVING_CAN_ID,
    DriveConstants.FRONT_LEFT_TURNING_CAN_ID,
    DriveConstants.FRONT_LEFT_CHASSIS_ANGULAR_OFFSET);
  
  private final MaxSwerveModule mFrontRight = new MaxSwerveModule(
    DriveConstants.FRONT_RIGHT_DRIVING_CAN_ID,
    DriveConstants.FRONT_RIGHT_TURNING_CAN_ID,
    DriveConstants.FRONT_RIGHT_CHASSIS_ANGULAR_OFFSET);
  
  private final MaxSwerveModule mBackLeft = new MaxSwerveModule(
    DriveConstants.BACK_LEFT_DRIVING_CAN_ID,
    DriveConstants.BACK_LEFT_TURNING_CAN_ID,
    DriveConstants.BACK_LEFT_CHASSIS_ANGULAR_OFFSET);

  private final MaxSwerveModule mBackRight = new MaxSwerveModule(
    DriveConstants.BACK_RIGHT_DRIVING_CAN_ID,
    DriveConstants.BACK_RIGHT_TURNING_CAN_ID,
    DriveConstants.BACK_RIGHT_CHASSIS_ANGULAR_OFFSET);
  private boolean HeadingPIDEngaged = false;
  private PIDController mHeadingController;

  //mGyro sensor/IMU (usb input type to roborio)
  private final AHRS mGyro = new AHRS(NavXComType.kUSB1); 

  //Odometry class for tracking robot pose 
  SwerveDriveOdometry Odometry = new SwerveDriveOdometry(
    DriveConstants.DriveKinematics,
    Rotation2d.fromDegrees(-mGyro.getAngle()), //inversion as NavX is CCW+
    new SwerveModulePosition[] {
        mFrontLeft.getPosition(),
        mFrontRight.getPosition(),
        mBackLeft.getPosition(),
        mBackRight.getPosition()
  });

  
  public DriveSubsystem() {
    mHeadingController = new PIDController(0.1, 0, 0);
    mHeadingController.setSetpoint(0);
    mHeadingController.setTolerance(AutoConstants.ROT_TOLERANCE_TAG_ALIGNMENT);
    //usage reporting for MAXSwerve template 
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
  }

  @Override
  public void periodic(){
  //updates Odometry in periodic block 
    Odometry.update(
        Rotation2d.fromDegrees(-mGyro.getAngle()),
        new SwerveModulePosition[] {
            mFrontLeft.getPosition(),
            mFrontRight.getPosition(),
            mBackLeft.getPosition(),
            mBackRight.getPosition()
        });
  }

  /**
   * Returns the currently-estimated pose of the robot.
   * @return The pose.
   */
  public Pose2d getPose() {
    return Odometry.getPoseMeters();
  }

  /**
   * Resets the Odometry to the specified pose.
   * @param pose The pose to which to set the Odometry.
   */
  public void resetOdometry(Pose2d pose) {
    Odometry.resetPosition(
        Rotation2d.fromDegrees(-mGyro.getAngle()),
        new SwerveModulePosition[] {
            mFrontLeft.getPosition(),
            mFrontRight.getPosition(),
            mBackLeft.getPosition(),
            mBackRight.getPosition()
        },
        pose);
  }
  public double HeadingPID(double targetHeading) {
      double RotOutput = mHeadingController.calculate(targetHeading);
      return RotOutput;
  }

  public void driveJoystick(double xJoystick, double yJoystick, double rotJoystick, boolean fieldRelative) {
    
    //convert joystick input (-1, 1) to m/s for drivetrain 
    double xSpeedDelivered = xJoystick * DriveConstants.MAX_SPEED_METERS_PER_SECOND; 
    double ySpeedDelivered = yJoystick * DriveConstants.MAX_SPEED_METERS_PER_SECOND;
    double rotDelivered;
    if (HeadingPIDEngaged){
      rotDelivered = HeadingPID(0);

    } else {
      rotDelivered = rotJoystick * DriveConstants.MAX_ANGULAR_SPEED;
    }
    driveChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, fieldRelative);
  }

  public void driveChassisSpeeds(double xSpeed, double ySpeed, double rotValue, boolean fieldRelative){
    // clamps speed to be within max/min range 
    double xSpeedClamped = MathUtil.clamp(xSpeed, -DriveConstants.MAX_SPEED_METERS_PER_SECOND,DriveConstants.MAX_SPEED_METERS_PER_SECOND); 
    double ySpeedClamped = MathUtil.clamp(ySpeed, -DriveConstants.MAX_SPEED_METERS_PER_SECOND,DriveConstants.MAX_SPEED_METERS_PER_SECOND); 
    double rotDelivered = MathUtil.clamp(rotValue, -DriveConstants.MAX_ANGULAR_SPEED, DriveConstants.MAX_ANGULAR_SPEED);

    //convert chassis speed to swerve module states (motor output); field relative or robot relative 
    var swerveModuleStates = DriveConstants.DriveKinematics.toSwerveModuleStates(
      fieldRelative 
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedClamped, ySpeedClamped, 
          rotDelivered, Rotation2d.fromDegrees(-mGyro.getAngle()))
        
        : new ChassisSpeeds(xSpeedClamped, ySpeedClamped, rotDelivered));
    
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
    
    mFrontLeft.setDesiredState(swerveModuleStates[0]);
    mFrontRight.setDesiredState(swerveModuleStates[1]);
    mBackLeft.setDesiredState(swerveModuleStates[2]);
    mBackRight.setDesiredState(swerveModuleStates[3]);

  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    mFrontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    mFrontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    mBackLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    mBackRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.MAX_SPEED_METERS_PER_SECOND);
    mFrontLeft.setDesiredState(desiredStates[0]);
    mFrontRight.setDesiredState(desiredStates[1]);
    mBackLeft.setDesiredState(desiredStates[2]);
    mBackRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    mFrontLeft.resetEncoders();
    mFrontRight.resetEncoders();
    mBackLeft.resetEncoders();
    mBackRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
    mGyro.reset();
  }

  /**
   * Returns the heading of the robot.
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(-mGyro.getAngle()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return mGyro.getRate() * (DriveConstants.GYRO_REVERSED ? -1.0 : 1.0);
  }

  //command to set module positions to an X shape for defense 
  public Command defensePosition(){
    return run(
      () -> {
        setX();
      });
  }

  //command to reset gyro
  public Command resetGyro(){
    return run(
      () -> {
        zeroHeading();
      });
  }
  public Command HeadingPIDenable(){
    return run(
      () -> {
        HeadingPIDEngaged = true;
      });
  }
  public Command HeadingPIDdisable(){
    return run(
      () -> {
        HeadingPIDEngaged = false;
      });
  }
}
