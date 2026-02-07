// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;


import java.util.function.Supplier;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import static edu.wpi.first.units.Units.Degrees;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.LimelightHelpers;

import dev.doglog.DogLog;


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

  //mGyro sensor/IMU (usb input type to roborio)
  private final AHRS mGyro = new AHRS(NavXComType.kUSB1); 

  private final Field2d field2d = new Field2d();


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

  /* Here we use SwerveDrivePoseEstimator so that we can fuse odometry readings. The numbers used
  below are robot specific, and should be tuned. */
  private final SwerveDrivePoseEstimator mPoseEstimator =
      new SwerveDrivePoseEstimator(
          DriveConstants.DriveKinematics,
          Rotation2d.fromDegrees(-mGyro.getAngle()),
          new SwerveModulePosition[] {
            mFrontLeft.getPosition(),
            mFrontRight.getPosition(),
            mBackLeft.getPosition(),
            mBackRight.getPosition()
          },
          new Pose2d(),
          VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
          VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(30)));


  
  public DriveSubsystem() {
    //usage reporting for MAXSwerve template 
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);
  }

  @Override
  public void periodic(){
    updateVisionOdometry();

    //adding field map to smart dashboard 
    field2d.setRobotPose(mPoseEstimator.getEstimatedPosition());
    SmartDashboard.putData(field2d);

    DogLog.log("hub distance", getHubDistance(), "meters");
    DogLog.log("ferry distance", getFerryDistance(), "meters");
    //OLD ODOMETRY UPDATE: pdates Odometry in periodic block 
    /*  Odometry.update(
          Rotation2d.fromDegrees(-mGyro.getAngle()),
          new SwerveModulePosition[] {
              mFrontLeft.getPosition(),
              mFrontRight.getPosition(),
              mBackLeft.getPosition(),
              mBackRight.getPosition()
          });
          */  
  }

  /**
   * Returns the currently-estimated pose of the robot.
   * @return The pose.
   */
  public Pose2d getPose() {
    return mPoseEstimator.getEstimatedPosition();
  }
  public Rotation2d getRotationPose2d() {
    return mPoseEstimator.getEstimatedPosition().getRotation();
  }

  /**
   * Resets the Odometry to the specified pose.
   * @param pose The pose to which to set the Odometry.
   */
  //TODO: change to mPoseEstimator When it works 
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

  public void driveJoystick(double xJoystick, double yJoystick, double rotJoystick, boolean fieldRelative) {
    
    //convert joystick input (-1, 1) to m/s for drivetrain 
    double xSpeedDelivered = xJoystick * DriveConstants.MAX_SPEED_METERS_PER_SECOND; 
    double ySpeedDelivered = yJoystick * DriveConstants.MAX_SPEED_METERS_PER_SECOND;
    double rotDelivered = rotJoystick * DriveConstants.MAX_ANGULAR_SPEED;

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
          rotDelivered, mPoseEstimator.getEstimatedPosition().getRotation())
        
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
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setStill() {
    mFrontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    mFrontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    mBackLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    mBackRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
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

    /** Updates the field relative position of the robot. */
  public void updateVisionOdometry() {
    mPoseEstimator.update(
          Rotation2d.fromDegrees(-mGyro.getAngle()),
        new SwerveModulePosition[] {
          mFrontLeft.getPosition(),
          mFrontRight.getPosition(),
          mBackLeft.getPosition(),
          mBackRight.getPosition()
        });


    boolean useMegaTag2 = true; //set to false to use MegaTag1
    boolean doRejectUpdate = false;
    if(useMegaTag2 == false)
    {
      LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
      
      if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
      {
        if(mt1.rawFiducials[0].ambiguity > .7)
        {
          doRejectUpdate = true;
        }
        if(mt1.rawFiducials[0].distToCamera > 3)
        {
          doRejectUpdate = true;
        }
      }
      if(mt1.tagCount == 0)
      {
        doRejectUpdate = true;
      }

      if(!doRejectUpdate)
      {
        mPoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
        mPoseEstimator.addVisionMeasurement(
            mt1.pose,
            mt1.timestampSeconds);
      }
    }
    else if (useMegaTag2 == true)
    {
      LimelightHelpers.SetRobotOrientation("limelight", mPoseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
      if(Math.abs(mGyro.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
      {
        doRejectUpdate = true;
      }
      if(mt2.tagCount == 0)
      {
        doRejectUpdate = true;
      }
      if(!doRejectUpdate)
      {
        mPoseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
        mPoseEstimator.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds);
      }
    }
  }
  
  public double getFerryDistance() {
      return getShotDistance(DriveConstants.getFerryPose(getPose().getTranslation()).toPose2d().getTranslation());
  }
  
  public void printFerryDistance(){
    System.out.println("Ferry distance" + getFerryDistance());
  }

  public double getShotDistance(Translation2d targetPose) {
        Pose2d drivePose = getPose();
        double centerToTargetMeters = drivePose.getTranslation().getDistance(targetPose);
        double centerToShooterMeters = DriveConstants.shooterSideOffset;
        double shooterToTargetMeters = Math.sqrt(Math.pow(centerToTargetMeters, 2.0) - Math.pow(centerToShooterMeters, 2.0));
        return shooterToTargetMeters;
    }

  public double getHubDistance() {
        return getShotDistance(DriveConstants.getHubPose().toPose2d().getTranslation());
    }

  public void printHubDistance() {
    System.out.println("Hub distance" + getHubDistance());
  }
  
  public Command alignDrive(CommandXboxController controller, Supplier<Pose2d> targetPoseSupplier) {

    return run( ()-> {
        double controllerVelX = MathUtil.applyDeadband(controller.getLeftY(),OperatorConstants.DRIVE_DEADBAND);
        double controllerVelY = MathUtil.applyDeadband(controller.getLeftX(),OperatorConstants.DRIVE_DEADBAND);

        Pose2d drivePose = getPose();
        Pose2d targetPose = targetPoseSupplier.get();
        double shooterOffset = -DriveConstants.shooterSideOffset;
        double targetDistance = drivePose.getTranslation().getDistance(targetPose.getTranslation());
        double shooterAngleRads = Math.acos(shooterOffset / targetDistance); 
        Rotation2d shooterAngle = Rotation2d.fromRadians(shooterAngleRads);
        Rotation2d offsetAngle = Rotation2d.kCCW_90deg.minus(shooterAngle);
        Rotation2d shooterAngleOffset = Rotation2d.fromDegrees(2);
        Rotation2d desiredAngle = offsetAngle.plus(drivePose.relativeTo(targetPose).getTranslation().getAngle()).plus(Rotation2d.k180deg).plus(shooterAngleOffset);
        Rotation2d currentAngle = drivePose.getRotation();
        Rotation2d deltaAngle = currentAngle.minus(desiredAngle);
        double wrappedAngleDeg = MathUtil.inputModulus(deltaAngle.getDegrees(), -180.0, 180.0);

        if (
            (Math.abs(wrappedAngleDeg) < DriveConstants.epsilonAngleToGoal.in(Degrees)) // if facing goal already
            && Math.hypot(controllerVelX, controllerVelY) < OperatorConstants.DRIVE_DEADBAND) {
               driveJoystick(controllerVelX, controllerVelY, 0, true); //TODO:IDK HOW FIELD RELATIVE WILL WOKR 
            } else {
            double rotationalRate = DriveConstants.rotationController.calculate(currentAngle.getRadians(), desiredAngle.getRadians());
              driveJoystick(controllerVelX, controllerVelY, rotationalRate, true);
        }
      });
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


  //TESTS 
  public Command hubDistance(){
    return run(
      () -> {
        printHubDistance();
      });
  }
  public Command ferryDistance(){
    return run(
      () -> {
        printFerryDistance();
      });
  }

           

  
}
