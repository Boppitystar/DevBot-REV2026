package frc.robot.commands; 

import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;


public class AlignToTagRelative extends Command {
  
  private PIDController mXController, mYController, mRotController;
  private boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer;
  private DriveSubsystem mDrivebase;
  private double tagID = -1;

  //command requires left or right side of april tag 
  public AlignToTagRelative(boolean isRightScore, DriveSubsystem Drivebase) {
    mXController = new PIDController(Constants.AutoConstants.X_TAG_ALIGNMENT_P, 0.0, 0);  // Vertical movement
    mYController = new PIDController(Constants.AutoConstants.Y_TAG_ALIGNMENT_P, 0.0, 0);  // Horitontal movement
    mRotController = new PIDController(Constants.AutoConstants.ROT_TAG_ALIGNMENT_P, 0, 0);  // Rotation
   
    this.isRightScore = isRightScore; 
    this.mDrivebase = Drivebase;
    addRequirements(mDrivebase); //only one drive subsystem instanstiated 
  }

  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();


    //TODO: find setpoints for limelight april tag test
    mRotController.setSetpoint(Constants.AutoConstants.ROT_SETPOINT_TAG_ALIGNMENT);
    mRotController.setTolerance(Constants.AutoConstants.ROT_TOLERANCE_TAG_ALIGNMENT);

    mXController.setSetpoint(Constants.AutoConstants.X_SETPOINT_TAG_ALIGNMENT);
    mXController.setTolerance(Constants.AutoConstants.X_TOLERANCE_TAG_ALIGNMENT);

    mYController.setSetpoint(isRightScore ? Constants.AutoConstants.Y_SETPOINT_TAG_ALIGNMENT
    : -Constants.AutoConstants.Y_SETPOINT_TAG_ALIGNMENT);
    mYController.setTolerance(Constants.AutoConstants.Y_TOLERANCE_TAG_ALIGNMENT);

    tagID = LimelightHelpers.getFiducialID("");
  }

  @Override
  public void execute() {
    if (LimelightHelpers.getTV("") && LimelightHelpers.getFiducialID("") == tagID) {
      this.dontSeeTagTimer.reset();

      double[] positions = LimelightHelpers.getBotPose_TargetSpace("");
      SmartDashboard.putNumber("x", positions[2]);

      //PID controllers for position to speed 
      double xOutput = -mXController.calculate(positions[2]);
      double yOutput = -mYController.calculate(positions[0]);
      double rotOutput= -mRotController.calculate(positions[4]); //TODO:test why they are negative  
      
      SmartDashboard.putNumber("xspeed", xOutput);

      //feeds the drive method the controller output which is then clamped (theory can be wrong)
      mDrivebase.driveChassisSpeeds(-xOutput, yOutput, rotOutput, false);

      //checks if aligned, if not restarts timer to restart process
      if (!mRotController.atSetpoint() || !mYController.atSetpoint() || !mXController.atSetpoint()) {
        stopTimer.reset();
      }
      } else {
        mDrivebase.driveChassisSpeeds(0,0, 0, false);
      }

    SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
    mDrivebase.driveChassisSpeeds(0,0, 0, false); //TODO: test to see if true/change back to field relative works 
  }

  @Override
  public boolean isFinished() {
    // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
    return this.dontSeeTagTimer.hasElapsed(Constants.AutoConstants.DONT_SEE_TAG_WAIT_TIME) ||
        stopTimer.hasElapsed(Constants.AutoConstants.POSE_VALIDATION_TIME);
  }
}






