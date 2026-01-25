package frc.robot.commands; 

import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class NewAlignToTagRelative extends Command {
  
  private PIDController mXController, mYController, mRotController;
  private boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer;
  private DriveSubsystem mDrivebase;
  private VisionSubsystem mVisionSubsystem;
  //command requires left or right side of april tag 
  public NewAlignToTagRelative(DriveSubsystem Drivebase) {
    mXController = new PIDController(Constants.AutoConstants.X_TAG_ALIGNMENT_P, 0.0, 0);  // Vertical movement
    mYController = new PIDController(Constants.AutoConstants.Y_TAG_ALIGNMENT_P, 0.0, 0);  // Horitontal movement
    mRotController = new PIDController(Constants.AutoConstants.ROT_TAG_ALIGNMENT_P, 0, 0);  // Rotation
   
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

  }

  @Override
  public void execute() {
    double turn = mVisionSubsystem.getTurnNeed();
    mDrivebase.driveChassisSpeeds(0,0, turn, true);

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






