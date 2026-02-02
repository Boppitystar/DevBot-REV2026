package frc.robot.commands; 

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
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
  private double TURN;

  //command requires left or right side of april tag 
  public AlignToTagRelative(DriveSubsystem Drivebase) {
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
    TURN = Units.degreesToRadians(mDrivebase.getTurn());
    double x = MathUtil.clamp(100*TURN, -Math.PI, Math.PI);
    mDrivebase.driveChassisSpeeds(0, 0, x, false);
    SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
    mDrivebase.driveChassisSpeeds(0,0, 0, false); //TODO: test to see if true/change back to field relative works 
  }

  @Override
  public boolean isFinished() {
      return TURN < 1.5 && TURN > - 1.5;
  }
}






