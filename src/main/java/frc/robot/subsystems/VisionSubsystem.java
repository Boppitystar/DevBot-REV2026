package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import limelight.Limelight;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.Orientation3d;
import limelight.networktables.LimelightPoseEstimator.EstimationMode;
import limelight.networktables.LimelightSettings.LEDMode;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;

public class VisionSubsystem extends SubsystemBase {


    public Limelight limelight; 

    private LimelightPoseEstimator poseEstimator;

    public VisionSubsystem(){ 
      limelight = new Limelight("limelight");
     
      // Set the limelight to use Pipeline LED control, with the Camera offset of 0, and save.
      limelight.getSettings()
         .withLimelightLEDMode(LEDMode.PipelineControl)
         .withCameraOffset(Pose3d.kZero)
         .save();

      poseEstimator = limelight.createPoseEstimator(EstimationMode.MEGATAG2);

    }

    public double autoAlignRotation(){
        double targetAngularVelocity = 0.08*LimelightHelpers.getTX("limelight");
        return targetAngularVelocity; 
    }

    public void getDistanceToHub(){
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight"); 
        NetworkTableEntry ty = table.getEntry("ty"); //gets the vertical angle to target 

        double targetVerticalOffsetAngle = ty.getDouble(0.0); 

        double angleToTargetDegrees = VisionConstants.LL_MOUNT_ANGLE_DEG + targetVerticalOffsetAngle; 

        double angleToTargetRad = Units.degreesToRadians(angleToTargetDegrees);

       // double angleToTargetRad2 = angleToTargetDegrees* (3.14159/180.0);

        double distanceToGoal = (VisionConstants.TARGET_HEIGHT_IN - VisionConstants.LL_LENS_HEIGHT_IN)/(Math.tan(angleToTargetRad));
        //return distanceToGoal; 
        System.out.println("distance:" + distanceToGoal);
    }

    public double getTA(){
        return LimelightHelpers.getTA("limelight");
    }

    public double getDistanceToTag(){
        double ta = getTA();
        double scale = 18669.31;
        double distance = (Math.pow(scale/ta, 1/1.93183))/100 - 0.30; 

        return distance; 
    
    }

    public Command estimateDistanceToHub(){
        return run(
        () -> {
        getDistanceToHub();
         });
    }
    
    @Override
    public void periodic(){
        SmartDashboard.putNumber("Testing/distance", getDistanceToTag());
        SmartDashboard.putNumber("Testing/targetArea", getTA());
    }
    
}
