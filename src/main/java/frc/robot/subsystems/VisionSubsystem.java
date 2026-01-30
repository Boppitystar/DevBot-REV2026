package frc.robot.subsystems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;;

public class VisionSubsystem extends SubsystemBase {

    public VisionSubsystem(){ 
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

    public Command estimateDistanceToHub(){
        return run(
        () -> {
        getDistanceToHub();
         });
    }
    
    @Override
    public void periodic(){
        
    }
    
}
