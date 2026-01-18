// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.configs.DriveConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

public class MaxSwerveModule extends SubsystemBase{ 
    
    private final SparkMax mDrivingSpark; 
    private final SparkMax mTurningSpark; 

    private final RelativeEncoder mDrivingEncoder; 
    private final AbsoluteEncoder mTurningEncoder;

    private final SparkClosedLoopController mDrivingClosedLoopController; 
    private final SparkClosedLoopController mTurningClosedLoopController; 

    private double mChassisAngularOffset = 0; 
    private SwerveModuleState mDesiredState = new SwerveModuleState(0.0, new Rotation2d()); 

   
    /**
     * Constructs a MAXSwerveModule and configures the driving and turning motor,
     * encoder, and PID controller. ONLY FOR REVMAXSWERVE MODULE 
     * built with NEOs, SPARKS MAX, and a Through Bore
     * Encoder.
     */

    public MaxSwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
      mDrivingSpark = new SparkMax(drivingCANId, MotorType.kBrushless); 
      mTurningSpark = new SparkMax(turningCANId, MotorType.kBrushless); 

      mDrivingEncoder = mDrivingSpark.getEncoder();
      mTurningEncoder = mTurningSpark.getAbsoluteEncoder();

      mDrivingClosedLoopController = mDrivingSpark.getClosedLoopController();
      mTurningClosedLoopController = mTurningSpark.getClosedLoopController(); 

      // Apply the respective configurations to the SPARKS. Reset parameters before
      //applying the configuration to bring the SPARK to a known good state. Persist
      // the settings to avoid losing them on a power cycle.

      mDrivingSpark.configure(DriveConfig.MAXSwerveModule.drivingConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters); 
      mTurningSpark.configure(DriveConfig.MAXSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

      mChassisAngularOffset = chassisAngularOffset; 
      
      mDesiredState.angle = new Rotation2d(mTurningEncoder.getPosition()); 
      mDrivingEncoder.setPosition(0); 
   }

   /**
   * @return The current state of the module = velocity (rad/s) + rotational angle (rad)
   */
   public SwerveModuleState getState() {
      return new SwerveModuleState(mDrivingEncoder.getVelocity(), 
        new Rotation2d(mTurningEncoder.getPosition() - mChassisAngularOffset)); 
      //encoder position relative to chassis corrected with offset of module 
   }
  
   /**
   * @return The current position of the module = position(m) + rotational angle (rad) 
   */
   public SwerveModulePosition getPosition(){
      return new SwerveModulePosition(mDrivingEncoder.getPosition(),
      new Rotation2d(mTurningEncoder.getPosition()- mChassisAngularOffset));
   }

   public void setDesiredState(SwerveModuleState desiredState){
      SwerveModuleState correctedDesiredState = new SwerveModuleState(); 
      correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond; //velocity from drive method
      correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(mChassisAngularOffset)); 
      //applies angular offset to desired state

      // Optimize the reference state to avoid spinning further than 90 degrees (takes shortest rotation)
      correctedDesiredState.optimize(new Rotation2d(mTurningEncoder.getPosition()));
      
      //set speed and rotational setpoint for driving and turning Sparkmaxs 
      mDrivingClosedLoopController.setSetpoint(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity); 
      mTurningClosedLoopController.setSetpoint(correctedDesiredState.angle.getRadians(), ControlType.kPosition); 

      mDesiredState = desiredState;
   }

   //zero swervemodule drive encoders 
   public void resetEncoders(){
      mDrivingEncoder.setPosition(0);
   }


}
