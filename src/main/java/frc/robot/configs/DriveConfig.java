package frc.robot.configs;

import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ModuleConstants;

public final class DriveConfig {
    public static final class MAXSwerveModule{
        public static final SparkMaxConfig drivingConfig = new SparkMaxConfig(); 
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig(); 

        static {
            //use module constance to calc conversion factors + feed forward gain 
            double drivingFactor = ModuleConstants.WHEEL_DIAMETER_METERS * Math.PI
                    /ModuleConstants.DRIVING_MOTOR_REDUCTION;
            double turningFactor = 2* Math.PI;  
            double drivingVelocityFeedForward = 1 / ModuleConstants.DRIVE_WHEEL_FREE_SPEED_RPS;

            //Current limit should ALWAYS be below 50 for driving and 20 for turning 
            drivingConfig
            .idleMode(IdleMode.kBrake).smartCurrentLimit(50);
            drivingConfig.encoder
            .positionConversionFactor(drivingFactor)
            .velocityConversionFactor(drivingFactor/60.0);
            drivingConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(0.04, 0, 0)  
            .outputRange(-1,1)  //speed setpoint to actual velocity  
            .feedForward.kV(drivingVelocityFeedForward);
            
            
            turningConfig
            .idleMode(IdleMode.kBrake).smartCurrentLimit(20);
            turningConfig.absoluteEncoder
            //invert as output shaft rotates oppositie to steering motor in MAXSwerveModule
            .inverted(true)  
            .positionConversionFactor(turningFactor) //radians 
            .velocityConversionFactor(turningFactor/60.0) //rad/s 
            .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoder);
            turningConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(1, 0, 0)
            .outputRange(-1, 1)
            // Enable PID wrap around for the turning motor. This will allow the PID
            // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
            // to 10 degrees will go through 0 rather than the other direction which is a
            // longer route.
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0, turningFactor);
        }


    }
}
