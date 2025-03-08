package frc.robot;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.AlgaeRotatorConstants;
import frc.robot.Constants.ElevatorConstants;

public class Configs {
    public static final class ElevatorConfig {

        public static final SparkMaxConfig elevatorConfig = new SparkMaxConfig();
        public static final SparkMaxConfig globalConfig = new SparkMaxConfig();
        public static final SparkMaxConfig elevatorFollowerConfig = new SparkMaxConfig();
    
        static {
    
          // Configure basic settings of the elevator motor
          globalConfig
            .idleMode(IdleMode.kBrake)
            
            .smartCurrentLimit(ElevatorConstants.kElevatorCurrentLimit)
            .closedLoopRampRate(ElevatorConstants.kElevatorRampRate);

   
          elevatorConfig
          .apply(globalConfig);
    
          elevatorFollowerConfig
            .apply(globalConfig);
    
        }       
    }
    public static final SparkMaxConfig algaeRotatorConfig = new SparkMaxConfig();



    static {
    
      // Configure basic settings of the elevator motor
      algaeRotatorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(AlgaeRotatorConstants.kAlgaeArmStallCurrentLimitAmps)
        .softLimit.reverseSoftLimit(0)
        .reverseSoftLimitEnabled(true)
        .forwardSoftLimit(30)
        .forwardSoftLimitEnabled(true);
      
    

      

    }       



}
