package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.AlgaeRotatorConstants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.CoralPlacerConstants;
import frc.robot.Constants.CoralFunnelConstants;
import frc.robot.Constants.climberConstants;
import frc.robot.Constants.AlgaeClawConstants;


public class Configs {
  public static final class ElevatorConfigs {

    public static final SparkMaxConfig elevatorConfig = new SparkMaxConfig();
    public static final SparkMaxConfig baseElevatorConfig = new SparkMaxConfig();
    public static final SparkMaxConfig elevatorFollowerConfig = new SparkMaxConfig();

    static {

      // Configure basic settings of the elevator motor
      baseElevatorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(ElevatorConstants.kElevatorCurrentLimit)
        .closedLoopRampRate(ElevatorConstants.kElevatorRampRate);


      elevatorConfig
        .apply(baseElevatorConfig);

      elevatorFollowerConfig
        .apply(baseElevatorConfig);

    }       
  }



  public static final class AlgaeClawConfigs {

    public static final SparkMaxConfig algaeRotatorConfig = new SparkMaxConfig();
    public static final SparkMaxConfig baseAlgaeClawConfig = new SparkMaxConfig();
    
    static {

      // Configure basic settings of the Algae Claw Rotation motor
      algaeRotatorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(AlgaeRotatorConstants.kAlgaeArmStallCurrentLimitAmps)
        .softLimit.reverseSoftLimit(0)
        .reverseSoftLimitEnabled(true)
        .forwardSoftLimit(30)
        .forwardSoftLimitEnabled(true);
  
      //Configure base settings for the wheels on the Algae Claw
      baseAlgaeClawConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(AlgaeClawConstants.k_AlgaeClawVoltageLimit);

    }  

  }

  public static final class CoralPlacerConfigs {

    public static final SparkFlexConfig baseCoralPlacerConfig = new SparkFlexConfig();

    static {

      //Configure the base settings for the Coral Placer Motors
      baseCoralPlacerConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(CoralPlacerConstants.k_CoralPlacerVoltageLimit);

    }

  }

  public static final class CoralFunnelConfigs {
    
    public static final SparkFlexConfig coralIntakeConfig = new SparkFlexConfig();
    public static final SparkFlexConfig funnelRotatorConfig = new SparkFlexConfig();

    static {

      //Configure the base settings for the Coral Intake Motor
      coralIntakeConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(CoralFunnelConstants.k_CoralFunnelVoltageLimit);

      //Configure the base settings for the coral funnel rotator motor
      funnelRotatorConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(CoralFunnelConstants.k_CoralFunnelVoltageLimit);
    
    }

  }

  public static final class ClimberConfigs {
    
    public static final SparkMaxConfig climberConfig = new SparkMaxConfig();

    static {

      //Configure the base settings for the Climber motor
      climberConfig
        .idleMode(IdleMode.kBrake)
        .smartCurrentLimit(climberConstants.k_climberVoltageLimit);

    }
    
    
  }

}
