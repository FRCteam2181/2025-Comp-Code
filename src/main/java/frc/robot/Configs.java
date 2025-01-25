package frc.robot;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class Configs {
    public static final class ElevatorConfig {

        public static final SparkMaxConfig elevatorConfig = new SparkMaxConfig();
        public static final SparkMaxConfig globalConfig = new SparkMaxConfig();
        public static final SparkMaxConfig elevatorFollowerConfig = new SparkMaxConfig();
    
        static {
    
          // Configure basic settings of the elevator motor
          globalConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(50)
            .voltageCompensation(12);
    
          /*
           * Configure the reverse limit switch for the elevator. By enabling the limit switch, this
           * will prevent any actuation of the elevator in the reverse direction if the limit switch is
           * pressed.
           */
          elevatorConfig
              .limitSwitch
              .reverseLimitSwitchEnabled(true)
              .reverseLimitSwitchType(Type.kNormallyOpen);
    
          /*
           * Configure the closed loop controller. We want to make sure we set the
           * feedback sensor as the primary encoder.
           */
          elevatorConfig
              .apply(globalConfig)
              .closedLoop
              .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
              // Set PID values for position control
              .p(0.1)
              .outputRange(-1, 1)
              .maxMotion
              // Set MAXMotion parameters for position control
              .maxVelocity(4200)
              .maxAcceleration(6000)
              .allowedClosedLoopError(0.5);
    
          elevatorFollowerConfig
            .apply(globalConfig);
    
        }       
    }
}
