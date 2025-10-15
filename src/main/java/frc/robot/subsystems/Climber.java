package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import static frc.robot.Constants.ClimberConstants.*;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs.ClimberConfigs; 

public class Climber extends SubsystemBase {
    SparkMax m_climber; 

    public Climber() {
        m_climber  = new SparkMax(k_climberID, MotorType.kBrushless);

        m_climber.configure(
          ClimberConfigs.climberConfig, 
          ResetMode.kResetSafeParameters, 
          PersistMode.kPersistParameters);

    }
    public void f_setClimberSpeed(double speed) {
        m_climber.set(speed);
    }

    public void f_stop() {
        m_climber.set(0);
    }

    public Command c_GetClimberUpCommand() {
        return this.startEnd(
            // When the command is initialized, set the wheels to the intake speed values
            () -> {
              f_setClimberSpeed(k_climberSpeedUp);
            },
            // When the command stops, stop the wheels
            () -> {
              f_stop();
            });
    }

    public Command c_GetClimberDownCommand() {
        return this.startEnd(
            // When the command is initialized, set the wheels to the intake speed values
            () -> {
              f_setClimberSpeed(-k_ClimberSpeedDown);
            },
            // When the command stops, stop the wheels
            () -> {
              f_stop();
            });
    }
}
