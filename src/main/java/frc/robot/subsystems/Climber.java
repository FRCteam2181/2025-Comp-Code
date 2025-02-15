package frc.robot.subsystems;

import static frc.robot.Constants.climberConstants.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase; 

public class Climber extends SubsystemBase {
    SparkMax m_climber; 

    SparkMaxConfig config;
    public Climber() {
        m_climber  = new SparkMax(m_climberID, MotorType.kBrushless);

        config = new SparkMaxConfig();

        m_climber.configure(config.smartCurrentLimit(m_climberVoltageLimit), null, null);
        m_climber.configure(config.idleMode(IdleMode.kBrake), null, PersistMode.kPersistParameters);

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
              f_setClimberSpeed(m_climberSpeed);
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
              f_setClimberSpeed(-m_climberSpeed);
            },
            // When the command stops, stop the wheels
            () -> {
              f_stop();
            });
    }
}
