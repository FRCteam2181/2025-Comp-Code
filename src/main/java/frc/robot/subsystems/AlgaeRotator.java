package frc.robot.subsystems;


import static frc.robot.Constants.AlgaeClawConstants.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AlgaeRotator extends SubsystemBase {
    SparkMax m_AlgaeRotator;

    SparkMaxConfig config;

    public AlgaeRotator() {
        m_AlgaeRotator = new SparkMax(k_AlgaeClawRotator, MotorType.kBrushless);

        config = new SparkMaxConfig();

        m_AlgaeRotator.configure(config.smartCurrentLimit(k_AlgaeClawVoltageLimit), null, null);
        m_AlgaeRotator.configure(config.idleMode(IdleMode.kBrake), null, PersistMode.kPersistParameters);
    }

    public Command c_GetAlgeaRotateUpCommand() {
        return this.startEnd(
            // When the command is initialized, set the wheels to the intake speed values
            () -> {
              f_setAlgaeRotateSpeed(k_AlgaeClawRotateSpeed);
            },
            // When the command stops, stop the wheels
            () -> {
              f_stop();
            });
    }

    public Command c_GetAlgeaRotateDownCommand() {
        return this.startEnd(
            // When the command is initialized, set the wheels to the intake speed values
            () -> {
              f_setAlgaeRotateSpeed(-k_AlgaeClawRotateSpeed);
            },
            // When the command stops, stop the wheels
            () -> {
              f_stop();
            });
    }

    public void f_setAlgaeRotateSpeed(double speed) {
        m_AlgaeRotator.set(speed);
    }

    public void f_stop() {
        m_AlgaeRotator.set(0);
    }

}
