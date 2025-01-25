package frc.robot.subsystems.swervedrive;

import static frc.robot.Constants.AlgaeClawConstants.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class AlgaeClaw extends SubsystemBase {
    SparkFlex m_AlgaeClawTopWheel;
    SparkFlex m_AlgaeClawBottomWheel;

    SparkFlexConfig config;

    public AlgaeClaw() {
        m_AlgaeClawTopWheel = new SparkFlex(k_AlgaeClawTopID, MotorType.kBrushless);
        m_AlgaeClawBottomWheel = new SparkFlex(k_AlgaeClawBottomID, MotorType.kBrushless);

        config = new SparkFlexConfig();

        m_AlgaeClawTopWheel.configure(config.smartCurrentLimit(k_AlgaeClawVoltageLimit), null, null);
        m_AlgaeClawBottomWheel.configure(config.smartCurrentLimit(k_AlgaeClawVoltageLimit), null, null);

        m_AlgaeClawBottomWheel.configure(config.idleMode(IdleMode.kBrake), null, PersistMode.kPersistParameters);
        m_AlgaeClawTopWheel.configure(config.idleMode(IdleMode.kBrake), null, PersistMode.kPersistParameters);

    }

    public Command c_getAlgaeIntakeCommand() {
        return this.startEnd(
            // When the command is initialized, set the wheels to the intake speed values
            () -> {
              f_setAlgaeClawWheel(k_AlgaeClawIntakeSpeed);
            },
            // When the command stops, stop the wheels
            () -> {
              f_stop();
            });
    }

    public Command c_getAlgaeProcessorCommand() {
        return this.startEnd(
            // When the command is initialized, set the wheels to the intake speed values
            () -> {
              f_setAlgaeClawWheel(-k_AlgaeClawProcessorSpeed);
            },
            // When the command stops, stop the wheels
            () -> {
              f_stop();
            });
    }

    public Command c_getAlgaeBargeCommand() {
      return this.startEnd(
          // When the command is initialized, set the wheels to the intake speed values
          () -> {
            f_setAlgaeClawWheel(-k_AlgaeClawBargeSpeed);
          },
          // When the command stops, stop the wheels
          () -> {
            f_stop();
          });
  }



    public void f_setAlgaeClawWheel(double speed) {
        m_AlgaeClawTopWheel.set(speed);
        m_AlgaeClawBottomWheel.set(-speed);
    }

    public void f_stop() {
        m_AlgaeClawTopWheel.set(0);
        m_AlgaeClawBottomWheel.set(0);
    }

}
