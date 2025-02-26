package frc.robot.subsystems;

import static frc.robot.Constants.AlgaeClawConstants.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class AlgaeClaw extends SubsystemBase {
    SparkMax m_AlgaeClawTopWheel;
    SparkMax m_AlgaeClawBottomWheel;

    SparkMaxConfig config;

      LinearFilter currentFilter = LinearFilter.movingAverage(10);
  private double filteredCurrent;

    public AlgaeClaw() {
        m_AlgaeClawTopWheel = new SparkMax(k_AlgaeClawTopID, MotorType.kBrushless);
        m_AlgaeClawBottomWheel = new SparkMax(k_AlgaeClawBottomID, MotorType.kBrushless);

        config = new SparkMaxConfig();

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
public Command c_AutoAlgaeIntakeCommand() {

        Debouncer debounce = new Debouncer(1, Debouncer.DebounceType.kRising);
    // Open arms
    return runOnce(
            () -> {
              debounce.calculate(false);
            })
        // set the intake to cube intaking speed
        .andThen(
            run(() -> {
              f_setAlgaeClawWheel(k_AlgaeClawIntakeSpeed);
                })
                // Wait until current spike is detected for more than 1s
                .until(() -> debounce.calculate(getFilteredCurrent() > 7)))
        // Reduce motor power to holding power
        .finallyDo(
            (interrupted) -> {
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

    public double getFilteredCurrent() {
      return filteredCurrent;
    }

  public double getCurrent() {
      return m_AlgaeClawBottomWheel.getOutputCurrent();
    }
  
    @Override
    public void periodic() {
      filteredCurrent = currentFilter.calculate(getCurrent());
    }
}
