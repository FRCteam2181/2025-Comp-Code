package frc.robot.subsystems;
import static frc.robot.Constants.CoralFunnelConstants.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import swervelib.motors.SparkFlexSwerve;


public class CoralFunnel extends SubsystemBase {
  SparkFlex m_CoralFunnelWheel;
  SparkFlex m_FunnelRotator;
  SparkFlexConfig config;

  LinearFilter currentFilter = LinearFilter.movingAverage(10);
  private double filteredCurrent;



 public CoralFunnel() {
        m_CoralFunnelWheel = new SparkFlex(k_CoralFunnelWheelID, MotorType.kBrushless);
        m_FunnelRotator = new SparkFlex(k_CoralRotatorID, MotorType.kBrushless);

        config = new SparkFlexConfig();

        m_CoralFunnelWheel.configure(config.smartCurrentLimit(k_CoralFunnelVoltageLimit), null, null);
        m_FunnelRotator.configure(config.smartCurrentLimit(k_CoralFunnelVoltageLimit), null, null);

        m_CoralFunnelWheel.configure(config.idleMode(IdleMode.kBrake), null, PersistMode.kPersistParameters);
        m_FunnelRotator.configure(config.idleMode(IdleMode.kBrake), null, PersistMode.kPersistParameters);

    }

    public Command c_getFunnelWheelCommand() {
        return this.startEnd(
            // When the command is initialized, set the wheels to the intake speed values
            () -> {
              f_setFunnelWheel(k_CoralFunnelSpeed);
            },
            // When the command stops, stop the wheels
            () -> {
              f_stop();
            });
    }

    public Command c_getFunnelWheelCommandback() {
        return this.startEnd(
            // When the command is initialized, set the wheels to the intake speed values
            () -> {
              f_setFunnelWheel(-k_CoralFunnelSpeed);
            },
            // When the command stops, stop the wheels
            () -> {
              f_stop();
            });
    }

    public Command c_FunnelRotateCommandUp() {
      return this.startEnd(
          // When the command is initialized, set the wheels to the intake speed values
          () -> {
            f_setFunnelRotate(k_FunnelRotateSpeed);
          },
          // When the command stops, stop the wheels
          () -> {
            f_stop();
          });
  }

  public Command c_FunnelRotateCommandDown() {
    return this.startEnd(
        // When the command is initialized, set the wheels to the intake speed values
        () -> {
          f_setFunnelRotate(-k_FunnelRotateSpeed);
        },
        // When the command stops, stop the wheels
        () -> {
          f_stop();
        });
}


public Command c_AutoCoralFunnelCommand() {

        Debouncer debounce = new Debouncer(1, Debouncer.DebounceType.kRising);
    // Open arms
    return runOnce(
            () -> {
              debounce.calculate(false);
            })
        // set the intake to cube intaking speed
        .andThen(
            run(() -> {
              f_setFunnelWheel(k_CoralFunnelSpeed);
                })
                // Wait until current spike is detected for more than 1s
                .until(() -> debounce.calculate(getFilteredCurrent() > 7)))
        // Reduce motor power to holding power
        .finallyDo(
            (interrupted) -> {
                f_stop();
            });

    }





    public void f_setFunnelWheel(double speed) {
        m_CoralFunnelWheel.set(speed);
    }

    public void f_stop() {
        m_CoralFunnelWheel.set(0);
        m_FunnelRotator.set(0);
    }

    public void f_setFunnelRotate (double speed) {
      m_FunnelRotator.set(speed);
    }



    public double getFilteredCurrent() {
      return filteredCurrent;
    }

  public double getCurrent() {
      return m_CoralFunnelWheel.getOutputCurrent();
    }
  
    @Override
    public void periodic() {
      filteredCurrent = currentFilter.calculate(getCurrent());
    }



}
