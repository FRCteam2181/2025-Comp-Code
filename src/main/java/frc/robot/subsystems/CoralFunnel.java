package frc.robot.subsystems;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static frc.robot.Constants.CoralFunnelConstants.*;

import com.revrobotics.spark.SparkFlex;

//import com.playingwithfusion.TimeOfFlight;
//import com.playingwithfusion.TimeOfFlight.RangingMode;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.math.filter.Debouncer.DebounceType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.math.filter.LinearFilter;
import frc.robot.Robot;
import frc.robot.Configs.CoralFunnelConfigs;
import frc.robot.Configs.FunnelRotatorConfigs;
//import frc.robot.Constants.Constants.CoralFunnelConstants;
import edu.wpi.first.math.filter.Debouncer;

public class CoralFunnel extends SubsystemBase {
  SparkFlex m_CoralFunnelWheel;
  SparkFlex m_FunnelRotator;

  //private final TimeOfFlight coralSensor;
    private boolean scoreReady;
    private Debouncer risingDebouncer;


  LinearFilter currentFilter = LinearFilter.movingAverage(10);
  private double filteredCurrent;



 public CoralFunnel() {
        m_CoralFunnelWheel = new SparkFlex(k_CoralFunnelWheelID, MotorType.kBrushless);
        m_FunnelRotator = new SparkFlex(k_CoralRotatorID, MotorType.kBrushless);

        m_CoralFunnelWheel.configure(
          CoralFunnelConfigs.coralIntakeConfig, 
          ResetMode.kResetSafeParameters, 
          PersistMode.kPersistParameters);
          
        m_FunnelRotator.configure(
          FunnelRotatorConfigs.funnelRotatorConfig, 
          ResetMode.kResetSafeParameters, 
          PersistMode.kPersistParameters);


      // coralSensor = new TimeOfFlight(CoralFunnelConstants.coralSensorId);
      //   scoreReady = false;
      //   risingDebouncer = new Debouncer(0.01, DebounceType.kRising);

      //   coralSensor.setRangingMode(RangingMode.Medium, 24);

      //   coralInSensor().and(scoreReady().negate()).and(RobotModeTriggers.teleop()).whileTrue(loadCoral());


    }


 public void setScoreReady(boolean b) {
        scoreReady = b;
    }


    // public Trigger coralInSensor() {
    //     if(Robot.isReal()) {
    //         return new Trigger(() -> risingDebouncer.calculate(coralSensor.getRange() <= CoralFunnelConstants.coralDistanceThreshold));
    //     }
    //     return new Trigger(() -> false);

    // }

    // public Trigger scoreReady() {
    //     return new Trigger(() -> scoreReady || Robot.isSimulation());
    // }


  //   public Command loadCoral() {
  //     return Commands.run(() -> {
  //         f_setFunnelWheel(k_CoralFunnelSpeed);
  //     }, this).withInterruptBehavior(InterruptionBehavior.kCancelIncoming).until(coralInSensor()).finallyDo(() -> {
  //         scoreReady = true;
  //         f_stop();
  //     });
  // }


// public Command unloadCoral() {
//   return Commands.run(() -> {
//       f_setFunnelWheel(k_CoralFunnelSpeed);
//   }, this).withInterruptBehavior(InterruptionBehavior.kCancelIncoming).until(coralInSensor().negate()).finallyDo(() -> {
//       scoreReady = false;
//       f_stop();
//   });
// }



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
              f_setFunnelWheel(-k_CoralFunnelSpeed/2);
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
