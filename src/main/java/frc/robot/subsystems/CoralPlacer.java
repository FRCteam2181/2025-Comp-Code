package frc.robot.subsystems;

import static frc.robot.Constants.CoralPlacerConstants.*;
import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class CoralPlacer extends SubsystemBase{

    //SparkFlex m_CorPWheelRight;
    SparkFlex m_CorPWheelLeft;

    LinearFilter currentFilter = LinearFilter.movingAverage(10);
    private double filteredCurrent;

    SparkFlexConfig config;

    public CoralPlacer() {
        m_CorPWheelLeft = new SparkFlex(k_CoralWheelLeftID, MotorType.kBrushless);
        //m_CorPWheelRight = new SparkFlex(k_CoralWheelRightID, MotorType.kBrushless);

        config = new SparkFlexConfig();

        m_CorPWheelLeft.configure(config.smartCurrentLimit(k_CoralPlacerVoltageLimit), null, null);
        //m_CorPWheelRight.configure(config.smartCurrentLimit(k_CoralPlacerVoltageLimit), null, null);

        m_CorPWheelLeft.configure(config.idleMode(IdleMode.kBrake), null, PersistMode.kPersistParameters);
        m_CorPWheelLeft.configure(config.idleMode(IdleMode.kBrake), null, PersistMode.kPersistParameters);
        
    }

    public Command c_getCoralPlacerL1Command() {
        return this.runEnd(() -> {
            f_setCoralWheels(k_CoralPlacerSpeedL1);
        }, 
        
        () -> {
            f_stop();
        });
    }

    public Command c_AutoCoralPlacerCommand() {

        Debouncer debounce = new Debouncer(1, Debouncer.DebounceType.kRising);
    // Open arms
    return runOnce(
            () -> {
              debounce.calculate(false);
            })
        // set the intake to cube intaking speed
        .andThen(
            run(() -> {
                f_setCoralWheels(k_CoralPlacerSpeedL1);
                })
                // Wait until current spike is detected for more than 1s
                .until(() -> debounce.calculate(getFilteredCurrent() > 7)))
        // Reduce motor power to holding power
        .finallyDo(
            (interrupted) -> {
                f_stop();
            });

    }



    public Command c_getCoralPlacerGenCommand() {
        return this.startEnd(() -> {
            f_setCoralWheels(k_CoralPlacerSpeedL1);
        }, 
        
        () -> {
            f_stop();
        });
    }

    public void f_setCoralWheels(double speed) {
        m_CorPWheelLeft.set(speed);
        //m_CorPWheelRight.set(-speed*.5);
    }

    public void f_stop() { 
        m_CorPWheelLeft.set(0);
        //m_CorPWheelRight.set(0); 
    }

    public double getFilteredCurrent() {
        return filteredCurrent;
      }

    public double getCurrent() {
        return m_CorPWheelLeft.getOutputCurrent();
      }
    
      @Override
      public void periodic() {
        filteredCurrent = currentFilter.calculate(getCurrent());
      }


}
