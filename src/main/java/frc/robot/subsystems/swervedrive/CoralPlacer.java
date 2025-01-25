package frc.robot.subsystems.swervedrive;

import static frc.robot.Constants.CoralPlacerConstants.*;

import java.io.ObjectInputFilter.Config;
import java.lang.ProcessBuilder.Redirect.Type;

import com.revrobotics.spark.config.*;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class CoralPlacer extends SubsystemBase{

    SparkFlex m_CorPWheelRight;
    SparkFlex m_CorPWheelLeft;

    SparkFlexConfig config;

    public CoralPlacer() {
        m_CorPWheelLeft = new SparkFlex(k_CoralWheelLeftID, MotorType.kBrushless);
        m_CorPWheelRight = new SparkFlex(k_CoralWheelRightID, MotorType.kBrushless);

        config = new SparkFlexConfig();

        m_CorPWheelLeft.configure(config.smartCurrentLimit(k_CoralPlacerVoltageLimit), null, null);
        m_CorPWheelRight.configure(config.smartCurrentLimit(k_CoralPlacerVoltageLimit), null, null);

        m_CorPWheelLeft.configure(config.idleMode(IdleMode.kBrake), null, PersistMode.kPersistParameters);
        m_CorPWheelLeft.configure(config.idleMode(IdleMode.kBrake), null, PersistMode.kPersistParameters);
    }

    public Command c_getCoralPlacerL1Command() {
        return this.startEnd(() -> {
            f_setCoralWheels(k_CoralPlacerSpeedL1);
        }, 
        
        () -> {
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
        m_CorPWheelRight.set(-speed*.5);
    }

    public void f_stop() { 
        m_CorPWheelLeft.set(0);
        m_CorPWheelRight.set(0); 
    }

}
