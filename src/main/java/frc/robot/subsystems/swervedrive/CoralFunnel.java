package frc.robot.subsystems.swervedrive;
import static frc.robot.Constants.CoralFunnelConstants.*;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class CoralFunnel extends SubsystemBase {
  SparkFlex m_CoralFunnelWheel;
  SparkFlexConfig config;

 public CoralFunnel() {
        m_CoralFunnelWheel = new SparkFlex(k_CoralFunnelWheelID, MotorType.kBrushless);

        config = new SparkFlexConfig();

        m_CoralFunnelWheel.configure(config.smartCurrentLimit(k_CoralFunnelVoltageLimit), null, null);

        m_CoralFunnelWheel.configure(config.idleMode(IdleMode.kBrake), null, PersistMode.kPersistParameters);

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


    public void f_setFunnelWheel(double speed) {
        m_CoralFunnelWheel.set(speed);
    }

    public void f_stop() {
        m_CoralFunnelWheel.set(0);
    }

}
