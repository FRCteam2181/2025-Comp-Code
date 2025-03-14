package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ResetMode;

import frc.robot.Configs;
import frc.robot.Constants.AlgaeRotatorConstants;


public class AlgaeRotator extends SubsystemBase {

  SparkMax m_AlgaeRotator;
  RelativeEncoder rotatorEncoder;


  public AlgaeRotator() {
      m_AlgaeRotator = new SparkMax(AlgaeRotatorConstants.k_AlgaeClawRotatorID, MotorType.kBrushless);
      rotatorEncoder = m_AlgaeRotator.getEncoder();
      
      
      m_AlgaeRotator.configure(
            Configs.algaeRotatorConfig, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters);

      seedRotatorMotorPosition();
    }

     /**
   * Seed the algae claw motor encoder with the sensed position from the LaserCAN which tells us the height of the
   * elevator.
   */
  public void seedRotatorMotorPosition()
  {
    rotatorEncoder.setPosition(0);
  
  }


  public Command c_GetAlgeaRotateUpCommand() {
      return this.startEnd(
          // When the command is initialized, set the wheels to the intake speed values
          () -> {
            f_setAlgaeRotateSpeed(AlgaeRotatorConstants.k_AlgaeClawRotateSpeed);
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
            f_setAlgaeRotateSpeed(-AlgaeRotatorConstants.k_AlgaeClawRotateSpeed);
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
  
 