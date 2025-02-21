// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;


// import static edu.wpi.first.units.Units.Inches;
// import static edu.wpi.first.units.Units.Meters;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.config.SparkMaxConfig;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.controller.ElevatorFeedforward;
// import edu.wpi.first.math.controller.ProfiledPIDController;
// import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.Configs;


// import frc.robot.Constants;
// import frc.robot.Constants.ElevatorConstants;


// public class ElevatorSubsystem extends SubsystemBase
// {

//   // This gearbox represents a gearbox containing 1 Neo
//   private SparkMax m_ElevatorLeft;
//   private SparkMax m_ElevatorRight;
//   private RelativeEncoder elevatorEncoder;
//   SparkMaxConfig config;

//     public ElevatorSubsystem() {
//       m_ElevatorLeft  = new SparkMax(ElevatorConstants.k_ElevatorLeftID, MotorType.kBrushless);
//       m_ElevatorRight = new SparkMax(ElevatorConstants.k_ElevatorRightID, MotorType.kBrushless);
//         config = new SparkMaxConfig();

//         m_ElevatorRight.configure(config.smartCurrentLimit(40), null, null);
//         m_ElevatorLeft.configure(config.follow(ElevatorConstants.k_ElevatorRightID), null, PersistMode.kPersistParameters);

//     }
//     public void f_setElevatorSpeed(double speed) {
//       m_ElevatorRight.set(speed);
//     }

//     public void f_stop() {
//       m_ElevatorRight.set(0);
//     }

//     public Command c_GetElevatorUpCommand() {
//         return this.startEnd(
//             // When the command is initialized, set the wheels to the intake speed values
//             () -> {
//               f_setElevatorSpeed(.35);
//             },
//             // When the command stops, stop the wheels
//             () -> {
//               f_stop();
//             });
//     }

//     public Command c_GetElevatorDownCommand() {
//         return this.startEnd(
//             // When the command is initialized, set the wheels to the intake speed values
//             () -> {
//               f_setElevatorSpeed(-.35);
//             },
//             // When the command stops, stop the wheels
//             () -> {
//               f_stop();
//             });
//     }

// }