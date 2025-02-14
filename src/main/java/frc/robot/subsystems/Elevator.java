// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.RobotController;
// import edu.wpi.first.wpilibj.smartdashboard.*;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Configs;
// //import java.util.concurrent.PriorityBlockingQueue;

// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.spark.SparkClosedLoopController;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.SparkMaxAlternateEncoder;
// import com.revrobotics.spark.SparkBase.ControlType;
// import com.revrobotics.spark.SparkBase.PersistMode;
// import com.revrobotics.spark.SparkBase.ResetMode;
// //import com.revrobotics.spark.config.SparkMaxConfig;
// //import com.revrobotics.spark.ClosedLoopSlot;
// //import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

// import frc.robot.Constants.ElevatorConstants;

// public class Elevator extends SubsystemBase {
//         private SparkMax m_ElevatorLeft;
//         private SparkMax m_ElevatorRight;
//         private SparkClosedLoopController elevatorClosedLoopController;
//         private RelativeEncoder elevatorEncoder;


//         private boolean wasResetByButton = false;
//         private boolean wasResetByLimit = false;
//         private double elevatorCurrentTarget = ElevatorConstants.k_FeederStation;

//         public enum Setpoint {
            
//             k_FeederStation,
//             k_L1,
//             k_L2,
//             k_L3,
//             k_L4,
//             k_AGround,
//             k_A1,
//             k_A2,
//             k_Processor,
//             k_Net;
//         }
    
//     public Elevator() {

//         m_ElevatorLeft = new SparkMax(ElevatorConstants.k_ElevatorLeftID, MotorType.kBrushless);
//         m_ElevatorRight = new SparkMax(ElevatorConstants.k_ElevatorRightID, MotorType.kBrushless);
//         elevatorClosedLoopController = m_ElevatorLeft.getClosedLoopController();
//         elevatorEncoder = m_ElevatorLeft.getEncoder();
    
//         m_ElevatorRight.configure(
//             Configs.ElevatorConfig.elevatorFollowerConfig.follow(ElevatorConstants.k_ElevatorLeftID), 
//             ResetMode.kResetSafeParameters, 
//             PersistMode.kPersistParameters);

//         m_ElevatorLeft.configure( 
//             Configs.ElevatorConfig.elevatorConfig, 
//             ResetMode.kResetSafeParameters, 
//             PersistMode.kPersistParameters);
    
//         elevatorEncoder.setPosition(0);
        
//     }

//     public Command setSetpointCommand(Setpoint setpoint) {
//         return this.runOnce(
//             () -> {
//               switch (setpoint) {
//                 case k_FeederStation:
//                   elevatorCurrentTarget = ElevatorConstants.k_FeederStation;
//                   break;
//                 case k_L1:
//                   elevatorCurrentTarget = ElevatorConstants.k_L1;
//                   break;
//                 case k_L2:
//                   elevatorCurrentTarget = ElevatorConstants.k_L2;
//                   break;
//                 case k_L3:
//                   elevatorCurrentTarget = ElevatorConstants.k_L3;
//                   break;
//                 case k_L4:
//                   elevatorCurrentTarget = ElevatorConstants.k_L4;
//                   break;
//                 case k_A1:
//                   elevatorCurrentTarget = ElevatorConstants.k_A1;
//                   break;
//                 case k_A2:
//                   elevatorCurrentTarget = ElevatorConstants.k_A2;
//                   break;
//                 case k_AGround:
//                   elevatorCurrentTarget = ElevatorConstants.k_AGround;
//                   break;
//                 case k_Processor:
//                   elevatorCurrentTarget = ElevatorConstants.k_Processor;
//                   break;
//                 case k_Net:
//                   elevatorCurrentTarget = ElevatorConstants.k_Net;
//                   break;
//               }
//             });
//       }

//     private void f_moveToSetpoint() {
//         elevatorClosedLoopController.setReference( elevatorCurrentTarget, ControlType.kMAXMotionPositionControl); 
//     }

//     // private void f_zeroElevatorOnLimitSwitch() {
//     //     if (!wasResetByLimit && m_ElevatorLeft.getReverseLimitSwitch().isPressed()) {
//     //       // Zero the encoder only when the limit switch is switches from "unpressed" to "pressed" to
//     //       // prevent constant zeroing while pressed
//     //       elevatorEncoder.setPosition(0);
//     //       wasResetByLimit = true;
//     //     } else if (!m_ElevatorLeft.getReverseLimitSwitch().isPressed()) {
//     //       wasResetByLimit = false;
//     //     }
//     //   }

//     public Command c_ElevatorUpCommand () {
//       return this.startEnd(
//         () -> {
//           f_SetElevatorSpeed(ElevatorConstants.k_ElevatorSpeed);
//         }, () -> {
//           f_Stop();
//         });
//     }

//     public Command c_ElevatorDownCommand () {
//       return this.startEnd(
//         () -> {
//           f_SetElevatorSpeed(-ElevatorConstants.k_ElevatorSpeed);
//         }, () -> {
//           f_Stop();
//         });
//     }

//     private void f_SetElevatorSpeed (double speed) {
//       m_ElevatorLeft.set(speed);
//       m_ElevatorRight.set(speed);
//     }

//     private void f_Stop() {
//       m_ElevatorLeft.set(0);
//       m_ElevatorRight.set(0);
//     }
    
//     private void f_zeroOnUserButton() {
//     if (!wasResetByButton && RobotController.getUserButton()) {
//       // Zero the encoders only when button switches from "unpressed" to "pressed" to prevent
//       // constant zeroing while pressed
//       wasResetByButton = true;
//       elevatorEncoder.setPosition(0);
//     } else if (!RobotController.getUserButton()) {
//       wasResetByButton = false;
//     }
//     }

//     @Override
//     public void periodic() {
//       f_moveToSetpoint();
//       // f_zeroElevatorOnLimitSwitch();
//       f_zeroOnUserButton();

//       SmartDashboard.putNumber("Elevator Target Position", elevatorCurrentTarget);
//       SmartDashboard.putNumber("Elevator Actual Position", elevatorEncoder.getPosition());
//   }
// }
