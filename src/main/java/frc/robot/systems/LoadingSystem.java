package frc.robot.systems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;

import frc.robot.Setpoints;

import frc.robot.subsystems.AlgaeClaw;

import frc.robot.subsystems.AlgaeRotator;
import frc.robot.subsystems.Climber;

import frc.robot.subsystems.CoralFunnel;

import frc.robot.subsystems.CoralPlacer;

import frc.robot.subsystems.ElevatorSubsystemPID;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

;

public class LoadingSystem
{

  private CoralPlacer          m_coralPlacer;
  private AlgaeClaw            m_algaeClaw;
  private ElevatorSubsystemPID    m_elevator;
  private SwerveSubsystem      m_drivebase;
  private AlgaeRotator         m_algaeRotator;
  private TargetingSystem      m_targetSystem;
  private CoralFunnel          m_coralFunnel;
  private Climber              m_climber;




  public LoadingSystem(CoralPlacer coralPlacer,
                       AlgaeClaw algaeClaw,
                       ElevatorSubsystemPID elevator,
                       SwerveSubsystem drivebase,
                       AlgaeRotator algaeRotator,
                       TargetingSystem targetSys,
                       CoralFunnel coralFunnel,
                       Climber climber )
  {
    
  m_coralPlacer = coralPlacer;
  m_algaeClaw = algaeClaw;
  m_elevator = elevator;
  m_drivebase = drivebase;
  m_algaeRotator = algaeRotator;
  m_targetSystem = targetSys;
  m_coralFunnel = coralFunnel;
  m_climber = climber;
  }

  //For testing, set the sensor to low voltage first
  //The elevator needs to rise first for the arm to come out


//   public Command coralLoad()
//   {

//     return (Commands.parallel(m_elevator.CoralHP().repeatedly(), //Drive to HP and Move ELEVATOR AND ARM
//                               m_coralArm.setCoralArmAngle(Setpoints.Arm.Coral.HP),
//                               m_swerve.lockPos()))
//         .until(m_elevator.aroundCoralHP()
//                          .and(m_coralArm.aroundCoralHPAngle()))
//         .withTimeout(5) //Move Intake angle to 0
//         .andThen(m_coralIntake.wristRest().until(m_coralIntake.atRestingAngle()))
//         .andThen(Commands.parallel(m_coralIntake.wristIntake(),m_swerve.lockPos())
//                          .withDeadline(m_coralArm.load()) //end command
//                          .withTimeout(1)
//                          .until(() -> m_coralArm.coralLoaded()));

//   }

//   public Command coralLoadAuto()
//   {

//     return (Commands.parallel(m_elevator.CoralHP().repeatedly(),
//                               m_coralArm.setCoralArmAngle(Setpoints.Arm.Coral.HP).repeatedly()))
//         .until(m_elevator.aroundCoralHP()
//                          .and(m_coralArm.aroundCoralHPAngle()))
//         .withTimeout(5) //Move Intake angle to 0
//         .andThen(m_coralIntake.wristRest().until(m_coralIntake.atRestingAngle()))
//         .andThen(m_coralIntake.wristIntake())
//         .withTimeout(1)
//         .until(() -> m_coralArm.coralLoaded());

//   }


//   public Command algaeLoadAuto()
//   {

//     return Commands.parallel(m_elevator.getAlgaeCommand(m_targetSystem).repeatedly(),
//                              m_algaeArm.getAlgaeCommand(m_targetSystem).repeatedly())
//                    .until(m_elevator.atAlgaeHeight(m_targetSystem)
//                                     .and(m_algaeArm.atAlgaeAngle(m_targetSystem)))
//                    .withTimeout(5)
//                    .andThen(Commands.parallel(m_algaeIntake.setAlgaeIntakeRoller(IntakeConstants.AlgaeOuttakeSpeeds),
//                                               m_elevator.getAlgaeCommand(m_targetSystem).repeatedly())
//                                     .withDeadline(m_algaeArm.load())
//                                     .withTimeout(1)
//                                     .until(() -> m_algaeArm.algaeLoaded()))
//                    .andThen((m_elevator.getAlgaeCommand(m_targetSystem).repeatedly())
//                                 .withTimeout(1));


//   }

//   public Command algaeLoad()
//   {

//     return m_targetSystem.driveToCoralTarget(m_swerve)
//                          .andThen(Commands.parallel(m_elevator.getAlgaeCommand(m_targetSystem).repeatedly(),
//                                                     m_algaeArm.getAlgaeCommand(m_targetSystem).repeatedly(),                     
//                              m_swerve.lockPos())
//                                           .until(m_elevator.atAlgaeHeight(m_targetSystem)
//                                                            .and(m_algaeArm.atAlgaeAngle(m_targetSystem)))
//                                           .withTimeout(5))
//                          .andThen(Commands.parallel(m_algaeIntake.setAlgaeIntakeRoller(IntakeConstants.AlgaeOuttakeSpeeds),
//                                                     m_elevator.getAlgaeCommand(m_targetSystem).repeatedly(),
//                                                     m_swerve.lockPos())
//                                           .withDeadline(m_algaeArm.load())
//                                           .withTimeout(1)
//                                           .until(() -> m_algaeArm.algaeLoaded()))
//                          .andThen(m_swerve.driveForwards()
//                                           .alongWith(m_elevator.getAlgaeCommand(m_targetSystem).repeatedly())
//                                           .withTimeout(1));


//   }


//   public Command coralLock()
//   {
//     // Set arm to target angle, elev target height
//     return m_coralArm.getCoralCommand(m_targetSystem).repeatedly()
//                      .alongWith(m_elevator.getCoralCommand(m_targetSystem).repeatedly(), m_wrist.wristScore());
//   }

//   public Command algaeLockProcessor()
//   {
//     // Set arm to target angle, elev target height
//     double algaeArmLockingProcessorAngleDegrees      = -45;
//     double algaeElevatorLockingProcessorHeightMeters = 1.0;

//     return m_elevator.setElevatorHeight(algaeElevatorLockingProcessorHeightMeters)
//                      .andThen(m_algaeArm.setAlgaeArmAngle(algaeArmLockingProcessorAngleDegrees).repeatedly());
//   }


//   public Command algaeLockNet()
//   {
//     // Set arm to target angle, elev target height
//     double algaeArmLockingNetAngleDegrees      = 45;
//     double algaeElevatorLockingNetHeightMeters = Constants.ElevatorConstants.kMaxElevatorHeightMeters;
//     return m_elevator.setElevatorHeight(algaeElevatorLockingNetHeightMeters)
//                      .andThen(m_algaeArm.setAlgaeArmAngle(algaeArmLockingNetAngleDegrees).repeatedly());
//   }
}
