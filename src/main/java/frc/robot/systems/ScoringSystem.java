package frc.robot.systems;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaeRotator;
import frc.robot.subsystems.AlgaeClaw;
import frc.robot.subsystems.CoralPlacer;
import frc.robot.subsystems.CoralFunnel;
//import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ElevatorSubsystemPID;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
//import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Setpoints.AutoScoring.HumanPlayer.Left;
// import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.ElevatorSubsystemPID;
import frc.robot.subsystems.CoralFunnel;
import frc.robot.subsystems.CoralPlacer;
import frc.robot.subsystems.AlgaeClaw;
import frc.robot.subsystems.AlgaeRotator;
import frc.robot.subsystems.Climber;
import frc.robot.systems.TargetingSystem;
//import frc.robot.systems.ScoringSystem;
import frc.robot.systems.TargetingSystem.ReefBranch;
import frc.robot.systems.TargetingSystem.ReefSide;
import frc.robot.systems.field.FieldConstants.CoralStation;


public class ScoringSystem
{

  private CoralPlacer          m_coralPlacer;
  private AlgaeClaw            m_algaeIntake;
  private ElevatorSubsystemPID    m_elevator;
  private SwerveSubsystem      m_drivebase;
  private AlgaeRotator         m_algaeArm;
  private TargetingSystem      m_targetSystem;
  private CoralFunnel          m_coralFunnel;
  private Climber              m_climber;

  public ScoringSystem(
      CoralPlacer coralPlacer,
      ElevatorSubsystemPID elevator,
      SwerveSubsystem drivebase,
      AlgaeClaw algaeIntake,
      AlgaeRotator algaeArm,
      TargetingSystem targeting,
      CoralFunnel coralFunnel,
      Climber climber)
  {
    m_coralPlacer = coralPlacer;
    m_elevator = elevator;
    m_drivebase = drivebase;
    m_algaeIntake = algaeIntake;
    m_algaeArm = algaeArm;
    m_targetSystem = targeting;
    m_coralFunnel = coralFunnel;
    m_climber = climber;

  }

  public Command scoreCoral()
  {


    //return m_targetSystem.driveToCoralTarget(m_drivebase);
    
    
     if (m_targetSystem.targetReefSide == ReefSide.Middle)
    {

      return m_targetSystem.driveToAlgaeTarget(m_drivebase);
    
     
    }

    else 
    {
      
      
      return m_targetSystem.driveToCoralTarget(m_drivebase);
    
  



    }





  // //  return defer(() -> {
  //     Pose2d TargetPose = m_targetSystem.getTargetReefBranchPose();

      
  //     return m_drivebase.driveToPose(TargetPose);
  //  // });



    // if (m_elevator.getHeightMeters() > Units.inchesToMeters(60)) {
        
    //    return  m_coralPlacer.c_getCoralPlacerGenCommand().;
    //  }  else {
    //   return m_coralPlacer.c_getCoralPlacerGenCommand();
    //  }
     
    // // Arm down, elevator down, drive backwards x in
    // double coralArmAngleDegrees = m_targetSystem.getTargetBranchCoralArmAngle();
    // double elevatorHeightMeters = m_targetSystem.getTargetBranchHeightMeters();

    // return new ParallelDeadlineGroup(
    //   m_elevator.setElevatorHeight(elevatorHeightMeters).withName("ScoreCoralElevatorHeight")
    //   .andThen(m_coralIntake.spitCoralOut(IntakeConstants.defaultrRollerSpeed, 90))
    // .andThen(Commands.print("Tell me why aint nothing but a mistake"))
    // .andThen(m_elevator.setElevatorHeight(
    //     elevatorHeightMeters - Constants.ElevatorConstants.kLowerToScoreHeight).withName("ScoreCoralElevatorHeightLower"))
    //  //.alongWith(m_coralArm.setCoralArmAngle(coralArmAngleDegrees)).repeatedly()
    //     .andThen(Commands.print("Tell me why aint nothing but an heart ache")),
    //   m_coralArm.setCoralArmAngle(coralArmAngleDegrees).withName("ScoreCoralArmAngle").repeatedly(),
    //                 m_drivebase.lockPos().withName("LockPose")
    //                  );
  }

  public Command scoreAlgaeProcessor()
  {
    return Commands.print("Tell me why aint nothing but an heart ache");

    // //set elevator height, set algae angle, spit out ball, drive pose
    // double algaeArmAngleDegrees = -45;
    // double elevatorHeightMeters = 1.0;
    // return m_algaeArm.setAlgaeArmAngle(algaeArmAngleDegrees).repeatedly()
    //         .alongWith(m_elevator.setElevatorHeight(elevatorHeightMeters))
    //         .until(() -> m_elevator.aroundHeight(elevatorHeightMeters))
    //         .andThen(m_algaeIntake.setAlgaeIntakeRoller(Constants.IntakeConstants.AlgaeOuttakeSpeeds)
    //         .until(() -> !m_algaeArm.algaeLoaded()));
  }

  public Command scoreAlgaeNet()
  {
    return Commands.print("Tell me why aint nothing but an heart ache");
    
    //set elevator height, set alage angle, spit out ball, drive pose
    // double algaeArmAngleDegrees = 45;
    // double elevatorHeightMeters = 4.0;
    // return m_algaeArm.setAlgaeArmAngle(algaeArmAngleDegrees).repeatedly()
    //         .alongWith(m_elevator.setElevatorHeight(elevatorHeightMeters))
    //         .until(() -> m_elevator.aroundHeight(elevatorHeightMeters))
    //         .andThen(m_algaeIntake.setAlgaeIntakeRoller(Constants.IntakeConstants.AlgaeOuttakeSpeeds))
    //         .until(() -> !m_algaeArm.algaeLoaded());
  }

}
