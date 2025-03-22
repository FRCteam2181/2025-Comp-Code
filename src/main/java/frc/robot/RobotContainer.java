// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
// import edu.wpi.first.wpilibj.GenericHID.RumbleType;
// import edu.wpi.first.wpilibj.XboxController;


// Subsystem Imports 
// import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.ElevatorSubsystemPID;
import frc.robot.subsystems.CoralFunnel;
import frc.robot.subsystems.CoralPlacer;
import frc.robot.subsystems.AlgaeClaw;
import frc.robot.subsystems.AlgaeRotator;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.systems.TargetingSystem;

import frc.robot.systems.TargetingSystem.ReefBranchLevel;
import frc.robot.systems.field.AllianceFlipUtil;
import frc.robot.systems.field.FieldConstants.CoralStation;
import frc.robot.systems.ScoringSystem;


import java.io.File;
import swervelib.SwerveInputStream;

import frc.robot.Constants.OperatorConstants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Controllers and Button Board
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final Joystick positioningBoard = new Joystick(1);
  final Joystick elevatorBoard = new Joystick(2);
  
  final CommandXboxController opperatorXbox = new CommandXboxController(3);
  // final CommandXboxController opperatorXbox2 = new CommandXboxController(4);

  // The robot's subsystems are defined here
  private final SwerveSubsystem drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/maxSwerve"));
  private final ElevatorSubsystemPID s_Elevator = new ElevatorSubsystemPID();
  private final CoralFunnel s_CoralFunnel = new CoralFunnel();
  private final CoralPlacer s_CoralPlacer = new CoralPlacer();
  private final AlgaeClaw s_AlgaeClaw = new AlgaeClaw();
  private final AlgaeRotator s_AlgaeRotator = new AlgaeRotator();
  private final Climber s_climber = new Climber();
  //private final Blinkin s_Blinkin = new Blinkin();


  // The robot's systems are defined here
  private final TargetingSystem targetingSystem = new TargetingSystem();
  private final ScoringSystem   scoringSystem   = new ScoringSystem(s_CoralPlacer,
                                                                    s_Elevator,
                                                                    drivebase,
                                                                    s_AlgaeClaw,
                                                                    s_AlgaeRotator,
                                                                    targetingSystem,
                                                                    s_CoralFunnel,
                                                                    s_climber);


  // Establishing the Auto Chooser that will appear on the SmartDashboard
  private final SendableChooser<Command> autoChooser;


  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(() -> driverXbox.getRightX() * -1)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(driverXbox::getRightX,
                                                                                             driverXbox::getRightY)
                                                           .headingWhile(true);

  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverXbox.getLeftY(),
                                                                        () -> -driverXbox.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
                                                                               .withControllerHeadingAxis(() ->
                                                                                                              Math.sin(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2),
                                                                                                          () ->
                                                                                                              Math.cos(
                                                                                                                  driverXbox.getRawAxis(
                                                                                                                      2) *
                                                                                                                  Math.PI) *
                                                                                                              (Math.PI *
                                                                                                               2))
                                                                               .headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    //CanBridge.runTCP();
    //Silence the Joystick not Connected warnings (except for when connected to the FMS)
    DriverStation.silenceJoystickConnectionWarning(true);
    

    //Register NamedCommands to add to PathPlannerAdd all actions to PathPlanner
    NamedCommands.registerCommand("Score L4", s_Elevator.setElevatorHeight(Constants.ElevatorConstants.k_L4).withTimeout(2.5)
                                                   .andThen(new ParallelCommandGroup(s_CoralPlacer.c_getCoralPlacerGenCommand(),
                                                   new WaitCommand(.6)
                                                   .andThen(s_Elevator.setGoal(Units.inchesToMeters(73.5))))).withTimeout(3.5)
                                                   .andThen(s_Elevator.setElevatorHeight(Constants.ElevatorConstants.k_FeederStation)).withTimeout(5.5));

//TODO Part 3. Testing Command
// NamedCommands.registerCommand("Score L4", s_Elevator.setElevatorHeight(Constants.ElevatorConstants.k_L4)
//                                                    .until(s_Elevator.aroundCoralL4())
//                                                    .andThen(new ParallelDeadlineGroup(new WaitCommand(.6)
//                                                                                      .andThen(s_Elevator.setElevatorHeight(Constants.ElevatorConstants.k_L4BumpUP))
//                                                                                      .until(s_Elevator.aroundL4BumpUp()).andThen(new WaitCommand(.1)),
//                                                    s_CoralPlacer.c_getCoralPlacerGenCommand()))
//                                                    .andThen(s_Elevator.setElevatorHeight(Constants.ElevatorConstants.k_FeederStation)
//                                                    .until(s_Elevator.aroundFeederStation())));

                                                   
    NamedCommands.registerCommand("Score L3", new ParallelCommandGroup(s_Elevator.setElevatorHeight(Constants.ElevatorConstants.k_L3),
                                                   new WaitCommand(1.5).andThen(s_CoralPlacer.c_getCoralPlacerGenCommand()).withTimeout(2)).withTimeout(2.6)
                                                   .andThen(s_Elevator.setElevatoorZero()));

//TODO Part 3. Testing Command
// NamedCommands.registerCommand("Score L3", s_Elevator.setElevatorHeight(Constants.ElevatorConstants.k_L3)
//                                               .until(s_Elevator.aroundCoralL3())
//                                               .andThen(new ParallelDeadlineGroup(s_CoralFunnel.unloadCoral(),
//                                               s_CoralPlacer.c_getCoralPlacerGenCommand()))
//                                               .andThen(s_Elevator.setElevatorHeight(Constants.ElevatorConstants.k_FeederStation)
//                                               .until(s_Elevator.aroundFeederStation())));


    NamedCommands.registerCommand("Score L2", new ParallelCommandGroup(s_Elevator.setElevatorHeight(Constants.ElevatorConstants.k_L2),
                                                   new WaitCommand(.75).andThen(s_CoralPlacer.c_getCoralPlacerGenCommand())).withTimeout(2)
                                                   .andThen(s_Elevator.setElevatoorZero()));

//TODO Part 3. Testing Command
// NamedCommands.registerCommand("Score L2", s_Elevator.setElevatorHeight(Constants.ElevatorConstants.k_L2)
//                      .until(s_Elevator.aroundCoralL2())
//                      .andThen(new ParallelDeadlineGroup(s_CoralFunnel.unloadCoral(),
//                      s_CoralPlacer.c_getCoralPlacerGenCommand()))
//                      .andThen(s_Elevator.setElevatorHeight(Constants.ElevatorConstants.k_FeederStation)
//                      .until(s_Elevator.aroundFeederStation())));



    NamedCommands.registerCommand("Intake A1", s_Elevator.setElevatorHeight(Constants.ElevatorConstants.k_A1).withTimeout(1.5)
                                                    .andThen(new ParallelCommandGroup(s_AlgaeRotator.c_GetAlgeaRotateUpCommand(),
                                                    s_AlgaeClaw.c_getAlgaeIntakeCommand()).withTimeout(1)
                                                    .andThen(new ParallelCommandGroup(s_AlgaeRotator.c_GetAlgeaRotateDownCommand(),
                                                     new WaitCommand(.65).andThen(s_Elevator.setElevatoorZero())).withTimeout(2.5))));

    NamedCommands.registerCommand("Intake A2", s_Elevator.setElevatorHeight(Constants.ElevatorConstants.k_A2).withTimeout(1.5)
                                                    .andThen(new ParallelCommandGroup(s_AlgaeRotator.c_GetAlgeaRotateUpCommand(),
                                                    s_AlgaeClaw.c_getAlgaeIntakeCommand()).withTimeout(1)
                                                    .andThen(new ParallelCommandGroup(s_AlgaeRotator.c_GetAlgeaRotateDownCommand(),
                                                    new WaitCommand(.65).andThen(s_Elevator.setElevatoorZero())).withTimeout(3))));
  
    
    NamedCommands.registerCommand("Score Processor", s_AlgaeClaw.c_getAlgaeProcessorCommand().withTimeout(1.5));

    NamedCommands.registerCommand("Score Barge", s_Elevator.setElevatorHeight(Constants.ElevatorConstants.k_Net).withTimeout(2)
                                                      .andThen(new WaitCommand(0.5).andThen(s_AlgaeClaw.c_getAlgaeBargeCommand())).withTimeout(3)
                                                      .andThen(s_Elevator.setElevatoorZero()).withTimeout(5.5));

    NamedCommands.registerCommand("Intake from HP Station", s_CoralFunnel.loadCoral());

    
    
    NamedCommands.registerCommand("Drive to Pose Nearest Branch", targetingSystem.autoTargetCommand(drivebase::getPose)
                                                                       .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L1)).andThen(scoringSystem.scoreCoral()));
  
    NamedCommands.registerCommand("Drive to Pose LeftHP", scoringSystem.collectLeftHP());

    NamedCommands.registerCommand("Drive to Pose RightHP", scoringSystem.collectRightHP());

    NamedCommands.registerCommand("Drive to Pose ProcessorHP", scoringSystem.scoreProcessor());

    NamedCommands.registerCommand("Drive to Branch A", targetingSystem.manualTargetCommand(drivebase::getPose, 0)
                                                            .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L1))
                                                            .andThen(scoringSystem.scoreCoral()));

    NamedCommands.registerCommand("Drive to Branch B", targetingSystem.manualTargetCommand(drivebase::getPose, 1)
                                                            .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L1))
                                                            .andThen(scoringSystem.scoreCoral()));

    NamedCommands.registerCommand("Drive to Branch C", targetingSystem.manualTargetCommand(drivebase::getPose, 3)
                                                            .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L1))
                                                            .andThen(scoringSystem.scoreCoral()));

    NamedCommands.registerCommand("Drive to Branch D", targetingSystem.manualTargetCommand(drivebase::getPose, 2)
                                                            .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L1))
                                                            .andThen(scoringSystem.scoreCoral()));
              
    NamedCommands.registerCommand("Drive to Branch E", targetingSystem.manualTargetCommand(drivebase::getPose, 5)
                                                            .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L1))
                                                            .andThen(scoringSystem.scoreCoral()));

    NamedCommands.registerCommand("Drive to Branch F", targetingSystem.manualTargetCommand(drivebase::getPose, 4)
                                                            .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L1))
                                                            .andThen(scoringSystem.scoreCoral()));
    
    NamedCommands.registerCommand("Drive to Branch G", targetingSystem.manualTargetCommand(drivebase::getPose, 7)
                                                            .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L1))
                                                            .andThen(scoringSystem.scoreCoral()));
     
    NamedCommands.registerCommand("Drive to Branch H", targetingSystem.manualTargetCommand(drivebase::getPose, 6)
                                                            .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L1))
                                                            .andThen(scoringSystem.scoreCoral()));
       
    NamedCommands.registerCommand("Drive to Branch I", targetingSystem.manualTargetCommand(drivebase::getPose, 9)
                                                            .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L1))
                                                            .andThen(scoringSystem.scoreCoral()));
    
    NamedCommands.registerCommand("Drive to Branch J", targetingSystem.manualTargetCommand(drivebase::getPose, 8)
                                                            .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L1))
                                                            .andThen(scoringSystem.scoreCoral()));
              
    NamedCommands.registerCommand("Drive to Branch K", targetingSystem.manualTargetCommand(drivebase::getPose, 11)
                                                            .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L1))
                                                            .andThen(scoringSystem.scoreCoral()));
    
    NamedCommands.registerCommand("Drive to Branch L", targetingSystem.manualTargetCommand(drivebase::getPose, 10)
                                                            .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L1))
                                                            .andThen(scoringSystem.scoreCoral()));



//Attempt at compensating for Red/Blue flip

    // NamedCommands.registerCommand("Drive to Branch A", targetingSystem.inAutoTargetCommand(drivebase::getPose, 1, 0)
    //                                                         .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L1))
    //                                                         .andThen(scoringSystem.scoreCoral()));

    // NamedCommands.registerCommand("Drive to Branch B", targetingSystem.inAutoTargetCommand(drivebase::getPose, 0, 1)
    //                                                         .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L1))
    //                                                         .andThen(scoringSystem.scoreCoral()));

    // NamedCommands.registerCommand("Drive to Branch C", targetingSystem.inAutoTargetCommand(drivebase::getPose, 11, 3)
    //                                                         .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L1))
    //                                                         .andThen(scoringSystem.scoreCoral()));

    // NamedCommands.registerCommand("Drive to Branch D", targetingSystem.inAutoTargetCommand(drivebase::getPose, 10, 2)
    //                                                         .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L1))
    //                                                         .andThen(scoringSystem.scoreCoral()));
              
    // NamedCommands.registerCommand("Drive to Branch E", targetingSystem.inAutoTargetCommand(drivebase::getPose, 9, 5)
    //                                                         .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L1))
    //                                                         .andThen(scoringSystem.scoreCoral()));

    // NamedCommands.registerCommand("Drive to Branch F", targetingSystem.inAutoTargetCommand(drivebase::getPose, 8, 4)
    //                                                         .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L1))
    //                                                         .andThen(scoringSystem.scoreCoral()));
    
    // NamedCommands.registerCommand("Drive to Branch G", targetingSystem.inAutoTargetCommand(drivebase::getPose, 7, 7)
    //                                                         .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L1))
    //                                                         .andThen(scoringSystem.scoreCoral()));
     
    // NamedCommands.registerCommand("Drive to Branch H", targetingSystem.inAutoTargetCommand(drivebase::getPose, 6, 6)
    //                                                         .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L1))
    //                                                         .andThen(scoringSystem.scoreCoral()));
       
    // NamedCommands.registerCommand("Drive to Branch I", targetingSystem.inAutoTargetCommand(drivebase::getPose, 5, 9)
    //                                                         .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L1))
    //                                                         .andThen(scoringSystem.scoreCoral()));
    
    // NamedCommands.registerCommand("Drive to Branch J", targetingSystem.inAutoTargetCommand(drivebase::getPose, 4, 8)
    //                                                         .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L1))
    //                                                         .andThen(scoringSystem.scoreCoral()));
              
    // NamedCommands.registerCommand("Drive to Branch K", targetingSystem.inAutoTargetCommand(drivebase::getPose, 3, 11)
    //                                                         .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L1))
    //                                                         .andThen(scoringSystem.scoreCoral()));
    
    // NamedCommands.registerCommand("Drive to Branch L", targetingSystem.inAutoTargetCommand(drivebase::getPose, 2, 10)
    //                                                         .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L1))
    //                                                         .andThen(scoringSystem.scoreCoral()));                                                        






    //Set the default auto and put the autoChoser on the SmartDashboard
    autoChooser = AutoBuilder.buildAutoChooser("Center 1 Piece L4");
    SmartDashboard.putData("Auto Chooser", autoChooser); 
 
    //This should allow us to see what the active commands are
    SmartDashboard.putData(CommandScheduler.getInstance()); //TODO see if this works

    // Configure the trigger bindings
    configureBindings();

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    //All of this is different driving options that are default supported by YAGSL
    //Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    //Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
    //Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    //Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    //Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(driveDirectAngleKeyboard);

    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation())
    {
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());

    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
    } else
    {
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.y().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());

      //Driver Controller Algae Claw up and down for climbing
      driverXbox.b().whileTrue(s_AlgaeRotator.c_GetAlgeaRotateUpCommand());
      driverXbox.x().whileTrue(s_AlgaeRotator.c_GetAlgeaRotateDownCommand());
      
      //Climber
      driverXbox.leftTrigger().whileTrue(s_climber.c_GetClimberUpCommand());
      driverXbox.rightTrigger().whileTrue(s_climber.c_GetClimberDownCommand());

      //FunnelRotator
      driverXbox.rightBumper().whileTrue(s_CoralFunnel.c_FunnelRotateCommandUp());
      driverXbox.leftBumper().whileTrue(s_CoralFunnel.c_FunnelRotateCommandDown());

    }
    

    //This is the working go to nearest reef branch command
// JoystickButton abPositionButton = new JoystickButton(positioningBoard, 4);
//     abPositionButton.whileTrue(targetingSystem.autoTargetCommand(drivebase::getPose)
//                                                          .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L1)).andThen(scoringSystem.scoreCoral())
//                                                          );

    //Position Board Commands    

    // Reef A
    JoystickButton aPositionButton = new JoystickButton(positioningBoard, 1);
    aPositionButton.whileTrue(targetingSystem.manualTargetCommand(drivebase::getPose, 0)
    .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L1)).andThen(scoringSystem.scoreCoral())
    );

    // Reef B         
    JoystickButton bPositionButton = new JoystickButton(positioningBoard, 2);
    bPositionButton.whileTrue(targetingSystem.manualTargetCommand(drivebase::getPose, 1)
    .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L1)).andThen(scoringSystem.scoreCoral())
    );
   
    // Reef C
    JoystickButton cPositionButton = new JoystickButton(positioningBoard, 12);
    cPositionButton.whileTrue(targetingSystem.manualTargetCommand(drivebase::getPose, 3)
    .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L1)).andThen(scoringSystem.scoreCoral())
    );

    // Reef D
    JoystickButton dPositionButton = new JoystickButton(positioningBoard, 11);
    dPositionButton.whileTrue(targetingSystem.manualTargetCommand(drivebase::getPose, 2)
    .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L1)).andThen(scoringSystem.scoreCoral())
    );
          
    // Reef E
    JoystickButton ePositionButton = new JoystickButton(positioningBoard, 10);
    ePositionButton.whileTrue(targetingSystem.manualTargetCommand(drivebase::getPose, 5)
    .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L1)).andThen(scoringSystem.scoreCoral())
    );

    // Reef F
    JoystickButton fPositionButton = new JoystickButton(positioningBoard, 9);
    fPositionButton.whileTrue(targetingSystem.manualTargetCommand(drivebase::getPose, 4)
    .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L1)).andThen(scoringSystem.scoreCoral())
    );

    // Reef G
    JoystickButton gPositionButton = new JoystickButton(positioningBoard, 8);
    gPositionButton.whileTrue(targetingSystem.manualTargetCommand(drivebase::getPose, 7)
    .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L1)).andThen(scoringSystem.scoreCoral())
    );

    // Reef H         
    JoystickButton hPositionButton = new JoystickButton(positioningBoard, 7);
    hPositionButton.whileTrue(targetingSystem.manualTargetCommand(drivebase::getPose, 6)
    .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L1)).andThen(scoringSystem.scoreCoral())
    );
   
    // Reef I
    JoystickButton iPositionButton = new JoystickButton(positioningBoard, 6);
    iPositionButton.whileTrue(targetingSystem.manualTargetCommand(drivebase::getPose, 9)
    .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L1)).andThen(scoringSystem.scoreCoral())
    );

    // Reef J
    JoystickButton jPositionButton = new JoystickButton(positioningBoard, 5);
    jPositionButton.whileTrue(targetingSystem.manualTargetCommand(drivebase::getPose, 8)
    .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L1)).andThen(scoringSystem.scoreCoral())
    );
          
    // Reef K
    JoystickButton kPositionButton = new JoystickButton(positioningBoard, 4);
    kPositionButton.whileTrue(targetingSystem.manualTargetCommand(drivebase::getPose, 11)
    .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L1)).andThen(scoringSystem.scoreCoral())
    );

    // Reef L
    JoystickButton lPositionButton = new JoystickButton(positioningBoard, 3);
    lPositionButton.whileTrue(targetingSystem.manualTargetCommand(drivebase::getPose, 10)
    .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L1)).andThen(scoringSystem.scoreCoral())
    );
    
    //Closest Branch
    Trigger closeBranchPositionButton = new Trigger(() -> positioningBoard.getX() < -0.5);
    closeBranchPositionButton.whileTrue(targetingSystem.autoTargetCommand(drivebase::getPose)
    .andThen(targetingSystem.setBranchLevel(ReefBranchLevel.L1)).andThen(scoringSystem.scoreCoral()));


    //Elevator Board Commands 

    //Reverse coral funnel intake wheels
    JoystickButton coralReverseIntakeButton = new JoystickButton(elevatorBoard, 11);
    coralReverseIntakeButton.whileTrue(s_CoralFunnel.c_getFunnelWheelCommandback());

    //Run coral funnel intake wheels
    JoystickButton coralIntakeButton = new JoystickButton(elevatorBoard, 12);
    coralIntakeButton.whileTrue(s_CoralFunnel.c_getFunnelWheelCommand());
                    //alongWith(s_CoralPlacer.c_getCoralPlacerReverseCommand())


    //Algae Auto Ground Intake Command 
    JoystickButton algaeGroundButton = new JoystickButton(elevatorBoard, 10);
    algaeGroundButton.onTrue(s_Elevator.setElevatorHeight(Constants.ElevatorConstants.k_Processor).withTimeout(1)
                            .andThen(new ParallelCommandGroup(s_AlgaeRotator.c_GetAlgeaRotateUpCommand(),
                            s_AlgaeClaw.c_getAlgaeIntakeCommand()).withTimeout(1.5)
                            .andThen(new ParallelCommandGroup(s_AlgaeRotator.c_GetAlgeaRotateDownCommand(),
                            new WaitCommand(.25).andThen(s_Elevator.setElevatoorZero())).withTimeout(1.5))));

    //Algae Auto Dunk Command
    JoystickButton algaeNetButton = new JoystickButton(elevatorBoard, 9);
    algaeNetButton.onTrue(s_Elevator.setElevatorHeight(Constants.ElevatorConstants.k_Net).withTimeout(2)
                          .andThen(new WaitCommand(0.5).andThen(s_AlgaeClaw.c_getAlgaeBargeCommand())).withTimeout(3)
                          .andThen(s_Elevator.setElevatoorZero()).withTimeout(5.5));

    //Algae retrival from A2                                          
    JoystickButton a2Button = new JoystickButton(elevatorBoard, 8);
    a2Button.onTrue(s_Elevator.setElevatorHeight(Constants.ElevatorConstants.k_A2).withTimeout(1.5)
                    .andThen(new ParallelCommandGroup(s_AlgaeRotator.c_GetAlgeaRotateUpCommand(),
                    s_AlgaeClaw.c_getAlgaeIntakeCommand()).withTimeout(1)
                    .andThen(new ParallelCommandGroup(s_AlgaeRotator.c_GetAlgeaRotateDownCommand(),
                    new WaitCommand(.65).andThen(s_Elevator.setElevatoorZero())).withTimeout(3))));

    //Algae retrival from A1
    JoystickButton a1Button = new JoystickButton(elevatorBoard, 7);
    a1Button.onTrue(s_Elevator.setElevatorHeight(Constants.ElevatorConstants.k_A1).withTimeout(1.5)
                   .andThen(new ParallelCommandGroup(s_AlgaeRotator.c_GetAlgeaRotateUpCommand(),
                    s_AlgaeClaw.c_getAlgaeIntakeCommand()).withTimeout(1)
                    .andThen(new ParallelCommandGroup(s_AlgaeRotator.c_GetAlgeaRotateDownCommand(),
                    new WaitCommand(.65).andThen(s_Elevator.setElevatoorZero())).withTimeout(2.5))));

    //Algae Auto Processor Command 
    JoystickButton processorButton = new JoystickButton(elevatorBoard, 6);
    processorButton.onTrue(s_AlgaeClaw.c_getAlgaeProcessorCommand().withTimeout(1.5));

        
    //Set Elevator to intake height for coral funnel
    JoystickButton coralIntakeHeighButton = new JoystickButton(elevatorBoard, 5);
    coralIntakeHeighButton.onTrue(s_Elevator.setElevatorHeight(Constants.ElevatorConstants.k_FeederStation));

    //TODO Part 2. Testing Command
    // coralIntakeHeighButton.onTrue(s_Elevator.setElevatorHeight(Constants.ElevatorConstants.k_FeederStation)
    //                               .until(s_Elevator.aroundFeederStation()));

    //L4 Auto Score
    JoystickButton L4Button = new JoystickButton(elevatorBoard, 4);//set correct number
    L4Button.onTrue(s_Elevator.setElevatorHeight(Constants.ElevatorConstants.k_L4).withTimeout(3)
                    .andThen(new ParallelCommandGroup(s_CoralPlacer.c_getCoralPlacerGenCommand(),
                    new WaitCommand(.6).andThen(s_Elevator.setGoal(Units.inchesToMeters(73.5))))).withTimeout(4.5)
                    .andThen(s_Elevator.setElevatoorZero()));

    //TODO Part 2. Testing Command
    // L4Button.onTrue(s_Elevator.setElevatorHeight(Constants.ElevatorConstants.k_L4)
    //                 .until(s_Elevator.aroundCoralL4())
    //                 .andThen(new ParallelDeadlineGroup(new WaitCommand(.6)
    //                                                   .andThen(s_Elevator.setElevatorHeight(Constants.ElevatorConstants.k_L4BumpUP))
    //                                                   .until(s_Elevator.aroundL4BumpUp()).andThen(new WaitCommand(.1)),
    //                 s_CoralPlacer.c_getCoralPlacerGenCommand()))
    //                 .andThen(s_Elevator.setElevatorHeight(Constants.ElevatorConstants.k_FeederStation)
    //                 .until(s_Elevator.aroundFeederStation())));


    //L3 Auto Score
    JoystickButton L3Button = new JoystickButton(elevatorBoard, 3);//set correct number
    L3Button.onTrue(new ParallelCommandGroup(s_Elevator.setElevatorHeight(Constants.ElevatorConstants.k_L3),
                    new WaitCommand(1.5).andThen(s_CoralPlacer.c_getCoralPlacerGenCommand()).withTimeout(2)).withTimeout(2.6)
                    .andThen(s_Elevator.setElevatoorZero()));

    //TODO Part 2. Testing Command
    // L3Button.onTrue(s_Elevator.setElevatorHeight(Constants.ElevatorConstants.k_L3)
    //                 .until(s_Elevator.aroundCoralL3())
    //                 .andThen(new ParallelDeadlineGroup(s_CoralFunnel.unloadCoral(),
    //                 s_CoralPlacer.c_getCoralPlacerGenCommand()))
    //                 .andThen(s_Elevator.setElevatorHeight(Constants.ElevatorConstants.k_FeederStation)
    //                 .until(s_Elevator.aroundFeederStation())));


    //L2 Auto Score
    JoystickButton L2Button = new JoystickButton(elevatorBoard, 2);//set correct number
    L2Button.onTrue(new ParallelCommandGroup(s_Elevator.setElevatorHeight(Constants.ElevatorConstants.k_L2),
                    new WaitCommand(.75).andThen(s_CoralPlacer.c_getCoralPlacerGenCommand())).withTimeout(2)
                    .andThen(s_Elevator.setElevatoorZero()));

    //TODO Part 2. Testing Command
    // L2Button.onTrue(s_Elevator.setElevatorHeight(Constants.ElevatorConstants.k_L2)
    //                 .until(s_Elevator.aroundCoralL2())
    //                 .andThen(new ParallelDeadlineGroup(s_CoralFunnel.unloadCoral(),
    //                 s_CoralPlacer.c_getCoralPlacerGenCommand()))
    //                 .andThen(s_Elevator.setElevatorHeight(Constants.ElevatorConstants.k_FeederStation)
    //                 .until(s_Elevator.aroundFeederStation())));


    //Manually return Elevator to 0            
    JoystickButton ZeroButton = new JoystickButton(elevatorBoard, 1);
    ZeroButton.onTrue(s_Elevator.setElevatorHeight(0));

    //TODO Part 2. Testing Command
    //ZeroButton.onTrue(s_Elevator.setElevatorHeight(0).until(s_Elevator.aroundElevatorZero()));
   
    //Shoot Out Coral
    Trigger ShootCoralButton = new Trigger(() -> elevatorBoard.getX() < -0.5);
    ShootCoralButton.whileTrue(s_CoralPlacer.c_getCoralPlacerGenCommand());

    //Disable auto zero
    Trigger AZStop = new Trigger(() -> positioningBoard.getY() > 0.5);
    AZStop.onTrue(s_Elevator.autoZeroSwitchCommand());
                                                      

    //Operator 1 and 2 Xbox Controller Testing Buttons 

    //TODO Item 1. Try this 
    opperatorXbox.a().onTrue(s_Elevator.setElevatorHeight(Constants.ElevatorConstants.k_FeederStation)
                             .until(s_Elevator.aroundFeederStation()).andThen(s_CoralFunnel.loadCoral())
                                            );
    // opperatorXbox.b().onTrue(s_Elevator.setElevatorHeight(Constants.ElevatorConstants.k_L2)
    //                          .andThen(new ParallelDeadlineGroup(s_CoralFunnel.unloadCoral(),
    //                          s_CoralPlacer.c_getCoralPlacerGenCommand()))
    //                          .andThen(s_Elevator.setElevatoorZero())); //L2
    // opperatorXbox.x().onTrue(s_CoralFunnel.loadCoral());//L3
    // opperatorXbox.y().onTrue(new ParallelDeadlineGroup(s_CoralFunnel.unloadCoral(),
    //                           s_CoralPlacer.c_getCoralPlacerGenCommand()));//L4

    // //Funnel intake height
    // opperatorXbox.leftBumper().onTrue(s_Elevator.setGoal(Units.inchesToMeters(17.375)));

    // //CoralFunnel
    // opperatorXbox.rightBumper().whileTrue(s_CoralFunnel.c_getFunnelWheelCommand());
  
    // //CoralPlacer 
    // opperatorXbox.rightTrigger().whileTrue(s_CoralPlacer.c_getCoralPlacerGenCommand());
    // opperatorXbox.leftTrigger().onTrue(new ParallelCommandGroup(s_CoralPlacer.c_getCoralPlacerGenCommand().withTimeout(1.5),
    //                                                            new WaitCommand(.6).andThen(s_Elevator.setGoal(Units.inchesToMeters(73.5)))));
    
    // //Algae claw height
    // opperatorXbox2.a().onTrue(s_Elevator.setGoal(Units.inchesToMeters(15.5))); //a1
    // opperatorXbox2.b().onTrue(s_Elevator.setGoal(Units.inchesToMeters(31))); //a2
    // opperatorXbox2.y().onTrue(s_Elevator.setGoal(Units.inchesToMeters(73.875))); //barge
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    return autoChooser.getSelected();
  }


  public Command driveToSetPoint(double x, double y, double angle)
  {
    return drivebase.driveToPose(
        new Pose2d(new Translation2d
                       (Meter.of(x),
                        Meter.of(y)),
                   Rotation2d.fromDegrees(angle)));
  }



public Command driveToHumanPlayer1()
  {
    if (AllianceFlipUtil.shouldFlip())
    {
      return drivebase.driveToPose(AllianceFlipUtil.flip(CoralStation.leftCenterFace));
    } else
    {
      return drivebase.driveToPose((CoralStation.leftCenterFace));
    }
  }

  public Command driveToHumanPlayer2()
  {
    if (AllianceFlipUtil.shouldFlip())
    {
      return drivebase.driveToPose(AllianceFlipUtil.flip(CoralStation.rightCenterFace));
    } else
    {
      return drivebase.driveToPose((CoralStation.rightCenterFace));
    }
  }

  public Command driveToProcessor()
  {
    return drivebase.driveToPose(
        new Pose2d(new Translation2d
                       (Meter.of(11.5),
                        Meter.of(7.5)),
                   Rotation2d.fromDegrees(90)));
  }


  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}