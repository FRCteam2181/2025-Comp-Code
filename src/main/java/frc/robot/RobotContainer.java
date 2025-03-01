// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
//import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;


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
import frc.robot.systems.ScoringSystem;
import frc.robot.systems.TargetingSystem.ReefBranch;
import frc.robot.systems.TargetingSystem.ReefSide;

import java.io.File;
import swervelib.SwerveInputStream;

import frc.robot.Constants.OperatorConstants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController opperatorXbox = new CommandXboxController(3);
  final CommandXboxController opperatorXbox2 = new CommandXboxController(4);

  private final Joystick positioningBoard = new Joystick(1);
  private final Joystick elevatorBoard = new Joystick(2);
  
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/maxSwerve"));
  private final ElevatorSubsystemPID s_Elevator = new ElevatorSubsystemPID();
  private final CoralFunnel s_CoralFunnel = new CoralFunnel();
  private final CoralPlacer s_CoralPlacer = new CoralPlacer();
  private final AlgaeClaw s_AlgaeClaw = new AlgaeClaw();
  // private final Blinkin s_Blinkin = new Blinkin();
  private final AlgaeRotator s_AlgaeRotator = new AlgaeRotator();
  private final Climber s_climber = new Climber();

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
    
    DriverStation.silenceJoystickConnectionWarning(true);
    
     // Add all actions to PathPlanner
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    NamedCommands.registerCommand("Score L4", s_Elevator.setElevatorHeight(Constants.ElevatorConstants.k_L4).withTimeout(3).andThen(new ParallelCommandGroup(
      s_CoralPlacer.c_getCoralPlacerGenCommand(),
      new WaitCommand(.6).andThen(s_Elevator.setGoal(Units.inchesToMeters(73.5)))))
      .withTimeout(4.5).andThen(s_Elevator.setElevatoorZero()));
    
    NamedCommands.registerCommand("Elevator to Intake Height", s_Elevator.setElevatorHeight(Constants.ElevatorConstants.k_FeederStation));

    NamedCommands.registerCommand("Zero Elevator", s_Elevator.setElevatorHeight(0));

    NamedCommands.registerCommand("Intake Coral", s_CoralFunnel.c_getFunnelWheelCommand());

  
    
    
    autoChooser = AutoBuilder.buildAutoChooser("Center 1 Piece L4");

    SmartDashboard.putData("Auto Chooser", autoChooser); 
 
      targetingSystem.setTarget(TargetingSystem.ReefBranch.AB);

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

    Command driveFieldOrientedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveRobotOrientedAngularVelocity = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);

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
      // driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      // driverXbox.b().whileTrue(
      //     drivebase.driveToPose(
      //         new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
      //                         );
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.y().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());

      //Algae Claw up and down for climbing
      driverXbox.b().whileTrue(s_AlgaeRotator.c_GetAlgeaRotateUpCommand());
      driverXbox.x().whileTrue(s_AlgaeRotator.c_GetAlgeaRotateDownCommand());
    }

    //AlgaeClaw
    
     


// //L3 Auto Score
// JoystickButton L3Button = new JoystickButton(elevatorBoard, 3);//set correct number
// L3Button.onTrue(new ParallelCommandGroup(s_Elevator.setElevatorHeight(Constants.ElevatorConstants.k_L3),
//                                 new WaitCommand(3).andThen(s_CoralPlacer.c_getCoralPlacerGenCommand()).withTimeout(4))
//                                         .withTimeout(5).andThen(s_Elevator.setElevatorHeight(0)));


// //L4 Auto Score
// JoystickButton L4Button = new JoystickButton(elevatorBoard, 4);//set correct number
// L4Button.onTrue(s_Elevator.setElevatorHeight(Constants.ElevatorConstants.k_L4).withTimeout(3).andThen(new ParallelCommandGroup(
//                                           s_CoralPlacer.c_getCoralPlacerGenCommand(),
//                                           new WaitCommand(.6).andThen(s_Elevator.setGoal(Units.inchesToMeters(73.5)))))
//                                           .withTimeout(5).andThen(new WaitCommand(1).andThen(s_Elevator.setElevatorHeight(0))));


    //Disable auto zero
    JoystickButton AZStop = new JoystickButton(positioningBoard,11);
    AZStop.onTrue(
      s_Elevator.autoZeroSwitchCommand()
    );



    //Algae retrival from A1
    JoystickButton a1Button = new JoystickButton(elevatorBoard, 7);
        a1Button.onTrue(s_Elevator.setElevatorHeight(Constants.ElevatorConstants.k_A1)
                                              );


    //Algae retrival from A2                                          
    JoystickButton a2Button = new JoystickButton(elevatorBoard, 8);
        a2Button.onTrue(s_Elevator.setElevatorHeight(Constants.ElevatorConstants.k_A2));
    

    //Algae Auto Dunk Command
    JoystickButton algaeNetButton = new JoystickButton(elevatorBoard, 9);
        algaeNetButton.onTrue(s_Elevator.setElevatorHeight(Constants.ElevatorConstants.k_Net));
  

  //   //Algae Auto Intake Command 
  //   JoystickButton algaeGroundButton = new JoystickButton(elevatorBoard, 10);
  //       algaeGroundButton.onTrue(
  //                                             new ParallelCommandGroup(s_AlgaeRotator.setGoal(260),//set the correct scoring angle
  //                                             s_AlgaeClaw.c_getAlgaeIntakeCommand()).withTimeout(1.5)
  //                                             .andThen(s_AlgaeRotator.setGoal(0)));


  //   //Algae Auto Processor Command 
  //  JoystickButton processorButton = new JoystickButton(elevatorBoard, 6);
  //       processorButton.onTrue(s_Elevator.setElevatorHeight(Constants.ElevatorConstants.k_Processor).withTimeout(2).andThen(
  //                                             new ParallelCommandGroup(s_AlgaeRotator.setGoal(180)),//set the correct scoring angle
  //                                             new WaitCommand(1).andThen(s_AlgaeClaw.c_getAlgaeProcessorCommand()).withTimeout(1).andThen(
  //                                               new ParallelCommandGroup(s_AlgaeRotator.setGoal(0),
  //                                                     new WaitCommand(.25).andThen(s_Elevator.setElevatoorZero()))).withTimeout(1)));
        
   
                                                    
        
        



    //Reef Sides

    // Reef AB
    JoystickButton abPositionButton = new JoystickButton(positioningBoard, 4);
    abPositionButton.whileTrue(targetingSystem.setBranchCommand(ReefBranch.AB)
    .andThen(drivebase.driveToPose(targetingSystem.getTargetReefBranchPose())));

    // // Reef CD         
    // JoystickButton cdPositionButton = new JoystickButton(positioningBoard, 5);
    //     cdPositionButton.onTrue(targetingSystem.setBranchCommand(ReefBranch.CD)
    //     .andThen(drivebase.driveToPose(targetingSystem.getTargetReefBranchPose())));

    // // Reef EF
    // JoystickButton efPositionButton = new JoystickButton(positioningBoard, 6);
    //     efPositionButton.onTrue(targetingSystem.setBranchCommand(ReefBranch.EF)
    //       .andThen(drivebase.driveToPose(targetingSystem.getTargetReefBranchPose())));

    // Reef GH
    JoystickButton jhPositionButton = new JoystickButton(positioningBoard, 7);
        jhPositionButton.whileTrue(targetingSystem.setBranchCommand(ReefBranch.GH)
          .andThen(drivebase.driveToPose(targetingSystem.getTargetReefBranchPose())));
          
    // // Reef IJ
    // JoystickButton ijPositionButton = new JoystickButton(positioningBoard, 8);
    //     ijPositionButton.onTrue(targetingSystem.setBranchCommand(ReefBranch.IJ)
    //       .andThen(drivebase.driveToPose(targetingSystem.getTargetReefBranchPose())));

    // // // Reef KL
    // JoystickButton klPositionButton = new JoystickButton(positioningBoard, 9);
    //     klPositionButton.onTrue(targetingSystem.setBranchCommand(ReefBranch.KL)
    //       .andThen(drivebase.driveToPose(targetingSystem.getTargetReefBranchPose())));






  //   // A Button -> Elevator/Arm to level 2 position
  //   opperatorXbox.a().onTrue(s_Elevator.setSetpointCommand(Setpoint.k_L2).alongWith(Blinkin.setRedChase()));
  
  //   // A Button -> Elevator/Arm to level 2 position
 // opperatorXbox.a().onTrue(s_Elevator.setSetpointCommand(Setpoint.k_L2));



  //   // X Button -> Elevator/Arm to level 3 position
  //   opperatorXbox.x().onTrue(s_Elevator.setSetpointCommand(Setpoint.k_L1));
    
  //   // Y Button -> Elevator/Arm to level 4 position
  //   opperatorXbox.y().onTrue(s_Elevator.setSetpointCommand(Setpoint.k_L3));

    //Elevator
   // opperatorXbox.rightBumper().onTrue(s_Elevator.c_ElevatorUpCommand());
   // opperatorXbox.leftBumper().onTrue(s_Elevator.c_ElevatorDownCommand());
    
    //opperatorXbox.x().whileTrue(s_Elevator.c_GetElevatorUpCommand());
    //opperatorXbox.y().whileTrue(s_Elevator.c_GetElevatorDownCommand());

    JoystickButton leftPositionButton = new JoystickButton(positioningBoard, 1);
    JoystickButton middlePositionButton = new JoystickButton(positioningBoard, 3);
    JoystickButton rightPositionButton = new JoystickButton(positioningBoard, 2);

    leftPositionButton.onTrue(targetingSystem.setReefSide(ReefSide.Left));
    middlePositionButton.onTrue(targetingSystem.setReefSide(ReefSide.Right));
    rightPositionButton.onTrue(targetingSystem.setReefSide(ReefSide.Middle));

    opperatorXbox.a().onTrue(s_Elevator.setGoal(Units.inchesToMeters(0))); //Full down
    opperatorXbox.b().onTrue(s_Elevator.setGoal(Units.inchesToMeters(27.75))); //L2
    opperatorXbox.x().onTrue(s_Elevator.setGoal(Units.inchesToMeters(43.625)));//L3
    opperatorXbox.y().onTrue(s_Elevator.setGoal(Units.inchesToMeters(68.875)));//L4

    //Set hight for coral funnel
    JoystickButton coralIntakeHeighButton = new JoystickButton(elevatorBoard, 5);
    coralIntakeHeighButton.onTrue(s_Elevator.setElevatorHeight(Constants.ElevatorConstants.k_FeederStation));

    //Run coral intake
    JoystickButton coralIntakeButtomn = new JoystickButton(elevatorBoard, 11);
    coralIntakeButtomn.whileTrue(s_CoralFunnel.c_getFunnelWheelCommand());

    JoystickButton coralReverseIntakeButton = new JoystickButton(elevatorBoard, 12);
    coralReverseIntakeButton.whileTrue(s_CoralFunnel.c_getFunnelWheelCommandback());

    //L2 Auto Score
    JoystickButton L2Button = new JoystickButton(elevatorBoard, 2);//set correct number
        L2Button.onTrue(new ParallelCommandGroup(s_Elevator.setElevatorHeight(Constants.ElevatorConstants.k_L2),
                                                  new WaitCommand(.75).andThen(s_CoralPlacer.c_getCoralPlacerGenCommand()))
                                                   .withTimeout(2).andThen(s_Elevator.setElevatoorZero()));

                                                  
    //L3 Auto Score
    JoystickButton L3Button = new JoystickButton(elevatorBoard, 3);//set correct number
        L3Button.onTrue(new ParallelCommandGroup(s_Elevator.setElevatorHeight(Constants.ElevatorConstants.k_L3),
                                        new WaitCommand(1.5).andThen(s_CoralPlacer.c_getCoralPlacerGenCommand()).withTimeout(2))
                                                .withTimeout(2.6).andThen(s_Elevator.setElevatoorZero()));


    //L4 Auto Score
    JoystickButton L4Button = new JoystickButton(elevatorBoard, 4);//set correct number
        L4Button.onTrue(s_Elevator.setElevatorHeight(Constants.ElevatorConstants.k_L4).withTimeout(3).andThen(new ParallelCommandGroup(
                                                  s_CoralPlacer.c_getCoralPlacerGenCommand(),
                                                  new WaitCommand(.6).andThen(s_Elevator.setGoal(Units.inchesToMeters(73.5)))))
                                                  .withTimeout(4.5).andThen(s_Elevator.setElevatoorZero()));
                                        
    JoystickButton ZeroButton = new JoystickButton(elevatorBoard, 1);
      ZeroButton.onTrue(s_Elevator.setElevatorHeight(0));
                                                        
         



    //Funnel intake height
    opperatorXbox.leftBumper().onTrue(s_Elevator.setGoal(Units.inchesToMeters(17.375)));

    //CoralFunnel
    opperatorXbox.rightBumper().whileTrue(s_CoralFunnel.c_getFunnelWheelCommand());
    //opperatorXbox.leftTrigger().whileTrue(s_CoralFunnel.c_getFunnelWheelCommandback());
    
     //FunnelRotator
     driverXbox.rightBumper().whileTrue(s_CoralFunnel.c_FunnelRotateCommandUp());
     driverXbox.leftBumper().whileTrue(s_CoralFunnel.c_FunnelRotateCommandDown());
 
    //CoralPlacer 
    opperatorXbox.rightTrigger().whileTrue(s_CoralPlacer.c_getCoralPlacerGenCommand());
    opperatorXbox.leftTrigger().onTrue(new ParallelCommandGroup(s_CoralPlacer.c_getCoralPlacerGenCommand().withTimeout(1.5),
                                                               new WaitCommand(.6).andThen(s_Elevator.setGoal(Units.inchesToMeters(73.5)))));
    // opperatorXbox2.rightTrigger().onTrue(s_CoralFunnel.c_AutoCoralFunnelCommand());
    
    //Algae claw height
    opperatorXbox2.a().onTrue(s_Elevator.setGoal(Units.inchesToMeters(15.5))); //a1
    opperatorXbox2.b().onTrue(s_Elevator.setGoal(Units.inchesToMeters(31))); //a2
    opperatorXbox2.y().onTrue(s_Elevator.setGoal(Units.inchesToMeters(73.875))); //barge
    
    //AlgaeClaw 
    JoystickButton algaeIntakeButton = new JoystickButton(elevatorBoard, 6);
    algaeIntakeButton.whileTrue(s_AlgaeClaw.c_getAlgaeIntakeCommand());
    // opperatorXbox2.b().whileTrue(s_AlgaeClaw.c_getAlgaeProcessorCommand());
    JoystickButton algaeReverseIntakeButton = new JoystickButton(elevatorBoard, 10);
    algaeReverseIntakeButton.whileTrue(s_AlgaeClaw.c_getAlgaeBargeCommand());

    // //AlgaeRotator
    // opperatorXbox2.leftBumper().onTrue(s_AlgaeRotator.setGoal(0));
    // opperatorXbox2.x().onTrue(s_AlgaeRotator.setGoal(260));
    // opperatorXbox2.rightBumper().onTrue(s_AlgaeRotator.setGoal(200));


   //Climber
   driverXbox.leftTrigger().whileTrue(s_climber.c_GetClimberUpCommand());
   driverXbox.rightTrigger().whileTrue(s_climber.c_GetClimberDownCommand());
    
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

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}