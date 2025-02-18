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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
//import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
// import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.CoralFunnel;
import frc.robot.subsystems.CoralPlacer;
//import frc.robot.subsystems.AlgaeClaw;
//import frc.robot.subsystems.AlgaeRotator;
import frc.robot.subsystems.Climber;
import frc.robot.systems.ButtonBoard;
import frc.robot.systems.TargetingSystem;
import frc.robot.systems.TargetingSystem.ReefBranch;
//import frc.robot.systems.ScoringSystem;
import frc.robot.systems.TargetingSystem.ReefSide;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import javax.imageio.plugins.tiff.TIFFDirectory;

import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController opperatorXbox = new CommandXboxController(1);
  final CommandXboxController opperatorXbox2 = new CommandXboxController(2);

  private final ButtonBoard opperatorBoard1 = new ButtonBoard(1);

  
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/maxSwerve"));
  private final ElevatorSubsystem s_Elevator = new ElevatorSubsystem();
  private final CoralFunnel s_CoralFunnel = new CoralFunnel();
  private final CoralPlacer s_CoralPlacer = new CoralPlacer();
  //private final AlgaeClaw s_AlgaeClaw = new AlgaeClaw();
  // private final Blinkin s_Blinkin = new Blinkin();
  //private final AlgaeRotator s_AlgaeRotator = new AlgaeRotator();
  private final Climber s_climber = new Climber();

  private final TargetingSystem targetingSystem = new TargetingSystem();
  // private final ScoringSystem   scoringSystem   = new ScoringSystem(s_CoralPlacer,
  //                                                                   s_Elevator,
  //                                                                   drivebase,
  //                                                                   s_AlgaeClaw,
  //                                                                   s_AlgaeRotator,
  //                                                                   targetingSystem,
  //                                                                   s_CoralFunnel,
  //                                                                   s_climber);



// Establishing the Auto Chooser that will appear on the SmartDashboard
  private final SendableChooser<Command> autoChooser;


  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() * -1,
                                                                () -> driverXbox.getLeftX() * -1)
                                                            .withControllerRotationAxis(driverXbox::getRightX)
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
 
      
    
    
    autoChooser = AutoBuilder.buildAutoChooser("New Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser); 
 

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
      driverXbox.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driverXbox.b().whileTrue(
          drivebase.driveToPose(
              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              );
      driverXbox.start().whileTrue(Commands.none());
      driverXbox.back().whileTrue(Commands.none());
      driverXbox.y().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
    }

   //Left or Right
   opperatorBoard1.button0.onTrue(targetingSystem.setReefSide(ReefSide.Left));
   opperatorBoard1.button5.onTrue(targetingSystem.setReefSide(ReefSide.Right));
   opperatorBoard1.button17.onTrue(targetingSystem.setReefSide(ReefSide.Middle));

   //Reef Sides
   opperatorBoard1.button3.onTrue(targetingSystem.setBranchCommand(ReefBranch.AB)
      .andThen(drivebase.driveToPose(targetingSystem.getTargetReefBranchPose())));
  
   opperatorBoard1.button1.onTrue(targetingSystem.setBranchCommand(ReefBranch.CD)
      .andThen(drivebase.driveToPose(targetingSystem.getTargetReefBranchPose())));

   opperatorBoard1.button2.onTrue(targetingSystem.setBranchCommand(ReefBranch.EF)
      .andThen(drivebase.driveToPose(targetingSystem.getTargetReefBranchPose())));

   opperatorBoard1.button4.onTrue(targetingSystem.setBranchCommand(ReefBranch.GH)
      .andThen(drivebase.driveToPose(targetingSystem.getTargetReefBranchPose())));

   opperatorBoard1.button7.onTrue(targetingSystem.setBranchCommand(ReefBranch.IJ)
      .andThen(drivebase.driveToPose(targetingSystem.getTargetReefBranchPose())));

   opperatorBoard1.button6.onTrue(targetingSystem.setBranchCommand(ReefBranch.KL)
      .andThen(drivebase.driveToPose(targetingSystem.getTargetReefBranchPose())));

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
    opperatorXbox.x().onTrue(s_Elevator.setGoal(Units.inchesToMeters(2)));
    opperatorXbox.y().onTrue(s_Elevator.setGoal(Units.inchesToMeters(4)));
    opperatorXbox.a().onTrue(s_Elevator.setGoal(Units.inchesToMeters(6)));
    opperatorXbox.b().onTrue(s_Elevator.setGoal(Units.inchesToMeters(8)));

    //CoralFunnel
    opperatorXbox.rightTrigger().whileTrue(s_CoralFunnel.c_getFunnelWheelCommand());
    opperatorXbox.leftTrigger().whileTrue(s_CoralFunnel.c_getFunnelWheelCommandext());
    
    
    
     //FunnelRotator
     driverXbox.rightBumper().whileTrue(s_CoralFunnel.c_FunnelRotateCommandUp());
     driverXbox.leftBumper().whileTrue(s_CoralFunnel.c_FunnelRotateCommandDown());
 

    //CoralPlacer 
    opperatorXbox.leftBumper().whileTrue(s_CoralPlacer.c_getCoralPlacerL1Command());
    opperatorXbox.rightBumper().whileTrue(s_CoralPlacer.c_getCoralPlacerGenCommand());
    
    //AlgaeClaw 
   // opperatorXbox2.a().whileTrue(s_AlgaeClaw.c_getAlgaeIntakeCommand());
    //opperatorXbox2.b().whileTrue(s_AlgaeClaw.c_getAlgaeProcessorCommand());
    //opperatorXbox2.x().whileTrue(s_AlgaeClaw.c_getAlgaeBargeCommand());

    //AlgaeRotator
   // driverXbox.leftBumper().whileTrue(s_AlgaeRotator.c_GetAlgeaRotateDownCommand());
    //driverXbox.rightBumper().whileTrue(s_AlgaeRotator.c_GetAlgeaRotateUpCommand());


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