// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
//import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Elevator.Setpoint;
import frc.robot.subsystems.CoralFunnel;
import frc.robot.subsystems.CoralPlacer;
// import frc.robot.subsystems.AlgaeClaw;

import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
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
  
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                                "swerve/maxSwerve"));
  private final Elevator s_Elevator = new Elevator();
  private final CoralFunnel s_CoralFunnel = new CoralFunnel();
  private final CoralPlacer s_CoralPlacer = new CoralPlacer();
  // private final AlgaeClaw s_AlgaeClaw = new AlgaeClaw();
  private final Blinkin s_Blinkin = new Blinkin();


//       //Attempt at adding Switchable cases for buttons
//       private OperatorDesiredReefBranchSide currentOperatorDesiredReefBranchSide = OperatorDesiredReefBranchSide.FULL_REEFSIDE_MANUAL;
//       private OperatorDesiredReefLevel currentOperatorDesiredReefLevel = OperatorDesiredReefLevel.FULL_REEFLEVEL_MANUAL;
//       private OperatorDesiredReefHexSide currentOperatorDesiredReefHexSide = OperatorDesiredReefHexSide.FULL_REEF_HEX_SIDE_MANUAL;
  

//       private boolean isUpdated = false;
  
//       public enum OperatorDesiredReefBranchSide {
//         AUTO_LEFT,
//         AUTO_RIGHT,
//         MANUAL_LEFT,
//         MANUAL_RIGHT,
//         FULL_REEFSIDE_MANUAL
//         }


//     public enum OperatorDesiredReefLevel {
//         AUTO_REEF_L1,
//         AUTO_REEF_L2,
//         AUTO_REEF_L3,
//         AUTO_REEF_L4,
//         MANUAL_REEF_L1,
//         MANUAL_REEF_L2,
//         MANUAL_REEF_L3,
//         MANUAL_REEF_L4,
//         FULL_REEFLEVEL_MANUAL
//         }

//     public enum OperatorDesiredReefHexSide{
//         AUTO_REEF_HEX_SIDE_11,
//         AUTO_REEF_HEX_SIDE_10,
//         AUTO_REEF_HEX_SIDE_9,
//         AUTO_REEF_HEX_SIDE_8,
//         AUTO_REEF_HEX_SIDE_7,
//         AUTO_REEF_HEX_SIDE_6,

//         AUTO_REEF_HEX_SIDE_17,
//         AUTO_REEF_HEX_SIDE_18,
//         AUTO_REEF_HEX_SIDE_19,
//         AUTO_REEF_HEX_SIDE_20,
//         AUTO_REEF_HEX_SIDE_21,
//         AUTO_REEF_HEX_SIDE_22,

//         MANUAL_REEF_HEX_SIDE_11,
//         MANUAL_REEF_HEX_SIDE_10,
//         MANUAL_REEF_HEX_SIDE_9,
//         MANUAL_REEF_HEX_SIDE_8,
//         MANUAL_REEF_HEX_SIDE_7,
//         MANUAL_REEF_HEX_SIDE_6,

//         MANUAL_REEF_HEX_SIDE_17,
//         MANUAL_REEF_HEX_SIDE_18,
//         MANUAL_REEF_HEX_SIDE_19,
//         MANUAL_REEF_HEX_SIDE_20,
//         MANUAL_REEF_HEX_SIDE_21,
//         MANUAL_REEF_HEX_SIDE_22,

//         FULL_REEF_HEX_SIDE_MANUAL
//         }
  
          

//       public void setOperatorDesiredReefBranchSide(OperatorDesiredReefBranchSide pOperatorDesiredReefBranchSide) {
//           currentOperatorDesiredReefBranchSide = pOperatorDesiredReefBranchSide;
//           setUpdated(false);
//       }
//       public OperatorDesiredReefBranchSide getOperatorDesiredReefBranchSide() {
//           return currentOperatorDesiredReefBranchSide;
//       }
      
      
//       public void setOperatorDesiredReefLevel(OperatorDesiredReefLevel pOperatorDesiredReefLevel) {
//           currentOperatorDesiredReefLevel = pOperatorDesiredReefLevel;
//       }
//       public OperatorDesiredReefLevel getOperatorDesiredReefLevel() {
//           return currentOperatorDesiredReefLevel;
//       }


//       public void setOperatorDesiredReefHexSide(OperatorDesiredReefHexSide pOperatorDesiredReefHexSide) {
//           currentOperatorDesiredReefHexSide = pOperatorDesiredReefHexSide;
//           setUpdated(false);
//       }
//       public OperatorDesiredReefHexSide getOperatorDesiredReefHexSide() {
//           return currentOperatorDesiredReefHexSide;
//       }
  
//       public boolean isUpdated() {
//         return isUpdated;
//     }

//     public void setUpdated(boolean updated) {
//         isUpdated = updated;
//     }
  
  

//     switch (this.getOperatorDesiredReefHexSide()) {
//       case AUTO_REEF_HEX_SIDE_11 -> {
        
//     }
//       case AUTO_REEF_HEX_SIDE_10 -> {
        

//     }
//     case AUTO_REEF_HEX_SIDE_9 -> {
        
//     }
// }




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
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
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
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
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
      driverXbox.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.rightBumper().onTrue(Commands.none());
    }


    // A Button -> Elevator/Arm to level 2 position
    opperatorXbox.a().onTrue(s_Elevator.setSetpointCommand(Setpoint.k_L2).alongWith(Blinkin.setRedChase()));
    
    // X Button -> Elevator/Arm to level 3 position
    opperatorXbox.x().onTrue(s_Elevator.setSetpointCommand(Setpoint.k_L3).alongWith(Blinkin.setPat1LarScan()));
    
    // Y Button -> Elevator/Arm to level 4 position
    opperatorXbox.y().onTrue(s_Elevator.setSetpointCommand(Setpoint.k_L4).alongWith(Blinkin.setHotPink()));

    //Elevator
    opperatorXbox.povUp().onTrue(s_Elevator.c_ElevatorUpCommand());
    opperatorXbox.povDown().onTrue(s_Elevator.c_ElevatorDownCommand());


    // Old prototype commands, 
    // *TODO* either reconfigure or adjust to be used. Disabled so buttons don't interfere

    //CoralFunnel
    opperatorXbox.rightTrigger().whileTrue(s_CoralFunnel.c_getFunnelWheelCommand());
    opperatorXbox.leftTrigger().whileTrue(s_CoralFunnel.c_getFunnelWheelCommandext());

    //CoralPlacer 
    opperatorXbox.leftBumper().whileTrue(s_CoralPlacer.c_getCoralPlacerL1Command());
    opperatorXbox.rightBumper().whileTrue(s_CoralPlacer.c_getCoralPlacerGenCommand());
    
    //AlgaeClaw 
    //opperatorXbox.leftBumper().whileTrue(s_AlgaeClaw.c_getAlgaeIntakeCommand());
    //opperatorXbox.rightBumper().whileTrue(s_AlgaeClaw.c_getAlgaeProcessorCommand());
    //opperatorXbox.leftTrigger().whileTrue(s_AlgaeClaw.c_getAlgaeBargeCommand());
  
    
  


  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}