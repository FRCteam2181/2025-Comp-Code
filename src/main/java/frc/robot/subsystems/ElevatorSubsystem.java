// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Configs;


import frc.robot.Constants.ElevatorConstants;


public class ElevatorSubsystem extends SubsystemBase
{

  // This gearbox represents a gearbox containing 1 Neo
  private SparkMax m_ElevatorLeft;
  private SparkMax m_ElevatorRight;
  private RelativeEncoder elevatorEncoder;
 
  // Closed Loop Controller + Feedback
  private final ProfiledPIDController m_controller  = new ProfiledPIDController(ElevatorConstants.kElevatorKp,
                                                                                ElevatorConstants.kElevatorKi,
                                                                                ElevatorConstants.kElevatorKd,
                                                                                new Constraints(ElevatorConstants.kMaxVelocity,
                                                                                                ElevatorConstants.kMaxAcceleration));
  private final ElevatorFeedforward   m_feedforward =
      new ElevatorFeedforward(
          ElevatorConstants.kElevatorkS,
          ElevatorConstants.kElevatorkG,
          ElevatorConstants.kElevatorkV,
          ElevatorConstants.kElevatorkA);

  public final  Trigger           atMin            = new Trigger(() -> MathUtil.isNear(getHeightMeters(),
                                                                                       ElevatorConstants.kMinElevatorHeightMeters,
                                                                                       Inches.of(3).in(Meters)
                                                                                      ));
  public final  Trigger           atMax            = new Trigger(() -> MathUtil.isNear(getHeightMeters(),
                                                                                       ElevatorConstants.kMinElevatorHeightMeters,
                                                                                       Inches.of(3).in(Meters)
                                                                                      ));


  /**
   * Subsystem constructor.
   */
  public ElevatorSubsystem() {

    m_ElevatorLeft = new SparkMax(ElevatorConstants.k_ElevatorLeftID, MotorType.kBrushless);
    m_ElevatorRight = new SparkMax(ElevatorConstants.k_ElevatorRightID, MotorType.kBrushless);
    elevatorEncoder = m_ElevatorLeft.getEncoder();


    m_ElevatorRight.configure(
            Configs.ElevatorConfig.elevatorFollowerConfig.follow(ElevatorConstants.k_ElevatorLeftID), 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters);

    m_ElevatorLeft.configure( 
            Configs.ElevatorConfig.elevatorConfig, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters);
    
    seedElevatorMotorPosition();

  }


  /**
   * Seed the elevator motor encoder with the sensed position from the LaserCAN which tells us the height of the
   * elevator.
   */
  public void seedElevatorMotorPosition()
  {
      elevatorEncoder.setPosition(0);
  
  }

  /**
   * Run control loop to reach and maintain goal.
   *
   * @param goal the position to maintain in meters.
   */
  public void reachGoal(double goal)
  {
    double voltsOut = MathUtil.clamp(
        m_controller.calculate(getHeightMeters(), goal) +
        m_feedforward.calculateWithVelocities(getVelocityMetersPerSecond(),
                                              m_controller.getSetpoint().velocity),
        -7,
        7); // 7 is the max voltage to send out.
        m_ElevatorLeft.setVoltage(voltsOut);
  }

 
  /**
   * Get the height in meters.
   *
   * @return Height in meters
   */
  public double getHeightMeters()
  {
    return (((elevatorEncoder.getPosition() / ElevatorConstants.kElevatorGearing) *
           (Math.PI * ElevatorConstants.kElevatorDrumDiameter))*3);
  }

  /**
   * The velocity of the elevator in meters per second.
   *
   * @return velocity in meters per second
   */
  public double getVelocityMetersPerSecond()
  {
    return ((((elevatorEncoder.getVelocity() / 60) / ElevatorConstants.kElevatorGearing) *
           (Math.PI * ElevatorConstants.kElevatorDrumDiameter))*3);
  }

  /**
   * A trigger for when the height is at an acceptable tolerance.
   *
   * @param height    Height in Meters
   * @param tolerance Tolerance in meters.
   * @return {@link Trigger}
   */
  public Trigger atHeight(double height, double tolerance)
  {
    return new Trigger(() -> MathUtil.isNear(height,
                                             getHeightMeters(),
                                             tolerance));
  }

  /**
   * Set the goal of the elevator
   *
   * @param goal Goal in meters
   * @return {@link edu.wpi.first.wpilibj2.command.Command}
   */
  public Command setGoal(double goal)
  {
    return run(() -> reachGoal(goal));
  }


  /**
   * Set the elevator goal and stop when it reaches its target.
   *
   * @param height Height in meters.
   * @return Command which ends when the elevator is near the target height.
   */
  public Command setElevatorHeight(double height)
  {
    return setGoal(height).until(() -> aroundHeight(height));
  }


  /**
   * Stop the control loop and motor output.
   */
  public void stop()
  {
    m_ElevatorLeft.set(0.0);
  }

 

  @Override
  public void periodic()
  {
  }

  /**
   * Gets the height of the elevator and compares it to the given height with the given tolerance.
   *
   * @param height         Height in meters
   * @param allowableError Tolerance in meters.
   * @return Within that tolerance.
   */
  public boolean aroundHeight(double height, double allowableError)
  {
    return MathUtil.isNear(height, getHeightMeters(), allowableError);
  }

  /**
   * Gets the height of the elevator and compares it to the given height with the given tolerance.
   *
   * @param height Height in meters
   * @return Within that tolerance.
   */
  public boolean aroundHeight(double height)
  {
    return aroundHeight(height, Units.inchesToMeters(ElevatorConstants.kElevatorAllowableError));
  }


}