// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Millimeters;
import static edu.wpi.first.units.Units.Rotations;


import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;
import au.grapplerobotics.interfaces.LaserCanInterface.RegionOfInterest;
import au.grapplerobotics.interfaces.LaserCanInterface.TimingBudget;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.RobotMath.Elevator;
import frc.robot.Setpoints;
import frc.robot.Constants.ElevatorConstants;



public class ElevatorSubsystemPID extends SubsystemBase
{

  // This gearbox represents a gearbox containing 1 Neo
  private SparkMax m_ElevatorLeft;
  private SparkMax m_ElevatorRight;
  private RelativeEncoder elevatorEncoder;
  private boolean autoZero = true;
 
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


  // Sensors
  private final LaserCan         m_elevatorLaserCan     = new LaserCan(ElevatorConstants.elevatorLaserCanID);
  private final RegionOfInterest m_laserCanROI          = new RegionOfInterest(0, 0, 4, 4);//TODO Change
  private final TimingBudget     m_laserCanTimingBudget = TimingBudget.TIMING_BUDGET_33MS;
  private final Alert            m_laserCanFailure      = new Alert("LaserCAN failed to configure.",
                                                                     AlertType.kError);
  







  /**
   * Subsystem constructor.
   */
  public ElevatorSubsystemPID() {

    m_ElevatorLeft = new SparkMax(ElevatorConstants.k_ElevatorLeftID, MotorType.kBrushless);
    m_ElevatorRight = new SparkMax(ElevatorConstants.k_ElevatorRightID, MotorType.kBrushless);
    elevatorEncoder = m_ElevatorLeft.getEncoder();


    m_ElevatorRight.configure(
            Configs.ElevatorConfigs.elevatorFollowerConfig.follow(ElevatorConstants.k_ElevatorLeftID), 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters);

    m_ElevatorLeft.configure( 
            Configs.ElevatorConfigs.elevatorConfig, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters);
  

            try
            {
             
              m_elevatorLaserCan.setRangingMode(RangingMode.SHORT);
              m_elevatorLaserCan.setTimingBudget(m_laserCanTimingBudget);
              m_elevatorLaserCan.setRegionOfInterest(m_laserCanROI);
            
            } catch (Exception e)
            {
              m_laserCanFailure.set(true);
            }




    seedElevatorMotorPosition();

  }

  public Command autoZeroSwitchCommand() {
    return run(() -> autoZeroSwitch() );
  }

  public void autoZeroSwitch() {
    autoZero = false;
    CommandScheduler.getInstance().cancelAll();
  }

  public Command setElevatoorZero() {
    if (autoZero) {
     return setElevatorHeight(0);
    } else {
      return setElevatorHeight(ElevatorConstants.k_FeederStation); 
    }
  }


  /**
   * Seed the elevator motor encoder with the sensed position from the LaserCAN which tells us the height of the
   * elevator.
   */
  public void seedElevatorMotorPosition()
  {
       Measurement seedMeasurement = m_elevatorLaserCan.getMeasurement();
      while (seedMeasurement == null)
      {
        seedMeasurement = m_elevatorLaserCan.getMeasurement();
      }

      elevatorEncoder.setPosition(Elevator.convertDistanceToRotations(Millimeters.of(
                                        m_elevatorLaserCan.getMeasurement().distance_mm - ElevatorConstants.kLaserCANOffset.in(Millimeters)))
                                    .in(Rotations));
  
  }


 /**
   * Seed the elevator motor encoder with the sensed position from the LaserCAN which tells us the height of the
   * elevator.
   */
  public void ReseedElevatorMotorPosition()
  {
      Measurement ReseedMeasurement = m_elevatorLaserCan.getMeasurement();
      while (ReseedMeasurement == null)
      {
        ReseedMeasurement = m_elevatorLaserCan.getMeasurement();
      }

      elevatorEncoder.setPosition(Elevator.convertDistanceToRotations(Millimeters.of(
                                        m_elevatorLaserCan.getMeasurement().distance_mm - ElevatorConstants.kLaserCANOffset.in(Millimeters)))
                                    .in(Rotations));
  
  }

  public Command ReseedElevator()
  {
    return runOnce(() -> ReseedElevatorMotorPosition());
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
        -8,
        8); // 7 is the max voltage to send out.
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

  //   return Commands.run(() -> {
  //     setGoal(height);
  // }, this).withInterruptBehavior(InterruptionBehavior.kCancelIncoming).until(aroundFeederStation().negate()).finallyDo(() -> {
      
  //     stop();
  // });
    
  }



 /**
   * Set the elevator goal and stop when it reaches its target.
   *
   * @param height Height in meters.
   * @return Command which ends when the elevator is near the target height.
   */
  public Command setElevatorHeightUntil(double height)
  {
    
    return Commands.run(() -> {
          reachGoal(height);
      }, this).withInterruptBehavior(InterruptionBehavior.kCancelIncoming).until(() -> aroundHeight(height)).finallyDo(() -> {
          
          stop();
      });

  }







  /**
   * Stop the control loop and motor output.
   */
  public void stop()
  {
    m_ElevatorLeft.set(0.0);
  }


  public Command Hold()
  {
    return runOnce(() -> stop());
  }
 

  @Override
  public void periodic()
  {
    Measurement laserCanMeasurement = m_elevatorLaserCan.getMeasurement();
    if (laserCanMeasurement != null)
    {
      SmartDashboard.putNumber("Elevator LaserCAN (Meters)",
                               (Millimeters.of(laserCanMeasurement.distance_mm).in(Meters)*3));
    }
      SmartDashboard.putNumber("Elevator Height (Meters)", getHeightMeters());
      
     

  }





 //Algae Auto Ground Intake Command 
 public Trigger aroundAlgaeGroundIntake()
 {
   return new Trigger(() -> aroundHeight(0));
 }  
 

    //Algae Auto Dunk Command
    public Trigger aroundAlgaeBarge()
    {
      return new Trigger(() -> aroundHeight(ElevatorConstants.k_Net));
    }
    

    //Algae retrival from A2
    public Trigger aroundAlgaeA2()
    {
      return new Trigger(() -> aroundHeight(ElevatorConstants.k_A2));
    }                                          
    

    //Algae retrival from A1

    public Trigger aroundAlgaeA1()
    {
      return new Trigger(() -> aroundHeight(ElevatorConstants.k_A1));
    }
    

    //Algae Auto Processor Command 

    public Trigger aroundAlgaePROCESSOR()
    {
      return new Trigger(() -> aroundHeight(0));
    }
    

        
    //Set Elevator to intake height for coral funnel
    public Trigger aroundFeederStation()
    {
      return new Trigger(() -> aroundHeight(ElevatorConstants.k_FeederStation));
    }
    

    //L4 Auto Score
    public Trigger aroundCoralL4()
    {
      return new Trigger(() -> aroundHeight(ElevatorConstants.k_L4));
    }
    

    //L3 Auto Score
    public Trigger aroundCoralL3()
    {
      return new Trigger(() -> aroundHeight(ElevatorConstants.k_L3));
    }
    

    //L2 Auto Score
    public Trigger aroundCoralL2()
    {
      return new Trigger(() -> aroundHeight(ElevatorConstants.k_L2));
    }
    

    //Manually return Elevator to 0 
    public Trigger aroundElevatorZero()
    {
      return new Trigger(() -> aroundHeight(0));
    }           
  

    //Bump up height
    public Trigger aroundL4BumpUp()
    {
      return new Trigger(() -> aroundHeight(ElevatorConstants.k_L4BumpUP));
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