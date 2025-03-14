package frc.robot.systems;

// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
// import frc.robot.Constants;

// import frc.robot.Setpoints;

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


}
