package frc.robot.systems;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.systems.field.AllianceFlipUtil;
import frc.robot.systems.field.FieldConstants.Reef;
import frc.robot.systems.field.FieldConstants.ReefHeight;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import frc.robot.Constants.TargetingConstants;

//targetting system should be able to select either left or right side of the branch
//then select what level we want
// that go to the nearest side of the reef and load.


public class TargetingSystem
{

  private AprilTagFieldLayout fieldLayout              = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);
  private ReefBranch targetBranch;
  private ReefBranchLevel targetLevel;
  private DesiredCage targetCage;
  private ProcessorScoring targetProcessor;
  private CoralLoadingPosition targetCoralLoading;
  private Transform2d         robotBranchScoringOffset = new Transform2d(Inches.of(19).in(Meters),
                                                                         Inches.of(0).in(Meters),
                                                                         Rotation2d.fromDegrees(0));

  private List<Pose2d>            reefBranches                 = null;
  private List<Pose2d>            allianceRelativeReefBranches = null;
  private Map<Pose2d, ReefBranch> reefPoseToBranchMap          = null;

  public TargetingSystem()
  {
    reefBranches = new ArrayList<>();
    reefPoseToBranchMap = new HashMap<>();
    for (int branchPositionIndex = 0; branchPositionIndex < Reef.branchPositions.size(); branchPositionIndex++)
    {
      Map<ReefHeight, Pose3d> branchPosition = Reef.branchPositions.get(branchPositionIndex);
      Pose2d                  targetPose     = AllianceFlipUtil.apply(branchPosition.get(ReefHeight.L4).toPose2d());
      reefBranches.add(targetPose);
      reefPoseToBranchMap.put(targetPose, ReefBranch.values()[branchPositionIndex]);
      reefPoseToBranchMap.put(AllianceFlipUtil.flip(targetPose), ReefBranch.values()[branchPositionIndex]);
    }
  }

  

    public Pose2d getTargetReefBranchPose()
    {
      switch (targetBranch)
      {
        case A ->
        {
          if(isRedAlliance()){
            return TargetingConstants.ReefBranch_Red_A;}
            else{
            return TargetingConstants.ReefBranch_Blue_A;
            }
        }
        
        case B ->
        {
          if(isRedAlliance()){
            return TargetingConstants.ReefBranch_Red_B;}
            else{
            return TargetingConstants.ReefBranch_Blue_B;
            }
        }

        case C ->
        {
          if(isRedAlliance()){
            return TargetingConstants.ReefBranch_Red_C;}
            else{
            return TargetingConstants.ReefBranch_Blue_C;
            }
        }

        case D ->
        {
          if(isRedAlliance()){
            return TargetingConstants.ReefBranch_Red_D;}
            else{
            return TargetingConstants.ReefBranch_Blue_D;
            }
        }

        case E ->
        {
          if(isRedAlliance()){
            return TargetingConstants.ReefBranch_Red_E;}
            else{
            return TargetingConstants.ReefBranch_Blue_E;
            }
        }

        case F ->
        {
          if(isRedAlliance()){
            return TargetingConstants.ReefBranch_Red_F;}
            else{
            return TargetingConstants.ReefBranch_Blue_F;
            }
        }

        case G ->
        {
          if(isRedAlliance()){
            return TargetingConstants.ReefBranch_Red_G;}
            else{
            return TargetingConstants.ReefBranch_Blue_G;
            }
        }

        case H ->
        {
          if(isRedAlliance()){
            return TargetingConstants.ReefBranch_Red_H;}
            else{
            return TargetingConstants.ReefBranch_Blue_H;
            }
        }

        case I ->
        {
          if(isRedAlliance()){
            return TargetingConstants.ReefBranch_Red_I;}
            else{
            return TargetingConstants.ReefBranch_Blue_I;
            }
        }

        case J ->
        {
          if(isRedAlliance()){
            return TargetingConstants.ReefBranch_Red_J;}
            else{
            return TargetingConstants.ReefBranch_Blue_J;
            }
        }

        case K ->
        {
          if(isRedAlliance()){
            return TargetingConstants.ReefBranch_Red_K;}
            else{
            return TargetingConstants.ReefBranch_Blue_K;
            }
        }

        case L ->
        {
          if(isRedAlliance()){
            return TargetingConstants.ReefBranch_Red_L;}
            else{
            return TargetingConstants.ReefBranch_Blue_L;
            }
        }
      }
      return new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));
    }


    public Distance getTargetReefBranchLevelHeight()
    {
      switch (targetLevel)
      {
        case L1 ->
        {
          return TargetingConstants.ReefBranchHeight_L1;
        }

        case L2 ->
        {
          return TargetingConstants.ReefBranchHeight_L2;
        }

        case L3 ->
        {
          return TargetingConstants.ReefBranchHeight_L3;
        }

        case L4 ->
        {
          return TargetingConstants.ReefBranchHeight_L4;
        }

        case A_Low ->
        {
          return TargetingConstants.ReefBranchHeight_A_Low;
        }

        case A_High ->
        {
          return TargetingConstants.ReefBranchHeight_A_High;
        }
        
      }
      return Meters.of(0);
    }


    public Pose2d getTargetDesiredCagePose()
    {
      switch (targetCage)
      {
        case Cage_1 ->
        {
          if(isRedAlliance()){
            return TargetingConstants.DesiredCage_Red_Cage_1;}
            else{
            return TargetingConstants.DesiredCage_Blue_Cage_1;
            }
        }

        case Cage_2 ->
        {
          if(isRedAlliance()){
            return TargetingConstants.DesiredCage_Red_Cage_2;}
            else{
            return TargetingConstants.DesiredCage_Blue_Cage_2;
            }
        }

        case Cage_3 ->
        {
          if(isRedAlliance()){
            return TargetingConstants.DesiredCage_Red_Cage_3;}
            else{
            return TargetingConstants.DesiredCage_Blue_Cage_3;
            }
        }
      }
      return new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));
        
    }


    public Pose2d getTargetProcessorScoringPose()
    {
      switch (targetProcessor)
      {
        case Red_Processor ->
        {
            return TargetingConstants.Red_Processor;
          }

          case Blue_Processor ->
        {
            return TargetingConstants.Blue_Processor;
          }
      }
      return new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));
        
    }


    public Pose2d getTargetCoralLoadingPositionPose()
    {
      switch (targetCoralLoading)
      {
        case Left_1 ->
        {
          if(isRedAlliance()){
            return TargetingConstants.CoralFeed_Red_Left_1;}
            else{
            return TargetingConstants.CoralFeed_Blue_Left_1;
            }
        }

        case Left_2 ->
        {
          if(isRedAlliance()){
            return TargetingConstants.CoralFeed_Red_Left_2;}
            else{
            return TargetingConstants.CoralFeed_Blue_Left_2;
            }
        }

        case Left_3 ->
        {
          if(isRedAlliance()){
            return TargetingConstants.CoralFeed_Red_Left_3;}
            else{
            return TargetingConstants.CoralFeed_Blue_Left_3;
            }
        }

        case Left_4 ->
        {
          if(isRedAlliance()){
            return TargetingConstants.CoralFeed_Red_Left_4;}
            else{
            return TargetingConstants.CoralFeed_Blue_Left_4;
            }
        }

        case Left_5 ->
        {
          if(isRedAlliance()){
            return TargetingConstants.CoralFeed_Red_Left_5;}
            else{
            return TargetingConstants.CoralFeed_Blue_Left_5;
            }
        }

        case Left_6 ->
        {
          if(isRedAlliance()){
            return TargetingConstants.CoralFeed_Red_Left_6;}
            else{
            return TargetingConstants.CoralFeed_Blue_Left_6;
            }
        }

        case Left_7 ->
        {
          if(isRedAlliance()){
            return TargetingConstants.CoralFeed_Red_Left_7;}
            else{
            return TargetingConstants.CoralFeed_Blue_Left_7;
            }
        }

        case Left_8 ->
        {
          if(isRedAlliance()){
            return TargetingConstants.CoralFeed_Red_Left_8;}
            else{
            return TargetingConstants.CoralFeed_Blue_Left_8;
            }
        }

        case Left_9 ->
        {
          if(isRedAlliance()){
            return TargetingConstants.CoralFeed_Red_Left_9;}
            else{
            return TargetingConstants.CoralFeed_Blue_Left_9;
            }
        }

        case Right_1 ->
        {
          if(isRedAlliance()){
            return TargetingConstants.CoralFeed_Red_Right_1;}
            else{
            return TargetingConstants.CoralFeed_Blue_Right_1;
            }
        }

        case Right_2 ->
        {
          if(isRedAlliance()){
            return TargetingConstants.CoralFeed_Red_Right_2;}
            else{
            return TargetingConstants.CoralFeed_Blue_Right_2;
            }
        }

        case Right_3 ->
        {
          if(isRedAlliance()){
            return TargetingConstants.CoralFeed_Red_Right_3;}
            else{
            return TargetingConstants.CoralFeed_Blue_Right_3;
            }
        }

        case Right_4 ->
        {
          if(isRedAlliance()){
            return TargetingConstants.CoralFeed_Red_Right_4;}
            else{
            return TargetingConstants.CoralFeed_Blue_Right_4;
            }
        }

        case Right_5 ->
        {
          if(isRedAlliance()){
            return TargetingConstants.CoralFeed_Red_Right_5;}
            else{
            return TargetingConstants.CoralFeed_Blue_Right_5;
            }
        }

        case Right_6 ->
        {
          if(isRedAlliance()){
            return TargetingConstants.CoralFeed_Red_Right_6;}
            else{
            return TargetingConstants.CoralFeed_Blue_Right_6;
            }
        }

        case Right_7 ->
        {
          if(isRedAlliance()){
            return TargetingConstants.CoralFeed_Red_Right_7;}
            else{
            return TargetingConstants.CoralFeed_Blue_Right_7;
            }
        }

        case Right_8 ->
        {
          if(isRedAlliance()){
            return TargetingConstants.CoralFeed_Red_Right_8;}
            else{
            return TargetingConstants.CoralFeed_Blue_Right_8;
            }
        }

        case Right_9 ->
        {
          if(isRedAlliance()){
            return TargetingConstants.CoralFeed_Red_Right_9;}
            else{
            return TargetingConstants.CoralFeed_Blue_Right_9;
            }
        }
      }
      return new Pose2d(
          Units.inchesToMeters(0),
          Units.inchesToMeters(0),
          Rotation2d.fromDegrees(0));
        
    }


//These commands will change the variable that we are targeting 

  public Command setBranchCommand(ReefBranch branch)
  {
    return Commands.runOnce(() -> {
      targetBranch = branch;
    });
  }

  public Command setBranchCommand(ReefBranchLevel level)
  {
    return Commands.runOnce(() -> {
      targetLevel = level;
    });
  }

  public Command setBranchCommand(DesiredCage cage)
  {
    return Commands.runOnce(() -> {
      targetCage = cage;
    });
  }

  public Command setBranchCommand(ProcessorScoring processor)
  {
    return Commands.runOnce(() -> {
      targetProcessor = processor;
    });
  }

  public Command setBranchCommand(CoralLoadingPosition coralloading)
  {
    return Commands.runOnce(() -> {
      targetCoralLoading = coralloading;
    });
  }





















  
  public void setTarget(ReefBranch targetBranch, ReefBranchLevel targetBranchLevel)
  {
    this.targetBranch = targetBranch;
    this.targetLevel = targetBranchLevel;
  }

  public Command setTargetCommand(ReefBranch targetBranch, ReefBranchLevel targetBranchLevel)
  {
    return Commands.runOnce(() -> setTarget(targetBranch, targetBranchLevel));
  }

  public Pose2d getTargetPose()
  {
    Pose2d scoringPose = Pose2d.kZero;
    if (targetBranch != null)
    {
      scoringPose = Reef.branchPositions.get(targetBranch.ordinal()).get(ReefHeight.L2).toPose2d()
                                        .plus(robotBranchScoringOffset);
    }
    return AllianceFlipUtil.apply(scoringPose);
  }


  public Pose2d autoTarget(Supplier<Pose2d> currentPose)
  {
    if (allianceRelativeReefBranches == null)
    {
      allianceRelativeReefBranches = reefBranches.stream()
                                                 .map(AllianceFlipUtil::apply)
                                                 .collect(Collectors.toList());
    }
    Pose2d selectedTargetPose = currentPose.get().nearest(allianceRelativeReefBranches);
    targetBranch = reefPoseToBranchMap.get(selectedTargetPose);
    return selectedTargetPose;
  }

  public Command autoTargetCommand(Supplier<Pose2d> currentPose)
  {
    return Commands.runOnce(() -> {
      autoTarget(currentPose);
    });
  }




















  //This lists each individual branch on the reef.
  public enum ReefBranch
  {
    A,
    B,
    C,
    D,
    E,
    F,
    G,
    H,
    I,
    J,
    K,
    L
  }

//This lists each individual branch level on the reef and the fact that the trough exists (L1) even though it is ont unique to the branches.
  public enum ReefBranchLevel
  {
    L1,
    L2,
    L3,
    L4,
    A_Low,
    A_High
  }

//This lists each cage location to line up just in front of the cage in that position.
//**TODO Add a SmartDashboard selector for which cage we are aligning to */
public enum DesiredCage
{
  Cage_1,
  Cage_2,
  Cage_3,
}

//Pose for scoring into the processor.
public enum ProcessorScoring
{
  Red_Processor,
  Blue_Processor
}

// There are 9 slots that can be used on the coral station and 2 coral stations per side, this defines the targeting pose to get to each
  public enum CoralLoadingPosition
  {
    Left_1,
    Left_2,
    Left_3,
    Left_4,
    Left_5,
    Left_6,
    Left_7,
    Left_8,
    Left_9,
    Right_1,
    Right_2,
    Right_3,
    Right_4,
    Right_5,
    Right_6,
    Right_7,
    Right_8,
    Right_9,
  }

 /**
   * Checks if the alliance is red, defaults to false if alliance isn't available.
   *
   * @return true if the red alliance, false if blue. Defaults to false if none is available.
   */
  private boolean isRedAlliance()
  {
    var alliance = DriverStation.getAlliance();
    return alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  }


}
