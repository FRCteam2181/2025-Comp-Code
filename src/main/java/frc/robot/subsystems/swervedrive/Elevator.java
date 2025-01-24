package frc.robot.subsystems.swervedrive;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

import java.util.concurrent.PriorityBlockingQueue;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
        private SparkMax m_ElevatorLeft;
        private SparkMax m_ElevatorRight;
        private SparkMaxConfig leftConfig;
        private SparkMaxConfig rightConfig;
        private SparkClosedLoopController leftCLC;
        private SparkClosedLoopController rightCLC;
        private RelativeEncoder rightEncoder;
        private RelativeEncoder leftEncoder;
    
    public Elevator() {
    
        m_ElevatorLeft = new SparkMax(Constants.ElevatorConstants.k_ElevatorLeftID, MotorType.kBrushless);
        leftCLC = m_ElevatorLeft.getClosedLoopController();
        leftEncoder = m_ElevatorLeft.getEncoder();

        m_ElevatorRight = new SparkMax(Constants.ElevatorConstants.k_ElevatorRightID, MotorType.kBrushless);
        rightCLC = m_ElevatorRight.getClosedLoopController();
        rightEncoder = m_ElevatorRight.getEncoder();

        leftConfig = new SparkMaxConfig();
        rightConfig = new SparkMaxConfig();


        leftConfig.encoder.positionConversionFactor(1);
        leftConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // Set PID values for position control. We don't need to pass a closed
            // loop slot, as it will default to slot 0.
            .p(0.4)
            .i(0)
            .d(0)
            .outputRange(-1, 1);
        leftConfig.closedLoop.maxMotion
            // Set MAXMotion parameters for position control. We don't need to pass
            // a closed loop slot, as it will default to slot 0.   
            .maxVelocity(1000)
            .maxAcceleration(1000)
            .allowedClosedLoopError(1);

        
        rightConfig.encoder.positionConversionFactor(1);
        rightConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // Set PID values for position control. We don't need to pass a closed
            // loop slot, as it will default to slot 0.
            .p(0.4)
            .i(0)
            .d(0)
            .outputRange(-1, 1);
        rightConfig.closedLoop.maxMotion
            // Set MAXMotion parameters for position control. We don't need to pass
            // a closed loop slot, as it will default to slot 0.   
            .maxVelocity(1000)
            .maxAcceleration(1000)
            .allowedClosedLoopError(1);

        /*
        * Apply the configuration to the SPARK MAX.
        *
        * kResetSafeParameters is used to get the SPARK MAX to a known state. This
        * is useful in case the SPARK MAX is replaced.
        *
        * kPersistParameters is used to ensure the configuration is not lost when
        * the SPARK MAX loses power. This is useful for power cycles that may occur
        * mid-operation.
        */        
        m_ElevatorLeft.configure(leftConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
        m_ElevatorRight.configure(rightConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters); 

        // Initialize dashboard values
        // SmartDashboard.setDefaultNumber("Target Position", 0);
        // SmartDashboard.setDefaultNumber("Target Velocity", 0);
        // SmartDashboard.setDefaultBoolean("Control Mode", false);
        // SmartDashboard.setDefaultBoolean("Reset Encoder", false);
        
    }

    public Command c_ElevatorToPosition(double position) {
        return this.runEnd(
        () -> {
            leftCLC.setReference(position, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
            rightCLC.setReference(position, ControlType.kMAXMotionPositionControl, ClosedLoopSlot.kSlot0);
        }, 
        () -> {
            m_ElevatorLeft.set(0);
            m_ElevatorRight.set(0);
        });
    } 

    public Command c_ElevatorMove(double speed) {
        return this.runEnd(
        () -> {
            m_ElevatorLeft.set(speed);
            m_ElevatorRight.set(speed);
        }, 
        () -> {
            m_ElevatorLeft.set(0);
            m_ElevatorRight.set(0);
        });
    } 
}
