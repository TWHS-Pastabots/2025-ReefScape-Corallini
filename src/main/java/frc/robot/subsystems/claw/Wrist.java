package frc.robot.subsystems.claw;

import org.opencv.features2d.FlannBasedMatcher;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

public class Wrist {
    private SparkClosedLoopController ControllerR;
    private SparkClosedLoopController ControllerL;
    public SparkMax MotorR;
    public SparkMax MotorL;
    public ArmFeedforward feedforward;
    public SparkMaxConfig MotorConfigR;
    public SparkMaxConfig MotorConfigL;

    // public SparkMaxPIDController RPID;
    // public SparkMaxPIDController LPID;
    public static Wrist instance;
    private double tiltPosition;
    private double rotatePosition;

    public Wrist() {
        feedforward = new ArmFeedforward(0.0, 0.0, 0.0, 0.0);
        tiltPosition = 0;
        rotatePosition = 0;

        MotorR = new SparkMax(10, MotorType.kBrushless); //SAY THE CORRECT PORT NUMBER LATER
        MotorL = new SparkMax(9, MotorType.kBrushless); //SAY THE CORRECT PORT NUMBER LATER
        MotorConfigL = new SparkMaxConfig();
        MotorConfigR = new SparkMaxConfig();

        MotorConfigL
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(60);
        MotorConfigL.encoder
            .positionConversionFactor(360);
        MotorConfigL.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(0, 0, 0)
            .outputRange(-1, 1);
        MotorL.configure(MotorConfigL, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        MotorConfigR
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(60);
        MotorConfigL.encoder
            .positionConversionFactor(360);
        MotorConfigL.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(0, 0, 0)
            .outputRange(-1, 1);
        MotorR.configure(MotorConfigR, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        ControllerL = MotorL.getClosedLoopController();
        ControllerR = MotorR.getClosedLoopController();
    }
    public void moveWrist(double targetTilt, double targetRotation){
        tiltPosition = targetTilt;
        rotatePosition = targetRotation;

        double motorLCurrent = MotorL.getEncoder().getPosition();
        double motorRCurrent = MotorR.getEncoder().getPosition();

        double currentTilt = (motorLCurrent + motorRCurrent) /2;
        double currentRotation = (motorRCurrent - motorLCurrent)/2;

        double motorLTarget = motorLCurrent + (targetTilt - currentRotation) - (targetRotation - currentRotation);
        double motorRTarget = motorRCurrent + (targetTilt - currentRotation) + (targetRotation - currentRotation);
        
        

        ControllerL.setReference(motorLTarget, ControlType.kPosition, ClosedLoopSlot.kSlot1, feedforward.calculate(MotorL.getEncoder().getPosition(), 0));
        ControllerR.setReference(motorRTarget, ControlType.kPosition, ClosedLoopSlot.kSlot1, feedforward.calculate(MotorR.getEncoder().getPosition(), 0));

        SmartDashboard.putNumber("Tilt Position (Target)", tiltPosition);
        SmartDashboard.putNumber("Rotation Position (Target)", rotatePosition);
        SmartDashboard.putNumber("Tilt Position (Current)", currentTilt);
        SmartDashboard.putNumber("Rotation Position (Current)", currentRotation);
        SmartDashboard.putNumber("Motor L Target", motorLTarget);
        SmartDashboard.putNumber("Motor R Target", motorRTarget);
    }

    public static Wrist getInstance() {
        if (instance == null)
            instance = new Wrist();
        return instance;
    }
}
