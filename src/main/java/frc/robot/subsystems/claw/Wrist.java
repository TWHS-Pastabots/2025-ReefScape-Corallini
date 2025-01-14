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

    public enum WristState
    {
        TEST(0,90),
        TEST2(20,0),
        TEST3(30,180),
        TEST4(50,160);

        public double tiltPosition;
        public double rotPosition;
        private WristState(double tiltPosition, double rotPosition){
            this.tiltPosition = tiltPosition;
            this.rotPosition = rotPosition;
        }
    }
    private SparkClosedLoopController ControllerR;
    private SparkClosedLoopController ControllerL;
    public SparkMax MotorR;
    public SparkMax MotorL;
    public ArmFeedforward feedforward;
    public SparkMaxConfig MotorConfigR;
    public SparkMaxConfig MotorConfigL;
    public static WristState wristState = WristState.TEST;
    // public SparkMaxPIDController RPID;
    // public SparkMaxPIDController LPID;
    public static Wrist instance;
    private double tiltPosition;
    private double rotatePosition;
    private boolean hasZerod;
    private boolean hasTilted;

    public Wrist() {
        feedforward = new ArmFeedforward(0.0, 0.0, 0.0, 0.0);
        tiltPosition = 0;
        rotatePosition = 0;
        hasZerod = false;
        hasTilted = false;

        MotorR = new SparkMax(4, MotorType.kBrushless); //SAY THE CORRECT PORT NUMBER LATER
        MotorL = new SparkMax(3, MotorType.kBrushless); //SAY THE CORRECT PORT NUMBER LATER
        MotorConfigL = new SparkMaxConfig();
        MotorConfigR = new SparkMaxConfig();

        MotorConfigL
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(60);
        MotorConfigL.absoluteEncoder
            .positionConversionFactor(360)
            .inverted(false);
        MotorConfigL.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(0, 0, 0)
            .outputRange(-1, 1);
        MotorL.configure(MotorConfigL, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        MotorConfigR
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(60);
        MotorConfigL.absoluteEncoder
            .positionConversionFactor(360)
            .inverted(true);
        MotorConfigL.closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(0, 0, 0)
            .outputRange(-1, 1);
        MotorR.configure(MotorConfigR, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        ControllerL = MotorL.getClosedLoopController();
        ControllerR = MotorR.getClosedLoopController();
    }
    public void UpdatePose(){
        if(hasZerod)
        {   
            if(hasTilted)
            {
                if(rotateWrist(wristState.rotPosition)){
                    hasZerod = false;
                    hasTilted = false;
                }
            }
            else if(tiltWrist(wristState.tiltPosition))
            {
                hasTilted = true;
                if(rotateWrist(wristState.rotPosition)){
                    hasZerod = false;
                    hasTilted = false;
                }
            }
        }
        else if(rezeroRotation())
        {
            hasZerod = true;
            if(tiltWrist(wristState.tiltPosition))
            {
                hasTilted = true;
                if(rotateWrist(wristState.rotPosition))
                {
                    hasZerod = false;
                    hasTilted = false;
                }
            }
        }

        SmartDashboard.putBoolean("hasZerod", hasZerod);
        SmartDashboard.putBoolean("hasTitled", hasTilted);
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

    public void rotateTheWrist(){
        MotorL.set(0.2);
       MotorR.set(0.2);
    }

    public void oppositeTherotateTheWrist(){
        MotorL.set(-0.2);
        MotorR.set(-0.2);
    }

    public void tiltTheWrist(){
        MotorL.set(-0.2);
        MotorR.set(0.2);
    }

    public void oppositeThetiltTheWrist(){
        MotorL.set(0.2);
       MotorR.set(-0.2);
    }

    public void ZeroTheWrist(){
        MotorL.set(0.0);
        MotorR.set(0.0);
    }

    public boolean tiltWrist(double targetTilt){
        tiltPosition = targetTilt;

        ControllerL.setReference(targetTilt, ControlType.kPosition, ClosedLoopSlot.kSlot1, feedforward.calculate(MotorL.getEncoder().getPosition(), 0));
        ControllerR.setReference(targetTilt, ControlType.kPosition, ClosedLoopSlot.kSlot1, feedforward.calculate(MotorR.getEncoder().getPosition(), 0));
        if(hasReachedPose(targetTilt, 2)){
            return true;
        }
        return false;
    }
    public boolean rotateWrist(double targetRot){
        rotatePosition = targetRot;

        ControllerL.setReference(tiltPosition + targetRot, ControlType.kPosition, ClosedLoopSlot.kSlot1, feedforward.calculate(MotorL.getEncoder().getPosition(), 0));
        ControllerR.setReference(tiltPosition - targetRot, ControlType.kPosition, ClosedLoopSlot.kSlot1, feedforward.calculate(MotorR.getEncoder().getPosition(), 0));
        if(hasReachedPose(tiltPosition + targetRot, 2)){
            return true;
        }
        return false;
    }
    public boolean rezeroRotation(){
        rotatePosition = 0;

        ControllerL.setReference(tiltPosition, ControlType.kPosition, ClosedLoopSlot.kSlot1, feedforward.calculate(MotorL.getEncoder().getPosition(), 0));
        ControllerR.setReference(tiltPosition, ControlType.kPosition, ClosedLoopSlot.kSlot1, feedforward.calculate(MotorR.getEncoder().getPosition(), 0));
        if(hasReachedPose(tiltPosition, 2)){
            return true;
        }
        return false;
    }
    public boolean hasReachedPose(double position, double tolerance) {
        return Math.abs(getPosition() - position) < tolerance;
    }
    public double getPosition()
    {
        return MotorL.getEncoder().getPosition();
    }
    public WristState getWristState(){
        return wristState;
    }
    public void setWriststate(WristState state){
        wristState = state;
    }
    public static Wrist getInstance() {
        if (instance == null)
            instance = new Wrist();
        return instance;
    }
}
