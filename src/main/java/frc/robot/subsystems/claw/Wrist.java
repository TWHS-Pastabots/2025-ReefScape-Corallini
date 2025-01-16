package frc.robot.subsystems.claw;

import org.opencv.features2d.FlannBasedMatcher;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

public class Wrist {

    // public enum WristState
    // {
    // TEST(5,0),
    // TEST2(30,0),
    // TEST3(120,0),
    // TEST4(270,0);

    // public double tiltPosition;
    // public double rotPosition;
    // private WristState(double tiltPosition, double rotPosition){
    // this.tiltPosition = tiltPosition;
    // this.rotPosition = rotPosition;
    // }
    // }
    public enum WristState {
        TEST(new EncoderPosition(5, 0), new EncoderPosition(0, -1)),
        TEST2(new EncoderPosition(30, 0), new EncoderPosition(0, -1)),
        TEST3(new EncoderPosition(60, 0), new EncoderPosition(0, -1)),
        TEST4(new EncoderPosition(120, 0), new EncoderPosition(0, -1));

        public EncoderPosition tiltPosition;
        public EncoderPosition rotPosition;

        private WristState(EncoderPosition tiltPosition, EncoderPosition rotPosition) {
            this.tiltPosition = tiltPosition;
            this.rotPosition = rotPosition;
        }

    }

    public SparkClosedLoopController ControllerR;
    public SparkClosedLoopController ControllerL;
    public SparkMax MotorR;
    public SparkMax MotorL;
    public AbsoluteEncoder encoderR;
    public AbsoluteEncoder encoderL;
    public ArmFeedforward feedforward;
    public SparkMaxConfig MotorConfigR;
    public SparkMaxConfig MotorConfigL;
    public WristState wristState = WristState.TEST2;
    // public SparkMaxPIDController RPID;
    // public SparkMaxPIDController LPID;
    public static Wrist instance;
    private EncoderPosition tiltPosition;
    private EncoderPosition rotatePosition;
    private EncoderPosition currentPositionL;
    private EncoderPosition currentPositionR;
    private double lastEncoderPositionL;
    private double lastEncoderPositionR;
    private boolean hasZerod;
    private boolean hasTilted;
    private boolean hasRotated;

    public Wrist() {
        feedforward = new ArmFeedforward(0.0, 0.0, 0.0, 0.0);
        tiltPosition = new EncoderPosition(0, -1);
        rotatePosition = new EncoderPosition(0, -1);
        hasZerod = false;
        hasTilted = false;
        hasRotated = false;

        MotorR = new SparkMax(4, MotorType.kBrushless); // SAY THE CORRECT PORT NUMBER LATER
        MotorL = new SparkMax(3, MotorType.kBrushless); // SAY THE CORRECT PORT NUMBER LATER

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
                .pid(0.005, 0, 0)
                .outputRange(-1, 1);
        MotorL.configure(MotorConfigL, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        MotorConfigR
                .inverted(false)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(60);
        MotorConfigR.absoluteEncoder
                .positionConversionFactor(360)
                .inverted(true);
        MotorConfigR.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(0.005, 0, 0)
                .outputRange(-1, 1);
        MotorR.configure(MotorConfigR, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        ControllerL = MotorL.getClosedLoopController();
        ControllerR = MotorR.getClosedLoopController();
        lastEncoderPositionL = getPosition(MotorL);
        lastEncoderPositionR = getPosition(MotorR);
    }

    public void UpdatePose() {
        lastEncoderPositionL = getPosition(MotorL);
        lastEncoderPositionR = getPosition(MotorR);
        for (int i = 0; i < 50; i++) {
        }
        UpdateCurrentPosition();
        tiltWrist(wristState.tiltPosition);
        // if(hasRotated)
        // {
        // rotateWrist(wristState.rotPosition);
        // }
        // else
        // {
        // if(hasZerod)
        // {
        // if(hasTilted)
        // {
        // if(rotateWrist(wristState.rotPosition)){
        // hasZerod = false;
        // hasTilted = false;
        // hasRotated = true;
        // }
        // }
        // else if(tiltWrist(wristState.tiltPosition))
        // {
        // hasTilted = true;
        // if(rotateWrist(wristState.rotPosition)){
        // hasZerod = false;
        // hasTilted = false;
        // hasRotated = true;
        // }
        // }
        // }
        // else if(rezeroRotation())
        // {
        // hasZerod = true;
        // if(tiltWrist(wristState.tiltPosition))
        // {
        // hasTilted = true;
        // if(rotateWrist(wristState.rotPosition))
        // {
        // hasZerod = false;
        // hasTilted = false;
        // hasRotated = true;
        // }
        // }
        // }
        // }

        SmartDashboard.putBoolean("hasZerod", hasZerod);
        SmartDashboard.putBoolean("hasTitled", hasTilted);
        SmartDashboard.putBoolean("hasRotated", hasRotated);
    }
    // public void moveWrist(double targetTilt, double targetRotation){
    // tiltPosition = targetTilt;
    // rotatePosition = targetRotation;

    // double motorLCurrent = MotorL.getAbsoluteEncoder().getPosition();
    // double motorRCurrent = MotorR.getAbsoluteEncoder().getPosition();

    // double currentTilt = (motorLCurrent + motorRCurrent) /2;
    // double currentRotation = (motorRCurrent - motorLCurrent)/2;

    // double motorLTarget = motorLCurrent + (targetTilt - currentRotation) -
    // (targetRotation - currentRotation);
    // double motorRTarget = motorRCurrent + (targetTilt - currentRotation) +
    // (targetRotation - currentRotation);

    // ControllerL.setReference(motorLTarget, ControlType.kPosition,
    // ClosedLoopSlot.kSlot0,
    // feedforward.calculate(MotorL.getAbsoluteEncoder().getPosition(), 0));
    // ControllerR.setReference(motorRTarget, ControlType.kPosition,
    // ClosedLoopSlot.kSlot0,
    // feedforward.calculate(MotorR.getAbsoluteEncoder().getPosition(), 0));

    // SmartDashboard.putNumber("Tilt Position (Target)", tiltPosition);
    // SmartDashboard.putNumber("Rotation Position (Target)", rotatePosition);
    // SmartDashboard.putNumber("Tilt Position (Current)", currentTilt);
    // SmartDashboard.putNumber("Rotation Position (Current)", currentRotation);
    // SmartDashboard.putNumber("Motor L Target", motorLTarget);
    // SmartDashboard.putNumber("Motor R Target", motorRTarget);
    // }
    public void UpdateCurrentPosition() {
        double sectionL = -1;
        double sectionR = -1;
        if ((getPosition(MotorL) < 20 && lastEncoderPositionL > 340)
                || (getPosition(MotorL) > 340 && lastEncoderPositionL < 20)) {
            if (currentPositionL.encoderSection == 0) {
                sectionL = 1;
            } else if (currentPositionL.encoderSection == 1) {
                sectionL = 0;
            }
        }
        if ((getPosition(MotorR) < 20 && lastEncoderPositionR > 340)
                || (getPosition(MotorR) > 340 && lastEncoderPositionR < 20)) {
            if (currentPositionR.encoderSection == 0) {
                sectionR = 1;
            } else if (currentPositionR.encoderSection == 1) {
                sectionR = 0;
            }
        }
        currentPositionL = new EncoderPosition(getPosition(MotorL), sectionL);
        currentPositionR = new EncoderPosition(getPosition(MotorR), sectionR);
    }

    public void rotateTheWrist() {
        MotorL.set(0.1);
        MotorR.set(0.1);
    }

    public void oppositeTherotateTheWrist() {
        MotorL.set(-0.1);
        MotorR.set(-0.1);
    }

    public void tiltTheWrist() {
        MotorL.set(-0.1);
        MotorR.set(0.1);
    }

    public void oppositeThetiltTheWrist() {
        MotorL.set(0.1);
        MotorR.set(-0.1);
    }

    public void ZeroTheWrist() {
        MotorL.set(0.0);
        MotorR.set(0.0);
    }

    public boolean tiltWrist(EncoderPosition targetTilt) {
        tiltPosition = targetTilt;
        double[] anglesL = WrapAngle(currentPositionL, targetTilt);
        double[] anglesR = WrapAngle(currentPositionR, targetTilt);
        if (!hasReachedPose(MotorL, anglesL[0], 0.5)) {
            ControllerL.setReference(anglesL[0], ControlType.kPosition, ClosedLoopSlot.kSlot0,
                    feedforward.calculate(MotorL.getAbsoluteEncoder().getPosition(), 0));
        } else if (hasReachedPose(MotorL, anglesL[0], 0.5) && anglesL[1] != -1) {
            ControllerL.setReference(anglesL[1], ControlType.kPosition, ClosedLoopSlot.kSlot0,
                    feedforward.calculate(MotorL.getAbsoluteEncoder().getPosition(), 0));
        }

        if (!hasReachedPose(MotorR, anglesL[0], 0.5)) {
            ControllerR.setReference(anglesR[0], ControlType.kPosition, ClosedLoopSlot.kSlot0,
                    feedforward.calculate(MotorL.getAbsoluteEncoder().getPosition(), 0));
        } else if (hasReachedPose(MotorL, anglesL[0], 0.5) && anglesR[1] != -1) {
            ControllerR.setReference(anglesR[1], ControlType.kPosition, ClosedLoopSlot.kSlot0,
                    feedforward.calculate(MotorL.getAbsoluteEncoder().getPosition(), 0));
        }

        // ControllerR.setReference(targetTilt, ControlType.kPosition,
        // ClosedLoopSlot.kSlot0,
        // feedforward.calculate(MotorR.getAbsoluteEncoder().getPosition(), 0));
        if (anglesL[1] != -1 && hasReachedPose(MotorL, anglesL[1], .5)) {
            return true;
        } else if (hasReachedPose(MotorL, anglesL[0], .5) && anglesL[1] == -1) {
            return true;
        }
        return false;
    }

    public boolean rotateWrist(EncoderPosition targetRot) {
        rotatePosition = targetRot;

        ControllerL.setReference(tiltPosition.encoderAngle + targetRot.encoderAngle, ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                feedforward.calculate(MotorL.getAbsoluteEncoder().getPosition(), 0));

        ControllerR.setReference(tiltPosition.encoderAngle - targetRot.encoderAngle, ControlType.kPosition,
                ClosedLoopSlot.kSlot0,
                feedforward.calculate(MotorR.getAbsoluteEncoder().getPosition(), 0));
        if (hasReachedPose(MotorL, tiltPosition.encoderAngle + targetRot.encoderAngle, 2)) {
            return true;
        }
        return false;
    }

    public boolean rezeroRotation() {
        rotatePosition = new EncoderPosition(0, -1);

        ControllerL.setReference(tiltPosition.encoderAngle, ControlType.kPosition, ClosedLoopSlot.kSlot0,
                feedforward.calculate(MotorL.getAbsoluteEncoder().getPosition(), 0));

        ControllerR.setReference(tiltPosition.encoderAngle, ControlType.kPosition, ClosedLoopSlot.kSlot0,
                feedforward.calculate(MotorR.getAbsoluteEncoder().getPosition(), 0));
        if (hasReachedPose(MotorL, tiltPosition.encoderAngle, 2)) {
            return true;
        }
        return false;
    }

    public boolean hasReachedPose(SparkMax motor, double position, double tolerance) {
        return Math.abs(getPosition(motor) - position) < tolerance;
    }

    public double getPosition(SparkMax motor) {
        return motor.getAbsoluteEncoder().getPosition();
    }

    public WristState getWristState() {
        return wristState;
    }

    public void setWriststate(WristState state) {
        wristState = state;
        hasRotated = false;
    }

    public double[] WrapAngle(EncoderPosition currentAngle, EncoderPosition targetAngle)
    {
        double[] angles = new double[2];
        angles[0] = -1; 
        angles[1] = -1;
        if(currentAngle.encoderAngle < 0){
            currentAngle.encoderAngle = currentAngle.encoderAngle + 360;
            if(currentAngle.encoderSection == 0){
                currentAngle.encoderSection = 1;
            }
            else if(currentAngle.encoderSection == 1){
                currentAngle.encoderSection = 0;
            }
        }
        if(targetAngle.encoderAngle < 0){
            targetAngle.encoderAngle = targetAngle.encoderAngle + 360;
            if(targetAngle.encoderSection == 0){
                targetAngle.encoderSection = 1;
            }
            else if(currentAngle.encoderSection == 1){
                currentAngle.encoderSection = 0;
            }
        }
        if(targetAngle.encoderSection != -1 && currentAngle.encoderSection != -1)
        {
            if(targetAngle.encoderSection == currentAngle.encoderSection){
                angles[0] = targetAngle.encoderAngle;
            }
            else{
                if(targetAngle.encoderAngle < currentAngle.encoderAngle)
                {
                    angles[0] = 360;
                    angles[1] = targetAngle.encoderAngle;
                }
                else if(targetAngle.encoderAngle > currentAngle.encoderAngle){
                    angles[0] = 0;
                    angles[1] = targetAngle.encoderAngle;
                }
            }
            // if(Math.abs(targetAngle.encoderAngle - currentAngle) > 180)
            // {
            
            //     if(targetAngle.encoderAngle < currentAngle)
            //     {
            //         angles[0] = 360;
            //         angles[1] = targetAngle.encoderAngle;
            //     }
            //     else if(targetAngle.encoderAngle > currentAngle){
            //         angles[0] = 0;
            //         angles[1] = targetAngle.encoderAngle;
            //     }
            // }
            // else{
            //     angles[0] = targetAngle.encoderAngle;
            // }
        }
        
        return angles;
    }

    public static Wrist getInstance() {
        if (instance == null)
            instance = new Wrist();
        return instance;
    }

    public static class EncoderPosition {
        public double encoderAngle;
        public double encoderSection;

        public EncoderPosition(double angle, double section) {
            encoderAngle = angle;
            encoderSection = section;
        }
    }
}
