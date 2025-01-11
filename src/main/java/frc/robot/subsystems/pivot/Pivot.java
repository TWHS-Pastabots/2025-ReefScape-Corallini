package frc.robot.subsystems.pivot;


import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.PivotConstants;

public class Pivot {
    private SparkMax pivotMotor;
    private SparkMaxConfig config;
    private ArmFeedforward feedForward;
    private SparkClosedLoopController pivotController;
    private static PivotState pivotState = PivotState.TEST;

    private static Pivot instance;

    public enum PivotState {
        TEST(0,0);

        public double position;
        public double launchSpeed;
        
        private PivotState(double position, double launchSpeed) {
            this.position = position;
            this.launchSpeed = launchSpeed;
        }
    }

    public Pivot() {
        pivotMotor = new SparkMax(0, MotorType.kBrushless);
        config = new SparkMaxConfig();
        config
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(60)
            .openLoopRampRate(1);
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(PivotConstants.pivotPCoefficient, PivotConstants.pivotICoefficient, PivotConstants.pivotDCoefficient)
            .outputRange(-1, 1);
        pivotMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        pivotController = pivotMotor.getClosedLoopController();
        feedForward = new ArmFeedforward(0, 0, 0, 0);

        pivotMotor.getEncoder().setPosition(0);
    }
    public void updatePose() {

        pivotController.setReference(pivotState.position, ControlType.kPosition, ClosedLoopSlot.kSlot0,
                feedForward.calculate(pivotMotor.getEncoder().getPosition(), 0));
    }
    public void setPivotPosition(double position)
    {
        pivotState.position = position;
    }
    public double getPosition() {
        return pivotMotor.getEncoder().getPosition();
    }
    public void setLauncherState(PivotState state) {
        pivotState = state;
    }
    public boolean hasReachedPose(double tolerance) {
        return Math.abs(getPosition() - pivotState.position) < tolerance;
    }
    public static Pivot getInstance() {
        if (instance == null)
            instance = new Pivot();
        return instance;
    }
}
