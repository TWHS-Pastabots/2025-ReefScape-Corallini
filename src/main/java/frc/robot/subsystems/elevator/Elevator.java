package frc.robot.subsystems.elevator;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ElevatorConstants;

import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

public class Elevator {
    public static Elevator instance;
    public SparkMax elevatorMotorL;
    public SparkMaxConfig configL;
    public SparkMax elevatorMotorR;
    public SparkMaxConfig configR;

    public SparkClosedLoopController ControllerR;
    public SparkClosedLoopController ControllerL;

    public static ElevatorFeedforward feedForward;
    public static ElevatorState elevatorState = ElevatorState.BOT;
    public enum ElevatorState{
        BOT(0),
        MID(25),
        TOP(50);

        public double position;
        private ElevatorState(double position){
            this.position = position;
        }
    }
    public Elevator() {
        elevatorMotorL = new SparkMax(1, MotorType.kBrushless); //LABEL THE PORT LATER
        elevatorMotorR = new SparkMax(2, MotorType.kBrushless); //LABEL THE PORT LATER
        configL = new SparkMaxConfig();
        configR = new SparkMaxConfig();

        configL
            .inverted(true)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(60);
        configL.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(ElevatorConstants.elevatorPCoefficient, ElevatorConstants.elevatorICoefficient, ElevatorConstants.elevatorDCoefficient)
            .outputRange(-.5, .5);
         elevatorMotorL.configure(configL, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

         configR
            .inverted(false)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(60);
        configR.closedLoop

        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
           .pid(ElevatorConstants.elevatorPCoefficient, ElevatorConstants.elevatorICoefficient, ElevatorConstants.elevatorDCoefficient)
            .outputRange(-.5, .5);
         elevatorMotorR.configure(configR, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        feedForward = new ElevatorFeedforward(0,0,0,0); //FILL THESE NUMBERS IN LATER

         ControllerL = elevatorMotorL.getClosedLoopController();
         ControllerR = elevatorMotorR.getClosedLoopController();

         
    }
    public void updatePose(){
        ControllerL.setReference(elevatorState.position, ControlType.kPosition, ClosedLoopSlot.kSlot0, //DOUBLE CHECK THE CLOSEDLOOP
        feedForward.calculate(elevatorMotorL.getEncoder().getPosition(), 0));//SLOTS ONCE WE KNOW 

        ControllerR.setReference(elevatorState.position, ControlType.kPosition, ClosedLoopSlot.kSlot0,//SAME THING
        feedForward.calculate(elevatorMotorR.getEncoder().getPosition(), 0));
    }
    public void setElevatorPosition(double position){
        elevatorState.position = position;
    }
    public void setElevatorState(ElevatorState state){
        elevatorState = state;
    }
    public ElevatorState getElevatorState(){
        return elevatorState;
    }
    public static Elevator getInstance() {
        if (instance == null)
            instance = new Elevator();
        return instance;
    }
    
}

