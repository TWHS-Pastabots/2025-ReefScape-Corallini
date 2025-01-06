package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import frc.robot.Ports;
import frc.robot.Constants.ElevatorConstants;

public class Elevator {
    public static Elevator instance;
    private CANSparkMax elevatorMotorL;
    private CANSparkMax elevatorMotorR;
    private static RelativeEncoder encoderL;
    private static RelativeEncoder encoderR;
    private SparkMaxPIDController elevatorControllerL;
    private SparkMaxPIDController elevatorControllerR;
    private static ElevatorFeedforward feedForward;
    public static ElevatorState elevatorState = ElevatorState.BOT;
    private boolean[] connections = new boolean[2];
    

    public enum ElevatorState{
        //THESE ARE JUST GUESSES CHANGE THEM LATER
        BOT(0),
        MID(25),
        TOP(50);

        public double position;
        private ElevatorState(double position){
            this.position = position;
        }
    }
    public Elevator(){
        elevatorMotorL = new CANSparkMax(Ports.elevatorMotorL, MotorType.kBrushless);
        elevatorMotorL.restoreFactoryDefaults();
        elevatorMotorL.setSmartCurrentLimit(60);
        elevatorMotorL.setIdleMode(IdleMode.kBrake);
        elevatorMotorL.setInverted(true);
        elevatorMotorL.burnFlash();


        elevatorMotorR = new CANSparkMax(Ports.elevatorMotorR, MotorType.kBrushless);
        elevatorMotorR.restoreFactoryDefaults();
        elevatorMotorR.setSmartCurrentLimit(60);
        elevatorMotorR.setIdleMode(IdleMode.kBrake);
        elevatorMotorR.setInverted(false);
        elevatorMotorR.burnFlash();
        //JUST A GUESS TUNE LATER
        feedForward = new ElevatorFeedforward(0.012, 0.017, 0.0);
        encoderL = elevatorMotorL.getEncoder();
        elevatorControllerL = elevatorMotorL.getPIDController();
        
        elevatorControllerL.setP(ElevatorConstants.elevatorPCoefficient);
        elevatorControllerL.setI(ElevatorConstants.elevatorICoefficient);
        elevatorControllerL.setD(ElevatorConstants.elevatorDCoefficient);
        elevatorControllerL.setFeedbackDevice(encoderL);
        elevatorControllerL.setOutputRange(-1, 1);

        encoderR = elevatorMotorR.getEncoder();
        elevatorControllerR = elevatorMotorR.getPIDController();
        
        elevatorControllerR.setP(ElevatorConstants.elevatorPCoefficient);
        elevatorControllerR.setI(ElevatorConstants.elevatorICoefficient);
        elevatorControllerR.setD(ElevatorConstants.elevatorDCoefficient);
        elevatorControllerR.setFeedbackDevice(encoderR);
        elevatorControllerR.setOutputRange(-1, 1);
        elevatorMotorL.burnFlash();
        elevatorMotorR.burnFlash();
    }

    public void updatePose(){
        elevatorControllerL.setReference(elevatorState.position, CANSparkMax.ControlType.kPosition, 0,
        feedForward.calculate(encoderL.getPosition(), 0));

        elevatorControllerR.setReference(elevatorState.position, CANSparkMax.ControlType.kPosition, 0,
        feedForward.calculate(encoderR.getPosition(), 0));
    }
    public void elevatorOn(){
        elevatorMotorL.set(.75);
        elevatorMotorR.set(.75);
    }
    public void elevatorReverse(){
        elevatorMotorL.set(-.75);
        elevatorMotorR.set(-.75);
    }
    public void elevatorOff(){
        elevatorMotorL.set(0);
        elevatorMotorR.set(0);
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
    public double getPosition(RelativeEncoder encoder) {
        return encoder.getPosition();
    }

    public boolean [] elevatorConnections(){
        if(elevatorMotorL.getBusVoltage() != 0){
            connections[0] = true;
        }else{
            connections[0] = false;
        }
        if(elevatorMotorR.getBusVoltage() != 0){
            connections[1] = true;
        }else{
            connections[1] = false;
        }
        return connections;
    }
    public static Elevator getInstance() {
        if (instance == null)
            instance = new Elevator();
        return instance;
    }
}
