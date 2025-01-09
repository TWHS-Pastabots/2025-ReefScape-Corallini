// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;


import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
// import com.revrobotics.CANSparkMaxLowLevel.MotorType;
// import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
// import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
        private final SparkMax drivingSparkMax;
        private final SparkMax steerSparkMax;
        private final SparkMaxConfig driveConfig;
        private final SparkMaxConfig steerConfig;

        // private final RelativeEncoder drivingEncoder;
        // private final AbsoluteEncoder turningEncoder;

        private final SparkClosedLoopController veloPIDController;
        private final SparkClosedLoopController anglePIDController;

        private double chassisAngularOffset = 0;
        private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());

        public SwerveModule(int drivingCANId, int turningCANId, double newChassisAngularOffset, boolean invert) {
                drivingSparkMax = new SparkMax(drivingCANId, MotorType.kBrushless);
                driveConfig = new SparkMaxConfig();
                steerSparkMax = new SparkMax(turningCANId, MotorType.kBrushless);
                steerConfig = new SparkMaxConfig();
                driveConfig
                        .inverted(false)
                        .idleMode(ModuleConstants.kDrivingMotorIdleMode)
                        .smartCurrentLimit(ModuleConstants.kDrivingMotorCurrentLimit);
                driveConfig.encoder
                        .positionConversionFactor(ModuleConstants.kDrivingEncoderPositionFactor)
                        .velocityConversionFactor(ModuleConstants.kDrivingEncoderVelocityFactor);
                driveConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .pid(ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD)
                        .velocityFF(ModuleConstants.kDrivingFF)
                        .outputRange(ModuleConstants.kDrivingMinOutput, ModuleConstants.kDrivingMaxOutput);
                drivingSparkMax.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);



                steerConfig
                        .inverted(true)
                        .idleMode(ModuleConstants.kTurningMotorIdleMode)
                        .smartCurrentLimit(ModuleConstants.kTurningMotorCurrentLimit);
                steerConfig.encoder
                        .positionConversionFactor(ModuleConstants.kTurningEncoderPositionFactor)
                        .velocityConversionFactor(ModuleConstants.kTurningEncoderVelocityFactor);
                steerConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                        .pid(ModuleConstants.kTurningP, ModuleConstants.kTurningI, ModuleConstants.kTurningD)
                        .velocityFF(ModuleConstants.kTurningFF)
                        .outputRange(ModuleConstants.kTurningMinOutput, ModuleConstants.kTurningMaxOutput)
                        .iZone(16)
                        .positionWrappingEnabled(true)
                        .positionWrappingMinInput(ModuleConstants.kTurningEncoderPositionPIDMinInput)
                        .positionWrappingMaxInput(ModuleConstants.kTurningEncoderPositionPIDMaxInput);
                        
                steerSparkMax.configure(steerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                
                // Factory reset, so we get the SPARKS MAX to a known state before configuring
                // them. This is useful in case a SPARK MAX is swapped out.
                // Setup encoders and PID controllers for the driving and turning SPARKS MAX.
                
                veloPIDController = drivingSparkMax.getClosedLoopController();
                anglePIDController = steerSparkMax.getClosedLoopController();
                
               

                // Apply position and velocity conversion factors for the driving encoder. The
                // native units for position and velocity are rotations and RPM, respectively,
                // but we want meters and meters per second to use with WPILib's swerve APIs.
        

                // Apply position and velocity conversion factors for the turning encoder. We
                // want these in radians and radians per second to use with WPILib's swerve
                // APIs.

                // Invert the turning encoder, since the output shaft rotates in the opposite
                // direction of
                // the steering motor in the MAXSwerve Module.
                

                // Enable PID wrap around for the turning motor. This will allow the PID
                // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                // to 10 degrees will go through 0 rather than the other direction which is a
                // longer route.
                
                // Set the PID gains for the driving motor. Note these are example gains, and
                // you
                // may need to tune them for your own robot!              
                // Set the PID gains for the turning motor. Note these are example gains, and
                // you
                // may need to tune them for your own robot!
               

                // Save the SPARK MAX configurations. If a SPARK MAX browns out during
                // operation, it will maintain the above configurations

                chassisAngularOffset = newChassisAngularOffset;
                desiredState.angle = new Rotation2d(steerSparkMax.getAbsoluteEncoder().getPosition());
                drivingSparkMax.getEncoder().setPosition(0);
        }

        /**
         * Returns the current state of the module.
         *
         * @return The current state of the module.
         */
        public SwerveModuleState getState() {
                // Apply chassis angular offset to the encoder position to get the position
                // relative to the chassis.
                return new SwerveModuleState(drivingSparkMax.getEncoder().getVelocity(),
                                new Rotation2d(steerSparkMax.getAbsoluteEncoder().getPosition() - chassisAngularOffset));
        }

        public double getModuleAngle(){
                return (new Rotation2d(steerSparkMax.getAbsoluteEncoder().getPosition() - chassisAngularOffset)).getDegrees();
        }

        /**
         * Returns the current position of the module.
         *
         * @return The current position of the module.
         */
        public SwerveModulePosition getPosition() {
                // Apply chassis angular offset to the encoder position to get the position
                // relative to the chassis.
                return new SwerveModulePosition(
                                drivingSparkMax.getEncoder().getPosition(),
                                new Rotation2d(steerSparkMax.getAbsoluteEncoder().getPosition() - chassisAngularOffset));
        }

        public double getTranslationalVelocity(){
                return drivingSparkMax.getEncoder().getVelocity();
        }

        public double get550Current(){
                return steerSparkMax.getOutputCurrent();
        }

        public double getNEOCurrent(){
                return drivingSparkMax.getOutputCurrent();
        }

        /**
         * Sets the desired state for the module.
         *
         * @param desiredState Desired state with speed and angle.
         */
        public void setDesiredState(SwerveModuleState desiredState) {
                // Apply chassis angular offset to the desired state.
                SwerveModuleState correctedDesiredState = new SwerveModuleState();
                correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
                correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

                correctedDesiredState.optimize(new Rotation2d(steerSparkMax.getAbsoluteEncoder().getPosition()));
                // Command driving and turning SPARKS MAX towards their respective setpoints.
                // veloPIDController.setReference(correctedDesiredState.speedMetersPerSecond,
                //                 SparkMax.ControlType.kVelocity);
                
                anglePIDController.setReference(correctedDesiredState.angle.getRadians(),
                                SparkMax.ControlType.kPosition);
                // Optimize the reference state to avoid spinning further than 90 degrees.
                // SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
                //                 new Rotation2d(steerSparkMax.getAbsoluteEncoder().getPosition()));

                double desiredSpeed = correctedDesiredState.speedMetersPerSecond/DriveConstants.kMaxSpeedMetersPerSecond;

                // // Command driving and turning SPARKS MAX towards their respective setpoints.
                drivingSparkMax.set(desiredSpeed);

                // anglePIDController.setReference(optimizedDesiredState.angle.getRadians(),
                //                 SparkMax.ControlType.kPosition);

                // // desiredState = desiredState;
        }

            public void setAutoSpeeds(SwerveModuleState desiredState) {
                // Apply chassis angular offset to the desired state.
                SwerveModuleState correctedDesiredState = new SwerveModuleState();
                correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
                correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));

                // Optimize the reference state to avoid spinning further than 90 degrees.
                // SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
                //                 new Rotation2d(steerSparkMax.getAbsoluteEncoder().getPosition()));
                
                correctedDesiredState.optimize(new Rotation2d(steerSparkMax.getAbsoluteEncoder().getPosition()));
                // Command driving and turning SPARKS MAX towards their respective setpoints.
                veloPIDController.setReference(correctedDesiredState.speedMetersPerSecond,
                                SparkMax.ControlType.kVelocity);
                
                anglePIDController.setReference(correctedDesiredState.angle.getRadians(),
                                SparkMax.ControlType.kPosition);

                // desiredState = desiredState;
        }

        /** Zeroes all the SwerveModule encoders. */
        public void resetEncoders() {
                drivingSparkMax.getEncoder().setPosition(0);
        }
}