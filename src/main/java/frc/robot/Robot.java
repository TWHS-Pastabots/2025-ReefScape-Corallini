package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.IO.LED;
import frc.robot.subsystems.claw.Wrist;
import frc.robot.subsystems.claw.Wrist.WristState;
import frc.robot.subsystems.swerve.Drivebase;
import frc.robot.subsystems.swerve.Drivebase.DriveState;
import frc.robot.subsystems.vision.CameraSystem;

public class Robot extends LoggedRobot {
  //all the initialing for the systems
  private static Field2d field = new Field2d();
  // private Drivebase drivebase;
  private Wrist wrist;
  // private LED litty;
  // private CameraSystem camSystem;
  
  public RobotConfig Rconfig;
  private static XboxController driver;
  private static XboxController operator;
  //initialization of the auton chooser in the dashboard
  private Command m_autoSelected;



  Double targetRange = null;
  Double targetAngle = null;

  

  //that is a chooser for the autons utilizing the sendableChooser which allows us to choose the auton commands
  private SendableChooser<Command> m_chooser = new SendableChooser<>();

  @Override
  public void robotInit() {
    // drivebase = Drivebase.getInstance();
    // litty = LED.getInstance();
    wrist = Wrist.getInstance();

    // camSystem = CameraSystem.getInstance();
    // camSystem.AddCamera(new PhotonCamera("Cam1"), new Transform3d(
    //     new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0.0, 0.0, 0.0))
    //     ,  true);

    // camSystem.AddCamera(new PhotonCamera("Cam2"),  new Transform3d(
    //   new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0.0, 0.0, 0.0)),
    //    true);


    driver = new XboxController(0);
    operator = new XboxController(1);
   
  
    SmartDashboard.putData("Auto choices", m_chooser);

  }

  @Override
  public void robotPeriodic() {
      SmartDashboard.putNumber("Left abs encoder", wrist.MotorL.getAbsoluteEncoder().getPosition());
      SmartDashboard.putNumber("Right abs encoder", wrist.MotorR.getAbsoluteEncoder().getPosition());

      SmartDashboard.putString("Wrist State", wrist.wristState.toString());

      SmartDashboard.putData("Orig Field", field);
      field.setRobotPose(new Pose2d(new Translation2d(10,5), new Rotation2d()));
      // Pose2d cameraPositionTele = camSystem.calculateRobotPosition();

      //  Pose2d posTele = drivebase.updateOdometry(cameraPositionTele);


        // SmartDashboard.putNumber("Odometry X", posTele.getX());
        // SmartDashboard.putNumber("Odometry Y", posTele.getY());

      //this is getting the data from the cameras through the cameraSystem class 
    //  if (camSystem.getCamera(0).isConnected()) {
    //         PhotonPipelineResult backResult = camSystem.getResult(0);      
    //         if (backResult.hasTargets()) {
    //             PhotonTrackedTarget target = backResult.getBestTarget();
    //             Transform3d bestCameraToTarget = target.getBestCameraToTarget();
    //             double distance  = bestCameraToTarget.getTranslation().getNorm();
    //             SmartDashboard.putString("Back Camera Target", "Yes Targets");
    //             SmartDashboard.putNumber("Back to Target", distance);
    //             SmartDashboard.putNumber("Back Camera Target Yaw", target.getYaw());
    //             SmartDashboard.putNumber("Back Camera Target Pitch", target.getPitch());
    //             SmartDashboard.putNumber("Back Camera Target Area", target.getArea());
    //             SmartDashboard.putNumber("ID", target.getFiducialId());

    //         } else if(backResult.hasTargets() == false) {
    //             SmartDashboard.putString("Back Camera Target", "No Targets");
    //         }
    //     } 
      //testing the valuies that the camera gives us and outputing it into the dashboard
      // Pose2d cameraPosition = camSystem.calculateRobotPosition();
      // SmartDashboard.putNumber("Camera X Position", cameraPosition.getX());
      // SmartDashboard.putNumber("Camera Y Position", cameraPosition.getY());
      // SmartDashboard.putNumber("Camera Heading", cameraPosition.getRotation().getDegrees());
    
    CommandScheduler.getInstance().run();
    // drivebase.periodic();
      
    //putting all of the info from the subsystems into the dashvoard so we can test things
    // SmartDashboard.putNumber("Gyro Angle:", (drivebase.getHeading() + 90) % 360);
    // SmartDashboard.putNumber("X-coordinate", drivebase.getPose().getX());
    // SmartDashboard.putNumber("Y-coordinate", drivebase.getPose().getY());

 
  }

  @Override
  public void autonomousInit() {
    //getting the value we chose from the dashboard and putting it into motion in the auton
    m_autoSelected = m_chooser.getSelected();
    
    //drivebase.resetOdometry(PathPlannerAuto.getStartingPoseFromAutoFile(m_chooser.getSelected().getName()));
    
    
//schedules the command so it actually begins moving
    if (m_autoSelected != null) {
      m_autoSelected.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    //updating the intake for the autointake command
    //using cameras to calculate the robot position instead of odometry.
    //we use a mix of odometry + camera positions to calculate the robot position
    //Pose2d cameraPosition = camSystem.calculateRobotPosition(); 
    // Pose2d pose = drivebase.updateOdometry(cameraPosition);

    // SmartDashboard.putNumber("Auto X", drivebase.getPose().getX());
    // SmartDashboard.putNumber("Auto Y", drivebase.getPose().getY());
    // SmartDashboard.putNumber("Odometry X", pose.getX());
    // SmartDashboard.putNumber("Odometry Y", pose.getY());
  }

  @Override
  public void teleopInit() {
    //as soon as we begin teleop we desable the auton selection
    // litty.setBlue();
    if (m_autoSelected != null) {
      m_autoSelected.cancel();
    }

    Translation2d testxy = new Translation2d(16.57 - 14.7, 5.54);
    Rotation2d testRot = new Rotation2d(0);
    Pose2d test = new Pose2d(testxy, testRot);
    // drivebase.resetOdometry(test);
  }

  @Override
  public void teleopPeriodic() {
    
    // wrist.moveWrist(operator.getLeftX() * 360, operator.getRightY() * 360);
    /* DRIVE CONTROLS */
    if(operator.getXButton()){
      wrist.setWriststate(WristState.TEST4);
      wrist.UpdatePose();
    }else if(operator.getYButton()){
      wrist.setWriststate(WristState.TEST3);
      wrist.UpdatePose();
    }else if(operator.getAButton()){
      wrist.setWriststate(WristState.TEST2);
      wrist.UpdatePose();
    }else if(operator.getBButton()){
      wrist.setWriststate(WristState.TEST);
      wrist.UpdatePose();
    }

    // if(driver.getXButton()){
    //   wrist.oppositeTherotateTheWrist();
    // }else if(driver.getYButton()){
    //   wrist.oppositeThetiltTheWrist();
    // }else if(driver.getAButton()){
    //   wrist.tiltTheWrist();
    // }else if(driver.getBButton()){
    //   wrist.rotateTheWrist();
    // }else{
    //   wrist.ZeroTheWrist();
    // }



   


    
    //setting inputs for driving through the driver controller
    // double ySpeed = drivebase.inputDeadband(-driver.getLeftX());
    // double xSpeed = drivebase.inputDeadband(driver.getLeftY());
    // double rot = drivebase.inputDeadband(-driver.getRightX());
    //using buttons to rotate the robot by increments of 90 degrees
    // if (driver.getAButton()) {
    //   drivebase.currHeading = -1;
    //   drivebase.rotateTo(xSpeed, ySpeed, 180);
    // } else if (driver.getBButton()) {
    //   drivebase.currHeading = -1;
    //   drivebase.rotateTo(xSpeed, ySpeed, 270);
    // } else if (driver.getYButton()) {
    //   drivebase.currHeading = -1;
    //   drivebase.rotateTo(xSpeed, ySpeed, 0);
    // } else if (driver.getXButton()) {
    //   drivebase.currHeading = -1;
    //   drivebase.rotateTo(xSpeed, ySpeed, 90);
    // } else if (driver.getLeftTriggerAxis() > 0) {
    // } else {
    //   drivebase.currHeading = -1;
    //   drivebase.drive(xSpeed, ySpeed, rot);
    // }


  
    // if (driver.getPOV() == 0) {
    //   drivebase.zeroHeading();
    // }
    
    

    // if (driver.getRightTriggerAxis() > 0) {
    //   drivebase.setDriveState(DriveState.SLOW);
    // } 
    //getting yaw from the tag to rotate towards it. The robot will allign itself with the 
    // if(driver.getLeftTriggerAxis() > 0)
    // {
    //   Double yaw = camSystem.getYawForTag(1, 4);
    //   targetRange = camSystem.getTargetRange(1, 4);
    //   if(yaw !=null)
    //   {
    //     rot =  -yaw * .002 * Constants.DriveConstants.kMaxAngularSpeed;
    //   }
    //   // if(targetRange != null){
    //   //   xSpeed = (targetRange - 2.5) * .1 * Constants.DriveConstants.kMaxSpeedMetersPerSecond;
    //   // }
      
      
    // }
    
    // if(targetRange != null)
    // {
    //   SmartDashboard.putNumber("TargetRange", targetRange);
    // }
    // if(targetAngle != null)
    // {
    //   SmartDashboard.putNumber("Target Angle", targetAngle);
    // }
    // if(camSystem.getResult(1).hasTargets() && camSystem.getResult(1).getBestTarget() != null){
    //   SmartDashboard.putNumber("TargetPitch", Units.degreesToRadians(camSystem.getResult(1).getBestTarget().getPitch()));
    // }
    // drivebase.drive(xSpeed, ySpeed, rot);
   
    
    
  }
    


  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void simulationInit() {
  }

  @Override
  public void simulationPeriodic() {
  }

}