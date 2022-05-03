/*
* @zaanthomas
*/

/*imports*/

/* prerequisites
WPILIB: Manage Vendor Libraries
Install New Libraries (online)
https://maven.ctr-electronics.com/release/com/ctre/phoenix/Phoenix-frc2022-latest.json
https://software-metadata.revrobotics.com/REVLib.json
https://www.kauailabs.com/dist/frc/2022/navx_frc.json
*/

//default
package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.util.*;


//motors
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.interfaces.Gyro;


//controlers
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;


//sensors
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

//encoders
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;

//solenoid
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;


//auto
import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.math.trajectory.Trajectory.State;


//camera
import edu.wpi.first.cameraserver.CameraServer;

//gyro
import com.kauailabs.navx.*;
import com.kauailabs.navx.AHRSProtocol;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SPI;

//other
import java.io.IOException;
import java.lang.Math;
import java.nio.file.Path;
import java.util.ResourceBundle.Control;
import java.util.concurrent.TimeUnit;




/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  //Controlers
  private Joystick joystick;
  private Joystick xbox;

///////////////////////////////
  //Motors

  /*drive motors*/
  //private WPI_TalonFX[] leftDrive = new WPI_TalonFX[2];
  private WPI_TalonFX[] rightDrive = new WPI_TalonFX[2];
  private WPI_TalonFX leftDriveBack;
  private WPI_TalonFX leftDriveFront;

  /*shooter*/
  private WPI_TalonFX shooterLeft;
  private WPI_TalonFX shooterRight;
  private WPI_TalonSRX turret;
  private MotorControllerGroup shooter;
  private boolean shooterOn = false;
  private boolean aimOn = false;
  private InterpolatingTreeMap<InterpolatingDouble,InterpolatingDouble> myTreeMap = new InterpolatingTreeMap<InterpolatingDouble,InterpolatingDouble>(5);

  /*elevator*/
  private WPI_TalonFX climberLeft;
  private WPI_TalonFX climberRight;
  private WPI_TalonSRX climberPivotLeftFront;
  private WPI_TalonSRX climberPivotRightFront;
  private WPI_TalonSRX climberPivotLeftBack;
  private WPI_TalonSRX climberPivotRightBack;
  private MotorControllerGroup climberPivotAll;
  private MotorControllerGroup climber;

  /*hopper*/
  private WPI_VictorSPX hopperStage1;
  private WPI_VictorSPX hopperStage2;

  /*intake*/
  private CANSparkMax intake;

///////////////////////////////
  //Motor Groups

  /*drive*/
  private MotorControllerGroup lsideDrive;
  //private MotorControllerGroup rsideDrive;

  /*elevator*/
  private MotorControllerGroup elevator;

  /*turret*/
  private double turretKp = -0.1;
  

///////////////////////////////
  //Drivetrain
  private DifferentialDrive chassis;

///////////////////////////////
  //Solinoid
  //private DoubleSolenoid intakePivot;
///////////////////////////////
  //Motor IDs
  private final int PCM = 16;
  private final int RIGHT_SHOOTER = 15;
  private final int LEFT_SHOOTER = 14;
  private final int RIGHT_CLIMBER = 13;
  private final int LEFT_CLIMBER = 12;
  private final int ANGLE_CLIMBER_RIGHT = 2;
  private final int ANGLE_CLIMBER_LEFT = 3;
  private final int TURRET = 11;
  private final int INTAKE_STAGE2 = 10;
  private final int INTAKE_STAGE1 = 6;
  private final int LEFT_DRIVE_BACK = 9;
  private final int LEFT_DRIVE_FRONT = 7;
  private final int INTAKE_NEO = 8;
  private final int RIGHT_DRIVE_FRONT = 5;
  private final int RIGHT_DRIVE_BACK = 4;

///////////////////////////////
  //Joystick axis

  /*joystick*/
  private final int FOREWARD_BACKWARD_AXIS = 1;
  private final int LEFT_RIGHT_AXIS = 2;
  private final int DRIVING_SPEED = 3;
  private final int TRIGGER = 1;
  private final int THUMB_BUTTON = 2;
  private final int CAMERA_TOGGLE = 3;
  
  /*XBOX*/
  private final int TURRET_AIM = 4;

  private final int CLIMBER_ANGLE_AXIS = 1;

  private final int A = 1;
  private final int B = 2;
  private final int X = 3;
  private final int Y = 4;

  private final int LEFT_BUMPER = 5;
  private final int RIGHT_BUMPER = 6;

  private final int LEFT_TRIGGER = 2;
  private final int RIGHT_TRIGGER = 3;

///////////////////////////////
  //Input Devices

  private NetworkTable limelight;
  private int cameraMode = 0;

///////////////////////////////
  //Speeds
  private final double BASE_INTAKE_SPEED = 1;
  private final double INTAKE_SPEED = 0.35;
  private final double HOPPER_SPEED = 0.75; 
  private double targetRPM = 3500;

  private boolean pivot = true;


///////////////////////////////
  //Autonomus
  private String trajectoryJason;
  private AHRS gyro = new AHRS(SPI.Port.kMXP);
  private  DifferentialDriveOdometry m_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(gyro.getAngle()));
  private Field2d myField = new Field2d(); 
  private double leftMeters = 0.0;
  private double rightMeters = 0.0;
  private double driveKs = 0.77564; //characterization
  private double driveKv = 0.085559; //characterization
  private double driveKa = 0.034434; //characterization
  private double driveKp = 0.35; //change this untill it drives the path right, trial and error
  private double driveKf = 0;
  private double driveKd = 0;
  private double driveKi = 0;
  private double driveAngularKV = 5.0833; //trackwidth
  Trajectory trajectory = new Trajectory();
  Timer autoTimer = new Timer();
  RamseteController controller = new RamseteController();
  private double gearRatio = (8.0/62.0) * (23.0/26.0);
  private boolean autoComplete = false;
  private String getSelectedAuto;

  //Limelight
  private double limeLightAngle = 37.0;
  private double limeLightHeight = 24.0;
  private double goalHeight = 102.5;
  private double goalDistance = 0.0;
  private void distanceToGoalCalculator(){
    double targetOffsetAngle_Vertical = getGoalVerticalOffset();
    double angleToGoalDegrees = limeLightAngle + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);
    goalDistance = (goalHeight - limeLightHeight)/Math.tan(angleToGoalRadians);
    //System.out.println("Distance to Goal: " + goalDistance);

  }

///////////////////////////////
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    //camera
    CameraServer.startAutomaticCapture();
    
    //initiate variables
    /*controlers*/
    joystick = new Joystick(0);
    xbox = new Joystick(1);

    /*drivetrain*/
    //leftDrive[0] = new WPI_TalonFX(LEFT_DRIVE_BACK);
    //leftDrive[1] = new WPI_TalonFX(LEFT_DRIVE_FRONT);
    leftDriveBack = new WPI_TalonFX(9);
    leftDriveFront = new WPI_TalonFX(7);
    rightDrive[0] = new WPI_TalonFX(RIGHT_DRIVE_BACK);
    rightDrive[1] = new WPI_TalonFX(RIGHT_DRIVE_FRONT);
    leftDriveBack.configFactoryDefault();
    leftDriveFront.configFactoryDefault();
    rightDrive[0].configFactoryDefault();
    rightDrive[1].configFactoryDefault();
    //leftDriveFront.follow(leftDriveBack);
    //leftDrive[1].follow(leftDrive[0]);
    rightDrive[1].follow(rightDrive[0]);
    //leftDrive[0].config_kP(0, 0.15);
    //leftDrive[1].config_kP(0, 0.15);
    leftDriveFront.config_kP(0, driveKp);
    leftDriveBack.config_kP(0, driveKp);
    rightDrive[0].config_kP(0, driveKp);
    rightDrive[1].config_kP(0, driveKp);
    leftDriveFront.config_kI(0, driveKi);
    leftDriveBack.config_kI(0, driveKi);
    rightDrive[0].config_kI(0, driveKi);
    rightDrive[1].config_kI(0, driveKi);
    leftDriveFront.config_kD(0, driveKd);
    leftDriveBack.config_kD(0, driveKd);
    rightDrive[0].config_kD(0, driveKd);
    rightDrive[1].config_kD(0, driveKd);
    leftDriveFront.config_kF(0, driveKf);
    leftDriveBack.config_kF(0, driveKf);
    rightDrive[0].config_kF(0, driveKf);
    rightDrive[1].config_kF(0, driveKf);
    //leftDrive[0].setInverted(true);
    //leftDrive[1].setInverted(true);
    leftDriveFront.setInverted(true);
    leftDriveBack.setInverted(true);

    lsideDrive = new MotorControllerGroup(leftDriveFront, leftDriveBack);
    //lsideDrive = new MotorControllerGroup(leftDrive[0], leftDrive[1]);
    //rsideDrive = new MotorControllerGroup(rightDrive[0], rightDrive[1]);
    chassis = new DifferentialDrive(lsideDrive, rightDrive[0]);

    /*intake*/
    intake= new CANSparkMax(INTAKE_NEO, MotorType.kBrushless);
    intake.setSmartCurrentLimit(30);

    /*shooter*/
    shooterLeft = new WPI_TalonFX(LEFT_SHOOTER);
    shooterRight = new WPI_TalonFX(RIGHT_SHOOTER);
    shooterRight.setInverted(true);
    shooterLeft.config_kP(0, 0.15);
    shooterRight.config_kP(0, 0.15);
    shooterLeft.config_kF(0, 0.05);
    shooterRight.config_kF(0, 0.05);
    shooter = new MotorControllerGroup(shooterLeft, shooterRight);
    //shooter.setInverted(true);


    //treemap
    myTreeMap.put(new InterpolatingDouble(93.3), new InterpolatingDouble(3300.0));
    myTreeMap.put(new InterpolatingDouble(106.7), new InterpolatingDouble(3350.0));
    myTreeMap.put(new InterpolatingDouble(125.3), new InterpolatingDouble(3550.0));
    myTreeMap.put(new InterpolatingDouble(134.6), new InterpolatingDouble(3675.0));
    myTreeMap.put(new InterpolatingDouble(147.5), new InterpolatingDouble(4000.0));
    /*turret*/
    turret = new WPI_TalonSRX(TURRET);

    /*hopper*/
    hopperStage1 = new WPI_VictorSPX(INTAKE_STAGE1);
    hopperStage2 = new WPI_VictorSPX(INTAKE_STAGE2);

    /*climber*/
    climberLeft = new WPI_TalonFX(LEFT_CLIMBER);
    climberRight = new WPI_TalonFX(RIGHT_CLIMBER);
    climber = new MotorControllerGroup(climberLeft, climberRight);
    climberPivotLeftFront = new WPI_TalonSRX(17);
    climberPivotRightFront = new WPI_TalonSRX(2);
    climberPivotLeftBack = new WPI_TalonSRX(3);
    climberPivotRightBack = new WPI_TalonSRX(16);
    climberPivotRightFront.setInverted(false);
    climberPivotRightBack.setInverted(false);
    climberPivotAll = new MotorControllerGroup(climberPivotRightFront,climberPivotRightBack,climberPivotLeftFront,climberPivotLeftBack);
    /*solinoid*/
    //intakePivot = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);

    //limelight
    limelight = NetworkTableInstance.getDefault().getTable("limelight");
    limelight.getEntry("ledMode").setNumber(1);
///////////////////////////////
    //auto chooser
    String[] autoChoices = {"Drive Forward", "Shoot 2 Human Side", "Shoot 2 Hanger Side", "Get Out Of The Way"};
    SmartDashboard.putStringArray("Auto List", autoChoices);
    
    gyro.reset();
    //leftDrive[0].setSelectedSensorPosition(0.0);
    leftDriveBack.setSelectedSensorPosition(0.0);
    //leftDrive[1].setSelectedSensorPosition(0.0);
    rightDrive[0].setSelectedSensorPosition(0.0);
    //rightDrive[1].setSelectedSensorPosition(0.0);

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    leftMeters = falconEncodertoMeters(leftDriveBack.getSelectedSensorPosition());
    rightMeters =  falconEncodertoMeters(rightDrive[0].getSelectedSensorPosition());
    Rotation2d myRotation2d = Rotation2d.fromDegrees(-gyro.getAngle());
    m_odometry.update(myRotation2d, leftMeters, rightMeters);
    myField.setRobotPose(m_odometry.getPoseMeters());
    //smart dashboard values
    SmartDashboard.putData("Field", myField);
    SmartDashboard.putNumber("Shooter rpm", shooter.get());
    SmartDashboard.putNumber("Gyro",gyro.getAngle());
    Double[] driveMotorSpeeds = {leftDriveFront.getMotorOutputPercent(), leftDriveBack.getMotorOutputPercent(), rightDrive[0].getMotorOutputPercent(), rightDrive[1].getMotorOutputPercent()};
    SmartDashboard.putNumberArray("RobotDrive Motors", driveMotorSpeeds);
    SmartDashboard.putBoolean("Auto Running", autoComplete);
    distanceToGoalCalculator();

  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    getSelectedAuto = SmartDashboard.getString("Auto Selector", "Drive Forward");  //get selected auto from selector in dashboard default is Drive Forward
    switch(getSelectedAuto){ //load the path for the selected auto and store
      case "Drive Forward":
        trajectoryJason = "paths/driveStraight.wpilib.json";
        break;
      case "Shoot 2 Human Side":
        trajectoryJason = "paths/driveandshoot.wpilib.json";
        break;
      case "Shoot 2 Hanger Side":
        trajectoryJason = "paths/hangerside.wpilib.json";
        break;
      case "Get Out Of The Way":
        trajectoryJason = "paths/hangerside.wpilib.json";
        break;
    }

    //get the path from the file path
    try{
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJason);
      trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException ex){
      DriverStation.reportError("Unable to open trajectory: " + trajectoryJason, ex.getStackTrace());
    }
    leftDriveFront.setNeutralMode(NeutralMode.Brake);
    leftDriveBack.setNeutralMode(NeutralMode.Brake);
    rightDrive[0].setNeutralMode(NeutralMode.Brake);
    rightDrive[1].setNeutralMode(NeutralMode.Brake);
    m_odometry.resetPosition(trajectory.getInitialPose(), Rotation2d.fromDegrees(gyro.getAngle()));
    autoTimer.reset();
    autoTimer.start();
    //intakePivot.set(kReverse);
    limelight.getEntry("ledMode").setNumber(3);
    limelight.getEntry("camMode").setNumber(0);
    cameraMode = 0;
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() { //run the extra stuf unique to each auto mode
    autoComplete = true;
    //intakePivot.set(kReverse);
    intake.set(0.3);
    switch(getSelectedAuto){
      case "Drive Forward":
        break;
      case "Shoot 2 Human Side":
        targetRPM = 3500;
        if (autoTimer.get() > 5){
          limelight.getEntry("ledMode").setNumber(3);
          limelight.getEntry("camMode").setNumber(0);
          cameraMode = 0;
          shooterLeft.set(ControlMode.Velocity, rpmToFalcon(targetRPM));
          shooterRight.set(ControlMode.Velocity, rpmToFalcon(targetRPM));
          aimTurret_automatic();
        }
          if (autoTimer.get() > 8){
            hopperStage1.set(0.85);
            hopperStage2.set(-0.85);
          }
          if (autoTimer.get() > 13){
            intake.stopMotor();
            hopperStage1.stopMotor();
            hopperStage2.stopMotor();
            shooterLeft.stopMotor();
            shooterRight.stopMotor();
          }
        break;
      case "Shoot 2 Hanger Side":
        targetRPM = 3300;
        if (autoTimer.get() > 3){
          limelight.getEntry("ledMode").setNumber(3);
          limelight.getEntry("camMode").setNumber(0);
          cameraMode = 0;
          shooterLeft.set(ControlMode.Velocity, rpmToFalcon(targetRPM));
          shooterRight.set(ControlMode.Velocity, rpmToFalcon(targetRPM));
        }
        if (autoTimer.get()>7){
          aimTurret_automatic();
        }
        if (autoTimer.get() > 8){
          hopperStage1.set(0.85);
          hopperStage2.set(-0.85);
        }
        if (autoTimer.get() > 12){
          intake.stopMotor();
          hopperStage1.stopMotor();
          hopperStage2.stopMotor();
          shooterLeft.stopMotor();
          shooterRight.stopMotor();
        }

        break;
      case "Get Out Of The Way":
        break;
    }
    //run the path, same for any path
    State currentState = trajectory.sample(autoTimer.get());
    //State previousState = trajectory.sample(autoTimer.get()-0.02);
    DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(24.5));
    ChassisSpeeds speeds = controller.calculate(m_odometry.getPoseMeters(), 
    currentState.poseMeters, currentState.velocityMetersPerSecond, currentState.velocityMetersPerSecond*currentState.curvatureRadPerMeter);
    double angularVelocity = currentState.curvatureRadPerMeter*currentState.velocityMetersPerSecond;
    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
    //double acceleration = currentState.accelerationMetersPerSecondSq;
    double leftVelocity = wheelSpeeds.leftMetersPerSecond;
    double rightVelcoity = wheelSpeeds.rightMetersPerSecond;
    double leftRPM = mpsToRpm(leftVelocity);
    double rightRPM = mpsToRpm(rightVelcoity);
    double leftCTRE = rpmToFalcon(leftRPM);
    double rightCTRE = rpmToFalcon(rightRPM);
    double leftFeedForward = leftVelocity*driveKv;
    //leftFeedForward += acceleration*driveKa;
    leftFeedForward -= angularVelocity*driveAngularKV;
    leftFeedForward += signOf(leftVelocity)*driveKs;
    leftFeedForward /= 12.0;
    double rightFeedForward = rightVelcoity*driveKv;
    //rightFeedForward += acceleration*driveKa;
    rightFeedForward += angularVelocity*driveAngularKV;
    rightFeedForward += signOf(rightVelcoity)*driveKs;
    rightFeedForward /= 12.0;
    leftDriveBack.set(ControlMode.Velocity,leftCTRE,DemandType.ArbitraryFeedForward, leftFeedForward );
    leftDriveFront.set(ControlMode.Velocity,leftCTRE,DemandType.ArbitraryFeedForward, leftFeedForward  );
    rightDrive[0].set(ControlMode.Velocity, rightCTRE,DemandType.ArbitraryFeedForward, rightFeedForward );
    //rightDrive[1].set(ControlMode.Velocity, rightCTRE, DemandType.ArbitraryFeedForward, rightFeedForward );
    chassis.feed();
  }
  
  private double signOf(double num){
    if (num < 0){
      return -1.0;
    } else if (num > 0){
      return 1.0;
    }
    return 0.0;
  }
  
  private double falconEncodertoMeters(double encoder){
    double wheelDiameterInches = 6.0;
    double wheelDiameterMeters = wheelDiameterInches * 0.0254;
    double wheelCircumferenceMeters = wheelDiameterMeters * Math.PI;
    double motorRotation = encoder/2048.0;
    double wheelRoation = motorRotation*gearRatio;
    double meters = wheelRoation*wheelCircumferenceMeters;
    return meters;
  }
  private double mpsToRpm(double mps){
    double wheelDiameterInches = 6.0;
    double wheelDiameterMeters = wheelDiameterInches * 0.0254;
    double wheelCircumferenceMeters = wheelDiameterMeters * Math.PI;
    double mpm = mps*60.0;
    double wheelRPM = mpm/wheelCircumferenceMeters;
    double motorRPM = wheelRPM/gearRatio;
    return motorRPM;
  }
  private double rpmToFalcon(double rpm){
    return (rpm*2048.0)/600.0;
  }
  private double falconToRPM(double falcon){
    return (falcon*600)/2048.0;
  }

  //robot control methods
  private void drive() { // Drives the robot
    double topSpeed = 1;
    //System.out.println(joystick.getRawAxis(DRIVING_SPEED));
    chassis.arcadeDrive(-joystick.getRawAxis(FOREWARD_BACKWARD_AXIS) * topSpeed, joystick.getRawAxis(LEFT_RIGHT_AXIS) * 0.75);
  }

  private void aimTurret_manual() { // Manual turret aiming using xbox controller
    if (!(xbox.getRawAxis(TURRET_AIM) == 0 )){
      double speed = xbox.getRawAxis(TURRET_AIM) * -0.3;
      turret.set(speed);
    } else{
      turret.stopMotor();
    }
;
  }

  private double getGoalHorizontalOffset() { // Tells how far off the limelight is in the X direction, unit is degrees
    return limelight.getEntry("tx").getDouble(0);
  }

  private double getGoalVerticalOffset() { // Tells how far off the limelight is in the X direction, unit is degrees
    return limelight.getEntry("ty").getDouble(0);
  }

  private void aimTurret_automatic() { // Auto aims turret maby

    double xOffset = getGoalHorizontalOffset(); // Horizontal offset
     
    //System.out.println(xOffset);
    
    if (xOffset < 2 && xOffset > -2) { // If it is within 2 degrees on either side of the goal, dont move it
      turret.stopMotor();
    } else {
        if (xOffset > 20){
          turret.set(-0.3);
        } else if (xOffset > 10) { // If it is greater than 10 degrees off
          turret.set(-0.2);  // Aim towards the goal at 20% power
        } else if (xOffset > 0) { // It is not explicitly stated in the condition, but there is an implied xOffset <= 10, so this is if 0 < xOffset <= 10
          turret.set(-0.1);        // Aim towards the goal at 10% power
        }
        if (xOffset < -20){
          turret.set(0.3);
        }else if (xOffset < -10) { // This is the same as above, just in the other direction
          turret.set(0.2);
        } else if (xOffset < 0) {
          turret.set(0.1);
        }
      } 
      
  }

  private void aimTurretTest(){
    if (joystick.getRawButton(THUMB_BUTTON)){
      aimOn = !aimOn;
    }
  }
  private void aimTurretToggle() {
    aimTurret_manual();
    if (aimOn) {
      aimTurret_automatic();
    }
  }
  private void aimTurret() {
    aimTurret_manual();
    if (joystick.getRawButton(THUMB_BUTTON)) {
      aimTurret_automatic();
    }
  }

  private void shooterOnTest(){
    if (joystick.getRawButton(TRIGGER)){
      shooterOn = !shooterOn;
    }
  }
  private void controlShooterToggle() {
    double targetRPM = 500;
    if (shooterOn) { // If the trigger is pulled (joystick)
      //shooter.set(0.6); // Set the speed of the shooter
      shooterLeft.set(ControlMode.Velocity, rpmToFalcon(targetRPM));
      shooterRight.set(ControlMode.Velocity, rpmToFalcon(targetRPM));
    } else if(!shooterOn) { // If the trigger is not pulled, stop the motor                                                  
      //shooter.set(0);
      //shooter.stopMotor();
      shooterLeft.stopMotor();
      shooterRight.stopMotor();
    }
    System.out.println("Velocity: " + falconToRPM(shooterLeft.getSelectedSensorVelocity()));
  }


  private void controlShooter() {
    /*
    if (joystick.getRawButtonPressed(9)){
      targetRPM -= 25;
    }
    if (joystick.getRawButtonPressed(10)){
      targetRPM += 25;
    }
    */
    targetRPM = myTreeMap.getInterpolated(new InterpolatingDouble(goalDistance)).value;
    System.out.println("Target RPM" + targetRPM);
    targetRPM *= 1;
    if (joystick.getRawButton(TRIGGER)) { // If the trigger is pulled (joystick)
      //shooter.set(0.6); // Set the speed of the shooter
      shooterLeft.set(ControlMode.Velocity, rpmToFalcon(targetRPM));
      shooterRight.set(ControlMode.Velocity, rpmToFalcon(targetRPM));
    } else{ // If the trigger is not pulled, stop the motor                                                  
      //shooter.set(0);
      //shooter.stopMotor();
      shooterLeft.stopMotor();
      shooterRight.stopMotor();
    }
    System.out.println("Velocity: " + falconToRPM(shooterLeft.getSelectedSensorVelocity()));
  }

  private void checkLimelightMode() {
    if (joystick.getRawButtonPressed(CAMERA_TOGGLE)) {
      if (cameraMode == 0) {
        limelight.getEntry("ledMode").setNumber(1);
        limelight.getEntry("camMode").setNumber(1);
        cameraMode = 1;
      } else {
        limelight.getEntry("ledMode").setNumber(3);
        limelight.getEntry("camMode").setNumber(0);
        cameraMode = 0;
      }
    }
  }

  private void intakeControl(){
    if (xbox.getRawButton(A)){ //everything
      intake.set(BASE_INTAKE_SPEED*INTAKE_SPEED);
      hopperStage1.set(BASE_INTAKE_SPEED*HOPPER_SPEED);
      hopperStage2.set(BASE_INTAKE_SPEED*-HOPPER_SPEED);
    } else if (xbox.getRawButton(X)){
      hopperStage1.set(BASE_INTAKE_SPEED*HOPPER_SPEED);
    } else if (xbox.getRawButton(Y)){
      hopperStage2.set(BASE_INTAKE_SPEED*-HOPPER_SPEED);
    } else if (xbox.getRawButton(B)){
      intake.set(BASE_INTAKE_SPEED*INTAKE_SPEED);
    } else if (xbox.getRawButton(RIGHT_BUMPER)){
      hopperStage1.set(BASE_INTAKE_SPEED*(-HOPPER_SPEED));
      hopperStage2.set(BASE_INTAKE_SPEED*(HOPPER_SPEED));
    } else if (xbox.getRawButton(LEFT_BUMPER)){
      intake.set(BASE_INTAKE_SPEED*(-INTAKE_SPEED));
    } else if ((xbox.getRawButton(LEFT_BUMPER)) && (xbox.getRawButton(RIGHT_BUMPER))){
      intake.set(BASE_INTAKE_SPEED*(-INTAKE_SPEED));
      hopperStage1.set(BASE_INTAKE_SPEED*(-HOPPER_SPEED));
      hopperStage2.set(BASE_INTAKE_SPEED*(HOPPER_SPEED));
      intake.set(BASE_INTAKE_SPEED*INTAKE_SPEED);
    } else{
      intake.stopMotor();
      hopperStage1.stopMotor();
      hopperStage2.stopMotor();
    }
  }

  private void climberControl(){
    if (xbox.getRawAxis(LEFT_TRIGGER) > 0){
      climber.set(xbox.getRawAxis(LEFT_TRIGGER) * 0.5);
    } else if (xbox.getRawAxis(RIGHT_TRIGGER) > 0){
      climber.set(xbox.getRawAxis(RIGHT_TRIGGER) * (-0.5));
    } else{
      climber.stopMotor();
    }
  }

  private void climberAngleControll(){
    /*
    if (pivot){
      climberPivotAll.set(-0.5);
    }
    */
    if (xbox.getRawAxis(CLIMBER_ANGLE_AXIS) != 0){
      //System.out.println("its working");
      climberPivotAll.set(xbox.getRawAxis(CLIMBER_ANGLE_AXIS)*1);
      pivot = false;
    }else{
      pivot = true;
    }
  }

  private void intakePivotControl(){
    if (xbox.getPOV() == 0){
      //intakePivot.toggle();
    } else if (xbox.getPOV() == 180){
      //intakePivot.toggle();
    }
  }

  private void intakePivotControlDpad(){
    if (xbox.getPOV() == 180){
      //intakePivot.toggle();
    } 
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    targetRPM = 3500;
    //chassis.arcadeDrive(0, 0);
    leftDriveFront.stopMotor();
    //leftDrive[1].stopMotor();
    rightDrive[0].stopMotor();
    //leftDrive[0].stopMotor();
    leftDriveFront.setNeutralMode(NeutralMode.Brake);
    leftDriveBack.setNeutralMode(NeutralMode.Brake);
    rightDrive[0].setNeutralMode(NeutralMode.Brake);
    rightDrive[1].setNeutralMode(NeutralMode.Brake);
    shooterLeft.stopMotor();
    shooterRight.stopMotor();
    turret.stopMotor();
    climberLeft.stopMotor();
    climberRight.stopMotor();
    hopperStage1.stopMotor();
    hopperStage1.stopMotor();
    intake.stopMotor();
    //intakePivot.set(kForward);
    limelight.getEntry("ledMode").setNumber(3);
    limelight.getEntry("camMode").setNumber(0);
    cameraMode = 0;
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    drive();
    //aimTurret_manual();
    //aimTurret_automatic();
    aimTurret();
    //shooterOnTest();
    //controlShooterToggle();
    controlShooter();
    intakeControl();
    climberControl();
    checkLimelightMode();
    //intakePivotControl();
    climberAngleControll();

  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    leftDriveFront.stopMotor();
    //leftDrive[1].stopMotor();
    rightDrive[0].stopMotor();
    //rightDrive[1].stopMotor();
    leftDriveFront.setNeutralMode(NeutralMode.Coast);
    leftDriveBack.setNeutralMode(NeutralMode.Coast);
    rightDrive[0].setNeutralMode(NeutralMode.Coast);
    rightDrive[1].setNeutralMode(NeutralMode.Coast);
    shooterLeft.stopMotor();
    shooterRight.stopMotor();
    turret.stopMotor();
    climberLeft.stopMotor();
    climberRight.stopMotor();
    hopperStage1.stopMotor();
    hopperStage1.stopMotor();
    intake.stopMotor();
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {
    limelight.getEntry("ledMode").setNumber(1);
    limelight.getEntry("camMode").setNumber(1);
    cameraMode = 1;
    /*
    SmartDashboard.putString("DB/String 0", "LB: " +String.valueOf(leftDrive[0].getTemperature()));
    SmartDashboard.putString("DB/String 1","LF: " +String.valueOf(leftDrive[1].getTemperature()));
    SmartDashboard.putString("DB/String 2","RB: " +String.valueOf(rightDrive[0].getTemperature()));
    SmartDashboard.putString("DB/String 3","RF: " +String.valueOf(rightDrive[1].getTemperature()));
    SmartDashboard.putString("DB/String 5","ShooterLeft: " +String.valueOf(shooterLeft.getTemperature()));
    SmartDashboard.putString("DB/String 6","ShooterRight: " +String.valueOf(shooterRight.getTemperature()));
    SmartDashboard.putString("DB/String 7","Intake: " +String.valueOf(intake.getMotorTemperature()));
    
    try {
    Thread.sleep(1000);
    } catch(InterruptedException ex) {
    Thread.currentThread().interrupt();
    }
    */
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    leftDriveFront.setNeutralMode(NeutralMode.Coast);
    leftDriveBack.setNeutralMode(NeutralMode.Coast);
    rightDrive[0].setNeutralMode(NeutralMode.Coast);
    rightDrive[1].setNeutralMode(NeutralMode.Coast);
    shooterLeft.stopMotor();
    shooterRight.stopMotor();
    turret.stopMotor();
    climberLeft.stopMotor();
    climberRight.stopMotor();
    hopperStage1.stopMotor();
    hopperStage1.stopMotor();
    intake.stopMotor();   
    leftDriveFront.stopMotor();
    //leftDrive[1].stopMotor();
    rightDrive[0].stopMotor();
    //rightDrive[1].stopMotor();
 
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
