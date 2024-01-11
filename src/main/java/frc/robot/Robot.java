// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


/*
 * Welcome to the Everybot team's starter code for the KitBot.
 * 
 * We decided to create a time based version to mimic our code from
 * past years for teams who are used to our style and time based.
 */

package frc.robot;

// Imports that allow the usage of REV Spark Max motor controllers
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class Robot extends TimedRobot {
  /*
   * Autonomous selection options.
   */
  private static final String kNothingAuto = "do nothing";
  private static final String kLaunchAndDrive = "launch drive";
  private static final String kLaunch = "launch";
  private static final String kDrive = "drive";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  /*
   * Drive motor controller instances.
   * 
   * Change the id's to match your robot.
   * Change kBrushed to kBrushless if you are uisng NEOs.
   * The rookie kit comes with CIMs which are brushed motors.
   * Use the appropriate other class if you are using different controllers.
   */
  CANSparkBase leftRear = new CANSparkMax(1, MotorType.kBrushed);
  CANSparkBase leftFront = new CANSparkMax(2, MotorType.kBrushed);
  CANSparkBase rightRear = new CANSparkMax(3, MotorType.kBrushed);
  CANSparkBase rightFront = new CANSparkMax(4, MotorType.kBrushed);

  /*
   * A class provided to control your drivetrain. Different drive styles can be passed to differential drive:
   * https://github.com/wpilibsuite/allwpilib/blob/main/wpilibj/src/main/java/edu/wpi/first/wpilibj/drive/DifferentialDrive.java 
   */
  DifferentialDrive m_drivetrain;

  /*
   * Mechanism motor controller instances.
   * 
   * Like the drive motors, set the CAN id's to match your robot or use different
   * motor controller classses (VictorSPX) to match your robot.
   * 
   * Both of the motors used on the kitbot mechansim are CIMs which are brushed motors
   */
  CANSparkBase m_launchWheel = new CANSparkMax(6, MotorType.kBrushed);
  CANSparkBase m_feedWheel = new CANSparkMax(5, MotorType.kBrushed);

    /**
   * The starter code uses the most generic joystick class.
   * 
   * To determine what a button does, open FRC driver station, go to the USB tab, plug in a controller and see which button lights ups
   * 
   * Buttons index from 0
   */
  Joystick m_driverController = new Joystick(0);

  /*
   * Many teams prefer two students to control their robot, typically one on drive, the other on mechanism
   * Uncomment below and replace m_driverController in the desired locations
  /*

  //Joystick m_manipController = new Joystick(1);


   
  // --------------- Magic numbers. Use these to adjust settings. ---------------


  /**
   * How many amps can an individual drivetrain motor use.
   */
  static final int DRIVE_CURRENT_LIMIT = 60;

  /**
   * How many amps the feeder motor can use.
   */
  static final int FEEDER_CURRENT_LIMIT = 80;

  /**
   * Percent output to run the feeder when expelling note
   */
  static final double FEEDER_OUT_SPEED = 1.0;

  /**
   * Percent output to run the feeder when intaking note
   */
  static final double FEEDER_IN_SPEED = -.4;
  /**
   * How many amps the launcher motor can use.
   * 
   * In our testing we favored the CIM over NEO, if using a NEO lower this to 60
   */
  static final int LAUNCHER_CURRENT_LIMIT = 80;

  /**
   * Percent output to run the launcher when intaking AND expelling note
   */
  static final double LAUNCHER_SPEED = 1.0;

  /**
   * How long (in seconds) should the launcher wheel spin up before the note is fed
   * 
   * This time may be lowered but allowing the wheel to achieve max speed will likely yield best performance
   */
  static final double LAUNCHER_DELAY = 1;


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("do nothing", kNothingAuto);
    m_chooser.addOption("launch note and drive", kLaunchAndDrive);
    m_chooser.addOption("launch", kLaunch);
    m_chooser.addOption("drive", kDrive);
    SmartDashboard.putData("Auto choices", m_chooser);



    /*
     * Apply the current limit to the drivetrain motors
     */
    leftRear.setSmartCurrentLimit(DRIVE_CURRENT_LIMIT);
    leftFront.setSmartCurrentLimit(DRIVE_CURRENT_LIMIT);
    rightRear.setSmartCurrentLimit(DRIVE_CURRENT_LIMIT);
    rightFront.setSmartCurrentLimit(DRIVE_CURRENT_LIMIT);

    /*
     * Tells the rear wheels to follow the same commands as the front wheels
     */
    leftRear.follow(leftFront);
    rightRear.follow(rightFront);

    /*
     * One side of the drivetrain must be inverted, as the motors are facing opposite directions
     */
    leftFront.setInverted(true);
    rightFront.setInverted(false);

    m_drivetrain = new DifferentialDrive(leftFront, rightFront);

    /*
     * Mechanism wheel(s) spinning the wrong direction? Change to true here.
     * 
     * Add white tape to wheel to help determine spin direction.
     */
    m_feedWheel.setInverted(true);
    m_launchWheel.setInverted(true);

    /*
     * Apply the current limit to the launching mechanism 
     */
    m_feedWheel.setSmartCurrentLimit(FEEDER_CURRENT_LIMIT);
    m_launchWheel.setSmartCurrentLimit(LAUNCHER_CURRENT_LIMIT);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Time (seconds)", Timer.getFPGATimestamp());
  }


  /*
   * Auto constants, change values for different autonomous behaviour
   * 
   * A delayed action starts X seconds into the autonomous period
   * 
   * A time action will perform an action for X amount of seconds
   * 
   * Speeds can be changed as desired and will be set to 0 when
   * performing an auto that does not require the system 
   */
  double AUTO_LAUNCH_DELAY;
  double AUTO_DRIVE_DELAY;

  double AUTO_DRIVE_TIME;

  double AUTO_DRIVE_SPEED;
  double AUTO_LAUNCHER_SPEED;

  double autonomousStartTime;

  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();

    leftRear.setIdleMode(IdleMode.kBrake);
    leftFront.setIdleMode(IdleMode.kBrake);
    rightRear.setIdleMode(IdleMode.kBrake);
    rightFront.setIdleMode(IdleMode.kBrake);

    AUTO_LAUNCH_DELAY = 2;
    AUTO_DRIVE_DELAY = 3;

    AUTO_DRIVE_TIME = 2.0;

    AUTO_DRIVE_SPEED = -0.5;
    AUTO_LAUNCHER_SPEED = 1;
    
    /*
     * Sets speed to 0 if not using the action
     * 
     * For kDrive you can also change the kAutoDriveBackDelay
     */
    if(m_autoSelected == kLaunch)
    {
      AUTO_DRIVE_SPEED = 0;
    }
    else if(m_autoSelected == kDrive)
    {
      AUTO_LAUNCHER_SPEED = 0;
    }
    else if(m_autoSelected == kNothingAuto)
    {
      AUTO_DRIVE_SPEED = 0;
      AUTO_LAUNCHER_SPEED = 0;
    }
    autonomousStartTime = Timer.getFPGATimestamp();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

    double timeElapsed = Timer.getFPGATimestamp() - autonomousStartTime;

    /*
     * Spins up launcher wheel until time spent in auto is greater than AUTO_LAUNCH_DELAY
     * 
     * Feeds note to launcher until time is greater than AUTO_DRIVE_DELAY
     * 
     * Drives until time is greater than AUTO_DRIVE_DELAY + AUTO_DRIVE_TIME
     * 
     * Does not move when time is greater than AUTO_DRIVE_DELAY + AUTO_DRIVE_TIME
     */
    if(timeElapsed < AUTO_LAUNCH_DELAY)
    {
      m_launchWheel.set(AUTO_LAUNCHER_SPEED);
      m_drivetrain.arcadeDrive(0, 0);

    }
    else if(timeElapsed < AUTO_DRIVE_DELAY)
    {
      m_feedWheel.set(AUTO_LAUNCHER_SPEED);
      m_drivetrain.arcadeDrive(0, 0);
    }
    else if(timeElapsed < AUTO_DRIVE_DELAY + AUTO_DRIVE_TIME)
    {
      m_launchWheel.set(0);
      m_feedWheel.set(0);
      m_drivetrain.arcadeDrive(AUTO_DRIVE_SPEED, 0);
    }
    else
    {
      m_drivetrain.arcadeDrive(0, 0);
    }
    /* For an explanation on differintial drive, squaredInputs, arcade drive and tank drive see the bottom of this file */
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    /*
     * Motors can be set to idle in brake or coast mode. 
     * 
     * Brake mode effectively shorts the leads of the motor when not running, making it more
     * difficult to turn when not running.
     * 
     * Coast doesn't apply any brake and allows the motor to spin down natuarlly.
     * 
     * (touch the leads of a motor and spin the shaft to feel the difference)
     * 
     * This setting is driver preferene.
     */
    leftRear.setIdleMode(IdleMode.kCoast);
    leftFront.setIdleMode(IdleMode.kCoast);
    rightRear.setIdleMode(IdleMode.kCoast);
    rightFront.setIdleMode(IdleMode.kCoast);
  }

  Timer launcherHoldTime = new Timer();

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    /*
     * Starts a timer when button has been pressed and
     * spins up launcher
     */
    if (m_driverController.getRawButtonPressed(1)) {
      launcherHoldTime.restart();
      m_launchWheel.set(LAUNCHER_SPEED);
    } 

    /*
     * If the button has been held longer than LAUNCHER_DELAY (1 second by default)
     * then spin the feeder wheel
     */
    if(launcherHoldTime.get() >= LAUNCHER_DELAY)
    {
      m_feedWheel.set(FEEDER_OUT_SPEED);
    }

    /*
     * When the launch button has been released stop the launching motors,
     * reset the timer and stop it.
     */
    if(m_driverController.getRawButtonReleased(1))
    {
      m_launchWheel.set(0);
      m_feedWheel.set(0);
      launcherHoldTime.reset();
      launcherHoldTime.stop();
    }

    /*
     * While the button is being held spin both motors to intake note
     */
    if(m_driverController.getRawButton(5))
    {
      m_launchWheel.set(-LAUNCHER_SPEED);
      m_feedWheel.set(FEEDER_IN_SPEED);
    }
    else if(m_driverController.getRawButtonReleased(5))
    {
      m_launchWheel.set(0);
      m_feedWheel.set(0);
    }

    if(m_driverController.getRawButton(3))
    {
      m_feedWheel.set(.4);
      m_launchWheel.set(.14);
    }
    else if(m_driverController.getRawButtonReleased(3))
    {
      m_feedWheel.set(0);
      m_launchWheel.set(0);
    }
    
    /*
     * Negative signs here because the values from the analog sticks are backwards
     * from what we want. Forward returns a negative when we want it positive.
     * 
     * You may want to change the axis used, open the driver station, go to the usb tab and push buttons to determine axis
     */
    m_drivetrain.arcadeDrive(-m_driverController.getRawAxis(1), -m_driverController.getRawAxis(4), false);
  }
}

/*
 * The kit of parts drivetrain is known as differential drive, tank drive or skid-steer drive.
 * 
 * There are two common ways to control this drivetrain: Arcade and Tank
 * 
 * Arcade allows one stick to be pressed forward/backwards to power both sides of the drivetrain to move straight forwards/backwards.
 * A 2nd stick can be pushed left/right to turn the robot in place.
 * When combined the robot will power the drivetrain such that it both moves fowards and turns, forming an arch.
 * 
 * Tank drive allows a single stick to control of a single side of the robot.
 * Push the left stick forward to power the left side of the drive train, causing the robot to spin around the wheel.
 * Push the right stick to power the opposite side.
 * Push both at equal distances to drive forwards/backwards and use at different speeds to turn in an arch.
 * Push both in opposite directions to spin in place. 
 * 
 * arcardeDrive can be replaced with tankDrive like so:
 * 
 * m_drivetrain.tankDrive(-m_driverController.getRawAxis(1), -m_driverController.getRawAxis(5))
 * 
 * Inputs can be squared which decreases the sensitivity of small drive inputs. 
 * 
 * It litterally just takes (your inputs * your inputs), so a 50% (0.5) input from the controller becomes (0.5 * 0.5) -> 0.25
 * 
 * This is an option that can be passed into arcade or tank drive:
 * arcadeDrive(double xSpeed, double zRotation, boolean squareInputs)
 * 
 * 
 * For more information see: 
 * https://docs.wpilib.org/en/stable/docs/software/hardware-apis/motors/wpi-drive-classes.html
 * 
 * https://github.com/wpilibsuite/allwpilib/blob/main/wpilibj/src/main/java/edu/wpi/first/wpilibj/drive/DifferentialDrive.java 
 * 
 */