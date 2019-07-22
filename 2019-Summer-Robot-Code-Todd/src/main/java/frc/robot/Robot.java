/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Joystick;

import frc.systems.DriveSystem;
import frc.systems.Elevator;
import frc.systems.ElevatorMMTest;
import frc.systems.Arm;
import frc.systems.HatchMech;

import frc.systems.sensors.Cameras;
import frc.systems.sensors.IMU;
import frc.systems.sensors.ADIS16448_IMU;

import frc.utilities.RoboRioPorts;

import frc.autonomous.DashboardInterface;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  public static Joystick leftJoystick;
  public static Joystick rightJoystick;
  public static Joystick xboxJoystick;

  public static Timer systemTimer;
  public static IMU mImu;
  public static ADIS16448_IMU robotIMU;
  public static boolean isEnabled;

  public static int nSequenceVisionSystem;
  public static JTargetInfo visionTargetInfo;

  Cameras robotCameraSystem;
  JReceiver visionInfoReceiver;
  Thread visionThread;

  public static DashboardInterface mSDBInterface;

  Notifier driveRateGroup;
  public static DriveSystem mDriveSystem;

  Notifier liftRateGroup;
  public static Elevator mElevator;
  public static ElevatorMMTest mElevatorMM;

  Notifier armRateGroup;
  public static Arm mArm;

  Notifier hatchRateGroup;
  public static HatchMech mHatch;

  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  @Override
  public void robotInit() {
    systemTimer = new Timer();
    mImu = new IMU();

    robotCameraSystem = new Cameras();
    visionInfoReceiver = new JReceiver();
    visionTargetInfo = new JTargetInfo();
    nSequenceVisionSystem = 0;
    visionThread = new Thread(new JVisionSystemReceiverRunnable());
    visionThread.start();

    leftJoystick = new Joystick(0);
    rightJoystick = new Joystick(1);
    xboxJoystick = new Joystick(2);

    mDriveSystem = new DriveSystem(true, RoboRioPorts.CAN_DRIVE_L1, RoboRioPorts.CAN_DRIVE_L2,
        RoboRioPorts.CAN_DRIVE_L3, RoboRioPorts.CAN_DRIVE_R1, RoboRioPorts.CAN_DRIVE_R2, RoboRioPorts.CAN_DRIVE_R3,
        RoboRioPorts.DRIVE_DOUBLE_SOLENOID_FWD, RoboRioPorts.DRIVE_DOUBLE_SOLENOID_REV, RoboRioPorts.DIO_DRIVE_RIGHT_A,
        RoboRioPorts.DIO_DRIVE_RIGHT_B, RoboRioPorts.DIO_DRIVE_LEFT_A, RoboRioPorts.DIO_DRIVE_LEFT_B);

    mElevator = new Elevator(RoboRioPorts.CAN_Elevator_1, RoboRioPorts.CAN_Elevator_2, RoboRioPorts.CAN_Elevator_3,
        RoboRioPorts.CAN_Elevator_4);

    mElevatorMM = new ElevatorMMTest(RoboRioPorts.CAN_Elevator_1, RoboRioPorts.CAN_Elevator_2, RoboRioPorts.CAN_Elevator_3,
        RoboRioPorts.CAN_Elevator_4);

    mArm = new Arm(RoboRioPorts.CAN_ARM_RIGHT, RoboRioPorts.CAN_ARM_LEFT);

    mHatch = new HatchMech(RoboRioPorts.HATCH_PISTON_FWD, RoboRioPorts.HATCH_PISTON_REV);

    driveRateGroup = new Notifier(mDriveSystem::operatorDrive);
    liftRateGroup = new Notifier(mElevator::performMainProcessing);
    //liftRateGroup = new Notifier(mElevatorMM::performMainProcessing);
    hatchRateGroup = new Notifier(mHatch::performMainProcessing);
    armRateGroup = new Notifier(mArm::performMainProcessing);

    driveRateGroup.startPeriodic(0.05);
    liftRateGroup.startPeriodic(0.1);
    hatchRateGroup.startPeriodic(0.1);
    armRateGroup.startPeriodic(0.1);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for
   * items like diagnostics that you want ran during disabled, autonomous,
   * teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("XX Pixel", visionTargetInfo.visionPixelX);
    SmartDashboard.putBoolean("XTarget Acquired", (visionTargetInfo.isCargoBayDetected != 0));
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select
   * between different autonomous modes using the dashboard. The sendable chooser
   * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
   * remove all of the chooser code and uncomment the getString line to get the
   * auto name from the text box below the Gyro
   *
   * <p>
   * You can add additional auto modes by adding additional comparisons to the
   * switch structure below with additional strings. If using the SendableChooser
   * make sure to add them to the chooser code above as well.
   */
  @Override
  public void autonomousInit() {

    robotCameraSystem.mainCamera.setExposureHoldCurrent();
    mDriveSystem.methodInit = true;
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    // mDriveSystem.rotateCam(4, visionTargetInfo.visionPixelX);

  }

  /**
   * 
   */
  @Override
  public void teleopInit() {
    isEnabled = true;

    super.teleopInit();
  }

  /**
   * 
   */
  @Override
  public void disabledInit() {
    isEnabled = false;

    super.disabledInit();
  }

  /**
   * 
   */
  @Override
  public void disabledPeriodic() {
    mDriveSystem.updateTelemetry();
    mElevator.updateTelemetry();
    mArm.updateTelemetry();
    mHatch.updateTelemetry();

    super.disabledPeriodic();
  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() {
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {
  }
}
