/**
 * This is a very simple robot program that can be used to send telemetry to
 * the data_logger script to characterize your drivetrain. If you wish to use
 * your actual robot code, you only need to implement the simple logic in the
 * autonomousPeriodic function and change the NetworkTables update rate
 */

package dc;

import java.util.function.Supplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//TODO: Add xbox controller for driving
//TODO: Add this project and PathWeaver project to github
public class Robot extends TimedRobot {
  static private double WHEEL_DIAMETER = 0.1016;

  Joystick stick;
  DifferentialDrive drive;

  CANSparkMax leftMaster;
  CANSparkMax rightMaster;

  final int Left_Encoder_PWM_PORT = 0;
  final int Right_Encoder_PWM_PORT = 1;

  double encoderConstant = WHEEL_DIAMETER * Math.PI;
  VelocityDutyCycleEncoder leftEncoder;
  VelocityDutyCycleEncoder rightEncoder;

  AHRS navx;

  Supplier<Double> leftEncoderPosition;
  Supplier<Double> leftEncoderRate;
  Supplier<Double> rightEncoderPosition;
  Supplier<Double> rightEncoderRate;
  Supplier<Double> gyroAngleRadians;
  Supplier<Double> gyroAngleDegrees;

  NetworkTableEntry autoSpeedEntry =
      NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
  NetworkTableEntry telemetryEntry =
      NetworkTableInstance.getDefault().getEntry("/robot/telemetry");
  NetworkTableEntry rotateEntry =
    NetworkTableInstance.getDefault().getEntry("/robot/rotate");

  double priorAutospeed = 0;
  Number[] numberArray = new Number[10];

  @Override
  public void robotInit() {
    // SimEnabler
    if (!isReal()) SmartDashboard.putData(new SimEnabler());
    // Joystick
    stick = new Joystick(0);

    // Configure masters
    leftMaster = new CANSparkMax(3, MotorType.kBrushless);
    leftMaster.setInverted(false);
    leftMaster.setIdleMode(IdleMode.kBrake);    
    rightMaster = new CANSparkMax(2, MotorType.kBrushless);
    rightMaster.setInverted(false);
    rightMaster.setIdleMode(IdleMode.kBrake);

    // Configure slaves
    CANSparkMax leftSlave0 = new CANSparkMax(4, MotorType.kBrushless);
    leftSlave0.follow(leftMaster);
    leftSlave0.setIdleMode(IdleMode.kBrake);
    CANSparkMax rightSlave0 = new CANSparkMax(5, MotorType.kBrushless);
    rightSlave0.follow(rightMaster);
    rightSlave0.setIdleMode(IdleMode.kBrake);

    // Configure encoder related functions -- getDistance and getrate should
    // return units and units/s
    // Make and set distPerRot for Encoders
    leftEncoder = new VelocityDutyCycleEncoder(Left_Encoder_PWM_PORT);
    rightEncoder = new VelocityDutyCycleEncoder(Right_Encoder_PWM_PORT);
    leftEncoder.setDistancePerRotation(encoderConstant);
    rightEncoder.setDistancePerRotation(encoderConstant);
    // Encoder supplier methods
    leftEncoderPosition = ()
        -> leftEncoder.getDistance();
    leftEncoderRate = ()
        -> leftEncoder.getRate();
    rightEncoderPosition = ()
        -> -rightEncoder.getDistance();
    rightEncoderRate = ()
        -> -rightEncoder.getRate(); 

    // Note that the angle from the NavX and all implementors of wpilib Gyro
    // must be negated because getAngle returns a clockwise positive angle
    // Make the gyro
    navx = new AHRS(SPI.Port.kMXP);
    // Gyro supplier methods
    gyroAngleRadians = () -> -1 * Math.toRadians(navx.getAngle());
    gyroAngleDegrees = () -> -1 * navx.getAngle();

    // Configure drivetrain movement
    drive = new DifferentialDrive(leftMaster, rightMaster);
    drive.setDeadband(0);    

    // Set the update rate instead of using flush because of a ntcore bug
    // -> probably don't want to do this on a robot in competition
    NetworkTableInstance.getDefault().setUpdateRate(0.010);

    resetAllSensors();
  }

  @Override
  public void robotPeriodic() {
    // Push values to smart dashboard to allow user to check sensors
    SmartDashboard.putNumber("Gyro Heading (deg): ", gyroAngleDegrees.get());
    SmartDashboard.putNumber("Left Encoder Distance (m): ", leftEncoderPosition.get());
    SmartDashboard.putNumber("Right Encoder Distance (m): ", rightEncoderPosition.get());
    SmartDashboard.putNumber("Left Encoder Velocity (m/s): ", leftEncoderRate.get());
    SmartDashboard.putNumber("Right Encoder Velocity (m/s): ", rightEncoderRate.get());
  }

  public void resetAllSensors() {
    leftEncoder.reset();
    rightEncoder.reset();
    navx.reset();
  }

  @Override
  public void disabledInit() {
    resetAllSensors();
    System.out.println("Robot disabled");
    drive.tankDrive(0, 0);
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void teleopInit() {
    resetAllSensors();
    System.out.println("Robot in operator control mode");
  }

  @Override
  public void teleopPeriodic() {
    drive.arcadeDrive(-stick.getY(), stick.getX());
  }

  @Override
  public void autonomousInit() {
    resetAllSensors();
    System.out.println("Robot in autonomous mode");
  }

  /**
   * If you wish to just use your own robot program to use with the data logging
   * program, you only need to copy/paste the logic below into your code and
   * ensure it gets called periodically in autonomous mode
   *
   * Additionally, you need to set NetworkTables update rate to 10ms using the
   * setUpdateRate call.
   */
  @Override
  public void autonomousPeriodic() {

    // Retrieve values to send back before telling the motors to do something
    double now = Timer.getFPGATimestamp();

    double leftPosition = leftEncoderPosition.get();
    double leftRate = leftEncoderRate.get();

    double rightPosition = rightEncoderPosition.get();
    double rightRate = rightEncoderRate.get();

    double battery = RobotController.getBatteryVoltage();

    double leftMotorVolts = leftMaster.getBusVoltage() * leftMaster.getAppliedOutput();
    double rightMotorVolts = rightMaster.getBusVoltage() * rightMaster.getAppliedOutput();

    // Retrieve the commanded speed from NetworkTables
    double autospeed = autoSpeedEntry.getDouble(0);
    priorAutospeed = autospeed;

    // command motors to do things
    drive.tankDrive(
      (rotateEntry.getBoolean(false) ? -1 : 1) * autospeed, autospeed,
      false
    );

    // send telemetry data array back to NT
    numberArray[0] = now;
    numberArray[1] = battery;
    numberArray[2] = autospeed;
    numberArray[3] = leftMotorVolts;
    numberArray[4] = rightMotorVolts;
    numberArray[5] = leftPosition;
    numberArray[6] = rightPosition;
    numberArray[7] = leftRate;
    numberArray[8] = rightRate;
    numberArray[9] = gyroAngleRadians.get();

    telemetryEntry.setNumberArray(numberArray);
  }
}
