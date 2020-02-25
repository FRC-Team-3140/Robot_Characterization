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

public class Robot extends TimedRobot {
  Joystick stick;
  DifferentialDrive drive;

  CANSparkMax leftMaster;
  CANSparkMax rightMaster;

  AHRS navx;

  Supplier<Double> angularPosition;
  Supplier<Double> angularRate;

  NetworkTableEntry autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
  NetworkTableEntry telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry");

  double priorAutospeed = 0;
  Number[] numberArray = new Number[6];


  @Override
  public void robotInit() {
    // SimEnabler
    if (!isReal()) SmartDashboard.putData(new SimEnabler());
    // Joystick
    stick = new Joystick(0);

    // Left and/or Right Sides Inverted
    boolean leftInverted = true;
    boolean rightInverted = false;

    // Configure masters
    leftMaster = new CANSparkMax(3, MotorType.kBrushless);
    leftMaster.setInverted(leftInverted);
    leftMaster.setIdleMode(IdleMode.kBrake);
    rightMaster = new CANSparkMax(2, MotorType.kBrushless);
    rightMaster.setInverted(rightInverted);
    rightMaster.setIdleMode(IdleMode.kBrake);

    // Configure slaves
    CANSparkMax leftSlave1 = new CANSparkMax(4, MotorType.kBrushless);
    leftSlave1.follow(leftMaster);
    leftSlave1.setIdleMode(IdleMode.kBrake);
    leftSlave1.setInverted(leftInverted);
    CANSparkMax rightSlave1 = new CANSparkMax(5, MotorType.kBrushless);
    rightSlave1.follow(rightMaster);
    rightSlave1.setIdleMode(IdleMode.kBrake);
    rightSlave1.setInverted(rightInverted);
    CANSparkMax leftSlave2 = new CANSparkMax(6, MotorType.kBrushless);
    leftSlave2.follow(leftMaster);
    leftSlave2.setIdleMode(IdleMode.kBrake);
    leftSlave2.setInverted(leftInverted);
    CANSparkMax rightSlave2 = new CANSparkMax(7, MotorType.kBrushless);
    rightSlave2.follow(rightMaster);
    rightSlave2.setIdleMode(IdleMode.kBrake);
    rightSlave2.setInverted(rightInverted);

    // Note that the angle from the NavX and all implementors of wpilib Gyro
    // must be negated because getAngle returns a clockwise positive angle
    // Make the Gyro
    navx = new AHRS(SPI.Port.kMXP);
    //
    // Configure gyro related functions -- angularPosition and angularRate should return
    // degrees and degrees/sec both (CCWP, Counter clockwise positive; -180deg. to 0deg. to 180deg.)
    //
    // Gyro supplier methods
    angularPosition = () -> -1 * navx.getYaw();
    angularRate = () -> -1 * navx.getRate();

    // Configure drivetrain movement
    drive = new DifferentialDrive(leftMaster, rightMaster);
    drive.setDeadband(0);

    // Set the update rate instead of using flush because of a ntcore bug
    // -> probably don't want to do this on a robot in competition
    NetworkTableInstance.getDefault().setUpdateRate(0.010);
  }

  @Override
  public void robotPeriodic() {
    // Push values to smart dashboard to allow user to check sensors
    SmartDashboard.putNumber("Gyro Heading (deg): ", gyroAngleDegrees.get());
  }

  public void resetAllSensors() {
    navx.reset();
  }


  @Override
  public void disabledInit() {
    resetAllSensors();
    System.out.println("Robot disabled");
    armMotor.set(0);
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
    drive.arcadeDrive(0, stick.getX());
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

    double position = angularPosition.get();
    double rate = angularRate.get();

    double battery = RobotController.getBatteryVoltage();
    double motorVolts = leftMaster.getBusVoltage() * leftMaster.getAppliedOutput();

    // Retrieve the commanded speed from NetworkTables
    double autospeed = autoSpeedEntry.getDouble(0);
    priorAutospeed = autospeed;

    // Command motors to do things
    drive.tankDrive(0, autospeed, false);

    // Send telemetry data array back to NT
    numberArray[0] = now;
    numberArray[1] = battery;
    numberArray[2] = autospeed;
    numberArray[3] = motorVolts;
    numberArray[4] = position;
    numberArray[5] = rate;
    telemetryEntry.setNumberArray(numberArray);
  }
}