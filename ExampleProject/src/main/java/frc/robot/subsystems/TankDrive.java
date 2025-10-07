package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotConstants;
import java.util.function.DoubleSupplier;

@Logged
public class TankDrive extends SubsystemBase {

  // Same stuff, just with two sets of two motors being controlled,
  // as well as a gyro and odometry

  @NotLogged private final SparkMax frontLeftMotor;
  @NotLogged private final SparkMax backLeftMotor;

  @NotLogged private final SparkMax frontRightMotor;
  @NotLogged private final SparkMax backRightMotor;

  @NotLogged private final SparkMaxConfig frontLeftConfig;
  @NotLogged private final SparkMaxConfig backLeftConfig;

  @NotLogged private final SparkMaxConfig frontRightConfig;
  @NotLogged private final SparkMaxConfig backRightConfig;

  @NotLogged private final RelativeEncoder leftEncoder;
  @NotLogged private final RelativeEncoder rightEncoder;

  @NotLogged private final SparkClosedLoopController leftPID;
  @NotLogged private final SparkClosedLoopController rightPID;

  @NotLogged private final Pigeon2 gyro;

  @NotLogged private final DifferentialDriveOdometry odometry;

  private Pose2d estimatedPose;

  private Current frontLeftCurrent;
  private Current backLeftCurrent;
  private Current frontRightCurrent;
  private Current backRightCurrent;

  private LinearVelocity leftVelocity;
  private Distance leftPosition;

  private LinearVelocity rightVelocity;
  private Distance rightPosition;

  private LinearVelocity leftSetpoint;
  private LinearVelocity rightSetpoint;

  private Voltage leftAppliedVolts;
  private Voltage rightAppliedVolts;

  @NotLogged private Distance simLeftPosition;
  @NotLogged private Distance simRightPosition;

  public TankDrive() {
    frontLeftMotor = new SparkMax(DriveConstants.FRONT_LEFT_MOTOR, MotorType.kBrushless);
    backLeftMotor = new SparkMax(DriveConstants.BACK_LEFT_MOTOR, MotorType.kBrushless);

    frontLeftConfig = new SparkMaxConfig();
    backLeftConfig = new SparkMaxConfig();

    frontLeftConfig.inverted(DriveConstants.FRONT_LEFT_MOTOR_CONSTANTS.inverted);
    frontLeftConfig.smartCurrentLimit(DriveConstants.FRONT_LEFT_MOTOR_CONSTANTS.currentLimit);

    backLeftConfig.inverted(DriveConstants.BACK_LEFT_MOTOR_CONSTANTS.inverted);
    backLeftConfig.smartCurrentLimit(DriveConstants.BACK_LEFT_MOTOR_CONSTANTS.currentLimit);

    frontLeftConfig.voltageCompensation(RobotConstants.NOMINAL_VOLTAGE.in(Volts));
    backLeftConfig.voltageCompensation(RobotConstants.NOMINAL_VOLTAGE.in(Volts));

    backLeftConfig.follow(frontLeftMotor);

    frontLeftConfig.encoder.positionConversionFactor(DriveConstants.WHEEL_CIRCUMFERENCE.in(Meters));
    frontLeftConfig.encoder.velocityConversionFactor(
        DriveConstants.WHEEL_CIRCUMFERENCE.in(Meters) / 60.0);

    // Note how only the leader gets pid set
    frontLeftConfig.closedLoop.pidf(
        DriveConstants.PID_GAINS.kp,
        DriveConstants.PID_GAINS.ki,
        DriveConstants.PID_GAINS.kd,
        DriveConstants.FEEDFORWARD_GAINS.kv);

    frontLeftMotor.configure(
        frontLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    backLeftMotor.configure(
        backLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    leftEncoder = frontLeftMotor.getEncoder();

    leftPID = frontLeftMotor.getClosedLoopController();

    frontRightMotor = new SparkMax(DriveConstants.FRONT_RIGHT_MOTOR, MotorType.kBrushless);
    backRightMotor = new SparkMax(DriveConstants.BACK_RIGHT_MOTOR, MotorType.kBrushless);

    frontRightConfig = new SparkMaxConfig();
    backRightConfig = new SparkMaxConfig();

    frontRightConfig.inverted(DriveConstants.FRONT_RIGHT_MOTOR_CONSTANTS.inverted);
    frontRightConfig.smartCurrentLimit(DriveConstants.FRONT_RIGHT_MOTOR_CONSTANTS.currentLimit);

    backRightConfig.inverted(DriveConstants.BACK_RIGHT_MOTOR_CONSTANTS.inverted);
    backRightConfig.smartCurrentLimit(DriveConstants.BACK_RIGHT_MOTOR_CONSTANTS.currentLimit);

    frontRightConfig.voltageCompensation(RobotConstants.NOMINAL_VOLTAGE.in(Volts));
    backRightConfig.voltageCompensation(RobotConstants.NOMINAL_VOLTAGE.in(Volts));

    backRightConfig.follow(frontLeftMotor);

    frontRightConfig.encoder.positionConversionFactor(
        DriveConstants.WHEEL_CIRCUMFERENCE.in(Meters));
    frontRightConfig.encoder.velocityConversionFactor(
        DriveConstants.WHEEL_CIRCUMFERENCE.in(Meters) / 60.0);

    frontRightConfig.closedLoop.pidf(
        DriveConstants.PID_GAINS.kp,
        DriveConstants.PID_GAINS.ki,
        DriveConstants.PID_GAINS.kd,
        DriveConstants.FEEDFORWARD_GAINS.kv);

    frontRightMotor.configure(
        frontRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    backRightMotor.configure(
        backRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    rightEncoder = frontRightMotor.getEncoder();

    rightPID = frontRightMotor.getClosedLoopController();

    gyro = new Pigeon2(DriveConstants.GYRO_PORT);

    odometry = new DifferentialDriveOdometry(new Rotation2d(), Meters.of(0), Meters.of(0));

    frontLeftCurrent = Amps.of(0);
    backLeftCurrent = Amps.of(0);
    frontRightCurrent = Amps.of(0);
    backRightCurrent = Amps.of(0);

    leftSetpoint = MetersPerSecond.of(0);
    leftVelocity = MetersPerSecond.of(0);

    rightSetpoint = MetersPerSecond.of(0);
    rightVelocity = MetersPerSecond.of(0);

    leftPosition = Meters.of(0);
    rightPosition = Meters.of(0);

    leftAppliedVolts = Volts.of(0);
    rightAppliedVolts = Volts.of(0);

    simLeftPosition = Meters.of(0);
    simRightPosition = Meters.of(0);
  }

  @Override
  public void periodic() {
    estimatedPose =
        odometry.update(gyro.getRotation2d(), leftPosition.in(Meters), rightPosition.in(Meters));

    frontLeftCurrent = Amps.of(frontLeftMotor.getOutputCurrent());
    backLeftCurrent = Amps.of(backLeftMotor.getOutputCurrent());
    frontRightCurrent = Amps.of(frontRightMotor.getOutputCurrent());
    backRightCurrent = Amps.of(backRightMotor.getOutputCurrent());

    leftAppliedVolts = Volts.of(frontLeftMotor.getAppliedOutput() * frontLeftMotor.getBusVoltage());
    rightAppliedVolts =
        Volts.of(frontRightMotor.getAppliedOutput() * frontRightMotor.getBusVoltage());

    leftVelocity = MetersPerSecond.of(leftEncoder.getVelocity());
    rightVelocity = MetersPerSecond.of(rightEncoder.getVelocity());

    leftPosition = Meters.of(leftEncoder.getPosition());
    rightPosition = Meters.of(rightEncoder.getPosition());

    leftPID.setReference(leftSetpoint.in(MetersPerSecond), ControlType.kVelocity);
    rightPID.setReference(rightSetpoint.in(MetersPerSecond), ControlType.kVelocity);
  }

  // Here's how you can do sim
  @Override
  public void simulationPeriodic() {
    leftVelocity = leftSetpoint;
    rightVelocity = rightSetpoint;

    simLeftPosition = simLeftPosition.plus(leftVelocity.times(RobotConstants.LOOP_TIME));
    simRightPosition = simRightPosition.plus(rightVelocity.times(RobotConstants.LOOP_TIME));

    leftPosition = simLeftPosition;
    rightPosition = simRightPosition;
  }

  private void setVelocity(LinearVelocity leftVelocity, LinearVelocity rightVelocity) {
    leftSetpoint = leftVelocity;
    rightSetpoint = rightVelocity;
  }

  public Command driveCommand(DoubleSupplier leftStick, DoubleSupplier rightStick) {
    return Commands.run(
        () ->
            this.setVelocity(
                DriveConstants.MAX_VELOCITY.times(
                    MathUtil.applyDeadband(
                        leftStick.getAsDouble(), RobotConstants.JOYSTICK_DEADZONE)),
                DriveConstants.MAX_VELOCITY.times(
                    MathUtil.applyDeadband(
                        rightStick.getAsDouble(), RobotConstants.JOYSTICK_DEADZONE))),
        this);
  }
}
