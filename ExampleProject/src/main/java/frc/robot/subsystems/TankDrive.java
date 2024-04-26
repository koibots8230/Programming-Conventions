package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;

import monologue.Annotations.*;
import monologue.Logged;

public class TankDrive extends SubsystemBase implements Logged {

    private final CANSparkMax frontLeftMotor;
    private final CANSparkMax backLeftMotor;

    private final CANSparkMax frontRightMotor;
    private final CANSparkMax backRightMotor;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    private final SparkPIDController leftPID;
    private final SparkPIDController rightPID;

    @Log private double leftVelocity = 0;
    @Log private double leftPosition = 0;

    @Log private double rightVelocity = 0;
    @Log private double rightPosition = 0;

    public TankDrive() {
        frontLeftMotor = new CANSparkMax(DriveConstants.FRONT_LEFT_MOTOR, MotorType.kBrushless);
        backLeftMotor = new CANSparkMax(DriveConstants.BACK_LEFT_MOTOR, MotorType.kBrushless);

        frontLeftMotor.setInverted(IntakeConstants.MOTOR_CONSTANTS.inverted);
        frontLeftMotor.setSmartCurrentLimit(IntakeConstants.MOTOR_CONSTANTS.currentLimit);
        backLeftMotor.setInverted(IntakeConstants.MOTOR_CONSTANTS.inverted);
        backLeftMotor.setSmartCurrentLimit(IntakeConstants.MOTOR_CONSTANTS.currentLimit);

        leftEncoder = frontLeftMotor.getEncoder();

        leftEncoder.setPositionConversionFactor(DriveConstants.WHEEL_CIRCUMFERENCE.in(Meters));
        leftEncoder.setVelocityConversionFactor(DriveConstants.WHEEL_CIRCUMFERENCE.in(Meters));

        leftPID = frontLeftMotor.getPIDController();

        leftPID.setP(DriveConstants.FEEDBACK.kP);
        leftPID.setD(DriveConstants.FEEDBACK.kD);
        leftPID.setFF(DriveConstants.FEEDFORWARD.kV);

        backLeftMotor.follow(frontLeftMotor);

        frontRightMotor = new CANSparkMax(DriveConstants.FRONT_RIGHT_MOTOR, MotorType.kBrushless);
        backRightMotor = new CANSparkMax(DriveConstants.BACK_RIGHT_MOTOR, MotorType.kBrushless);

        frontRightMotor.setInverted(IntakeConstants.MOTOR_CONSTANTS.inverted);
        frontRightMotor.setSmartCurrentLimit(IntakeConstants.MOTOR_CONSTANTS.currentLimit);
        backRightMotor.setInverted(IntakeConstants.MOTOR_CONSTANTS.inverted);
        backRightMotor.setSmartCurrentLimit(IntakeConstants.MOTOR_CONSTANTS.currentLimit);

        rightEncoder = frontRightMotor.getEncoder();

        rightEncoder.setPositionConversionFactor(DriveConstants.WHEEL_CIRCUMFERENCE.in(Meters));
        rightEncoder.setVelocityConversionFactor(DriveConstants.WHEEL_CIRCUMFERENCE.in(Meters));

        rightPID = frontRightMotor.getPIDController();

        rightPID.setP(DriveConstants.FEEDBACK.kP);
        rightPID.setD(DriveConstants.FEEDBACK.kD);
        rightPID.setFF(DriveConstants.FEEDFORWARD.kV);

        backRightMotor.follow(frontRightMotor);
    }

    @Override
    public void periodic() {
        this.log("Front Left/Current", frontLeftMotor.getOutputCurrent());
        this.log("Back Left/Current", backLeftMotor.getOutputCurrent());
        this.log("Front Right/Current", frontRightMotor.getOutputCurrent());
        this.log("Back Right/Current", backRightMotor.getOutputCurrent());

        this.log("Front Left/Temperature", frontLeftMotor.getMotorTemperature());
        this.log("Back Left/Temperature", backLeftMotor.getMotorTemperature());
        this.log("Front Right/Temperature", frontRightMotor.getMotorTemperature());
        this.log("Back Right/Temperature", backRightMotor.getMotorTemperature());

        this.log("Front Left/Applied Voltage", frontLeftMotor.getAppliedOutput() * frontLeftMotor.getBusVoltage());
        this.log("Back Left/Applied Voltage", backLeftMotor.getAppliedOutput() * backLeftMotor.getBusVoltage());
        this.log("Front Right/Applied Voltage", frontRightMotor.getAppliedOutput() * frontRightMotor.getBusVoltage());
        this.log("Back Right/Applied Voltage", backRightMotor.getAppliedOutput() * backRightMotor.getBusVoltage());
        
        leftVelocity = this.getVelocity()[0];
        rightVelocity = this.getVelocity()[1];

        leftPosition = this.getPosition()[0];
        rightPosition = this.getPosition()[1];
    }

    // =========================Velocity=========================

    public void setVelocity(
            Measure<Velocity<Distance>> leftVelocity, Measure<Velocity<Distance>> rightVelocity) {
        leftPID.setReference(leftVelocity.in(MetersPerSecond), ControlType.kVelocity);
        rightPID.setReference(rightVelocity.in(MetersPerSecond), ControlType.kVelocity);
    }

    public double[] getVelocity() {
        return new double[] {leftEncoder.getVelocity(), rightEncoder.getVelocity()};
    }

    // =========================Position=========================

    public double[] getPosition() {
        return new double[] {leftEncoder.getPosition(), rightEncoder.getPosition()};
    }
}
