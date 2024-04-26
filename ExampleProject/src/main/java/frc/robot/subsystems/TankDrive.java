package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.RobotConstants;
import monologue.Annotations.*;
import monologue.Logged;

public class TankDrive extends SubsystemBase implements Logged {

    private CANSparkMax frontLeftMotor;
    private CANSparkMax backLeftMotor;

    private DCMotorSim simLeft;

    private CANSparkMax frontRightMotor;
    private CANSparkMax backRightMotor;

    private DCMotorSim simRight;

    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;

    private SparkPIDController leftPID;
    private SparkPIDController rightPID;

    private PIDController simLeftFeedback;
    private SimpleMotorFeedforward simLeftFeedforward;

    private PIDController simRightFeedback;
    private SimpleMotorFeedforward simRightFeedforward;

    // Monolouge doesn't work with measures as of right now, so we can't use them here
    
    @Log private double leftVelocity = 0;
    @Log private double leftPosition = 0;

    @Log private double rightVelocity = 0;
    @Log private double rightPosition = 0;

    @Log private double leftSetpoint = 0;
    @Log private double rightSetpoint = 0;

    @Log private double leftAppliedVolts = 0;
    @Log private double rightAppliedVolts = 0;

    private final boolean isReal;

    public TankDrive(boolean isReal) {
        this.isReal = isReal;
        if (isReal) {
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

            leftPID.setP(DriveConstants.FEEDBACK_REAL.kP);
            leftPID.setI(DriveConstants.FEEDBACK_REAL.kI);
            leftPID.setD(DriveConstants.FEEDBACK_REAL.kD);
            leftPID.setFF(DriveConstants.FEEDFORWARD_REAL.kV);

            backLeftMotor.follow(frontLeftMotor);

            frontRightMotor =
                    new CANSparkMax(DriveConstants.FRONT_RIGHT_MOTOR, MotorType.kBrushless);
            backRightMotor = new CANSparkMax(DriveConstants.BACK_RIGHT_MOTOR, MotorType.kBrushless);

            frontRightMotor.setInverted(IntakeConstants.MOTOR_CONSTANTS.inverted);
            frontRightMotor.setSmartCurrentLimit(IntakeConstants.MOTOR_CONSTANTS.currentLimit);
            backRightMotor.setInverted(IntakeConstants.MOTOR_CONSTANTS.inverted);
            backRightMotor.setSmartCurrentLimit(IntakeConstants.MOTOR_CONSTANTS.currentLimit);

            rightEncoder = frontRightMotor.getEncoder();

            rightEncoder.setPositionConversionFactor(DriveConstants.WHEEL_CIRCUMFERENCE.in(Meters));
            rightEncoder.setVelocityConversionFactor(DriveConstants.WHEEL_CIRCUMFERENCE.in(Meters));

            rightPID = frontRightMotor.getPIDController();

            rightPID.setP(DriveConstants.FEEDBACK_REAL.kP);
            rightPID.setI(DriveConstants.FEEDBACK_REAL.kI);
            rightPID.setD(DriveConstants.FEEDBACK_REAL.kD);
            rightPID.setFF(DriveConstants.FEEDFORWARD_REAL.kV);

            backRightMotor.follow(frontRightMotor);
        } else {
            simLeft = new DCMotorSim(DCMotor.getNEO(2), 1, 1);
            simRight = new DCMotorSim(DCMotor.getNEO(2), 1, 1);

            simLeftFeedback =
                    new PIDController(
                            DriveConstants.FEEDBACK_SIM.kP,
                            DriveConstants.FEEDBACK_SIM.kI,
                            DriveConstants.FEEDBACK_SIM.kD);
            simLeftFeedforward =
                    new SimpleMotorFeedforward(
                            DriveConstants.FEEDFORWARD_SIM.kS, DriveConstants.FEEDFORWARD_SIM.kV);

            simRightFeedback =
                    new PIDController(
                            DriveConstants.FEEDBACK_SIM.kP,
                            DriveConstants.FEEDBACK_SIM.kI,
                            DriveConstants.FEEDBACK_SIM.kD);
            simRightFeedforward =
                    new SimpleMotorFeedforward(
                            DriveConstants.FEEDFORWARD_SIM.kS, DriveConstants.FEEDFORWARD_SIM.kV);
        }
    }

    @Override
    public void periodic() {
        if (isReal) {
            // Log each motor's current and temperature individually in case of problems with a
            // singluar
            // motor
            this.log("Front Left/Current", frontLeftMotor.getOutputCurrent());
            this.log("Back Left/Current", backLeftMotor.getOutputCurrent());
            this.log("Front Right/Current", frontRightMotor.getOutputCurrent());
            this.log("Back Right/Current", backRightMotor.getOutputCurrent());

            this.log("Front Left/Temperature", frontLeftMotor.getMotorTemperature());
            this.log("Back Left/Temperature", backLeftMotor.getMotorTemperature());
            this.log("Front Right/Temperature", frontRightMotor.getMotorTemperature());
            this.log("Back Right/Temperature", backRightMotor.getMotorTemperature());

            leftAppliedVolts = frontLeftMotor.getAppliedOutput() * frontLeftMotor.getBusVoltage();
            rightAppliedVolts =
                    frontRightMotor.getAppliedOutput() * frontRightMotor.getBusVoltage();

            leftVelocity = leftEncoder.getVelocity();
            rightVelocity = rightEncoder.getVelocity();

            leftPosition = leftEncoder.getPosition();
            rightPosition = rightEncoder.getPosition();

            leftPID.setReference(leftSetpoint, ControlType.kVelocity);
            rightPID.setReference(rightSetpoint, ControlType.kVelocity);
        }
    }

    // Here's how you can do sim
    @Override
    public void simulationPeriodic() {
        simLeft.update(RobotConstants.PERIODIC_LOOP.in(Seconds));
        simRight.update(RobotConstants.PERIODIC_LOOP.in(Seconds));

        this.log("leftCurrent", simLeft.getCurrentDrawAmps());
        this.log("rightCurrent", simRight.getCurrentDrawAmps());

        leftVelocity = simLeft.getAngularVelocityRPM();
        rightVelocity = simRight.getAngularVelocityRPM();

        leftPosition = simLeft.getAngularPositionRotations();
        rightPosition = simLeft.getAngularPositionRotations();

        leftAppliedVolts =
                simLeftFeedback.calculate(leftVelocity, leftSetpoint)
                        + simLeftFeedforward.calculate(leftSetpoint);
        rightAppliedVolts =
                simRightFeedback.calculate(rightVelocity, rightSetpoint)
                        + simRightFeedforward.calculate(rightSetpoint);
    }

    // =========================Velocity=========================

    public void setVelocity(
            Measure<Velocity<Distance>> leftVelocity, Measure<Velocity<Distance>> rightVelocity) {
        leftSetpoint = leftVelocity.in(MetersPerSecond);
        rightSetpoint = rightVelocity.in(MetersPerSecond);
    }

    public double[] getVelocity() {
        return new double[] {leftVelocity, rightVelocity};
    }

    // =========================Position=========================

    public double[] getPosition() {
        return new double[] {leftPosition, rightPosition};
    }
}
