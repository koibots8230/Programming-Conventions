package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.units.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import monologue.Annotations.Log;
import monologue.Logged;

public class Intake extends SubsystemBase implements Logged {

    private final CANSparkMax motor;

    private final RelativeEncoder encoder;

    private final SparkPIDController pidController;

    @Log private double current = 0;
    @Log private double velocity = 0;
    @Log private double appliedVoltage = 0;

    public Intake() {
        motor = new CANSparkMax(IntakeConstants.MOTOR_PORT, MotorType.kBrushless);

        motor.setInverted(IntakeConstants.MOTOR_CONSTANTS.inverted);
        motor.setSmartCurrentLimit(IntakeConstants.MOTOR_CONSTANTS.currentLimit);

        encoder = motor.getEncoder();

        pidController = motor.getPIDController();
    }

    @Override
    public void periodic() {
        current = motor.getOutputCurrent();
        velocity = this.getVelocity();
        appliedVoltage = motor.getAppliedOutput() * motor.getBusVoltage();
    }

    // =========================Velocity=========================

    public void setVelocity(Measure<Velocity<Angle>> velocity) {
        pidController.setReference(velocity.in(RPM), ControlType.kVelocity);
    }

    public double getVelocity() {
        return encoder.getVelocity();
    }
}
