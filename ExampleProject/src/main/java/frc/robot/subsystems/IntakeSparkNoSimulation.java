package frc.robot.subsystems;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;


import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;


/**
 * This example is a simple velocity control with no logging.
 * The velocity control is passed to the SpakrMax.
 * Running velocity control on the SparkMax provides more consistent intake velocity regardless of battery consumption.
 * Intakes are a low inertia system so a P value is sufficient for PID control.
 */

public class IntakeSparkNoSimulation extends SubsystemBase {

    private final CANSparkMax motor;
    private final SparkPIDController pidController;

    private Measure<Velocity<Angle>> targetVelocity;

    public IntakeSparkNoSimulation() {
        motor = new CANSparkMax(IntakeConstants.MOTOR_PORT, MotorType.kBrushless);

        motor.setInverted(IntakeConstants.MOTOR_CONSTANTS.inverted);
        motor.setSmartCurrentLimit(IntakeConstants.MOTOR_CONSTANTS.currentLimit);

        pidController = motor.getPIDController();
        pidController.setP(IntakeConstants.MOTOR_PID.kP);

    }

    @Override
    public void periodic() {
        pidController.setReference(targetVelocity.in(RPM), ControlType.kVelocity);
    }

    // =========================Velocity=========================

    public void setVelocity(Measure<Velocity<Angle>> targetVelocity) {
        this.targetVelocity = targetVelocity;
    }

}
