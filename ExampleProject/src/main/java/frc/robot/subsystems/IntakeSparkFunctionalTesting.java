package frc.robot.subsystems;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import monologue.Annotations.Log;
import monologue.Logged;


/**
 * This example is a simple velocity control with no logging.
 * The velocity control is passed to the SpakrMax.
 * Running velocity control on the SparkMax provides more consistent intake velocity regardless of battery consumption.
 * Intakes are a low inertia system so a P value is sufficient for PID control.
 * 
 * Functional testing:
 * 1 - The target velocity should update when setVelocity is called
 * 2 - The target velocity should update to the value requested in setVelocity
 * 3 - The motor velocity should accelerate to and hold steady at the target velocity
 */

public class IntakeSparkFunctionalTesting extends SubsystemBase {

    private final CANSparkMax motor;
    private final SparkPIDController pidController;
    private final RelativeEncoder encoder;

    private Measure<Velocity<Angle>> targetVelocity;

    /**
     * Set up data for logging.
     * Target velocity is duplicative for logging because we are using Measure types 
     * for state and Monologue needs primitive types
    */
    @Log private double targetVelocityRPM;
    @Log private double currentVelocityRPM;

    public IntakeSparkFunctionalTesting() {
        motor = new CANSparkMax(IntakeConstants.MOTOR_PORT, MotorType.kBrushless);

        motor.setInverted(IntakeConstants.MOTOR_CONSTANTS.inverted);
        motor.setSmartCurrentLimit(IntakeConstants.MOTOR_CONSTANTS.currentLimit);

        pidController = motor.getPIDController();
        pidController.setP(IntakeConstants.MOTOR_PID.kP);

        encoder = motor.getEncoder();

    }

    @Override
    public void periodic() {
        pidController.setReference(targetVelocity.in(RPM), ControlType.kVelocity);
        // Log the current velocity
        currentVelocityRPM = encoder.getVelocity();
    }

    // =========================Velocity=========================

    public void setVelocity(Measure<Velocity<Angle>> targetVelocity) {
        this.targetVelocity = targetVelocity;
        // Convert the Measure type into RPM for logging
        this.targetVelocityRPM = targetVelocity.in(RPM);
    }

}
