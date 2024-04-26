package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Seconds;

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
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.RobotConstants;
import monologue.Annotations.Log;
import monologue.Logged;

public class Intake extends SubsystemBase implements Logged {

    private CANSparkMax motor;
    private DCMotorSim simMotor;

    private RelativeEncoder encoder;

    private SparkPIDController pidController;

    private PIDController simFeedback;
    private SimpleMotorFeedforward simFeedforward;

    // Monolouge doesn't work with measures as of right now, so we can't use them here

    @Log private double current = 0;
    @Log private double velocity = 0;
    @Log private double appliedVoltage = 0;
    @Log private double setpoint = 0;

    private final boolean isReal;

    // Note that the value of isReal gets passed into intake, it doesn't get it itself (would create
    // a circular dependency, intake requires robot requires robotcontainer requires intake)
    public Intake(boolean isReal) {
        this.isReal = isReal;
        if (isReal) {
            motor = new CANSparkMax(IntakeConstants.MOTOR_PORT, MotorType.kBrushless);

            motor.setInverted(IntakeConstants.MOTOR_CONSTANTS.inverted);
            motor.setSmartCurrentLimit(IntakeConstants.MOTOR_CONSTANTS.currentLimit);

            encoder = motor.getEncoder();

            pidController = motor.getPIDController();
        } else {
            simMotor = new DCMotorSim(DCMotor.getNEO(1), 1, 1);

            simFeedback =
                    new PIDController(
                            IntakeConstants.FEEDBACK_SIM.kP,
                            IntakeConstants.FEEDBACK_SIM.kI,
                            IntakeConstants.FEEDBACK_SIM.kD);
            simFeedforward =
                    new SimpleMotorFeedforward(
                            IntakeConstants.FEEDFORWARD_SIM.kS, IntakeConstants.FEEDFORWARD_SIM.kV);
        }
    }

    @Override
    public void periodic() {
        if (isReal) {
            current = motor.getOutputCurrent();
            velocity = this.getVelocity();
            appliedVoltage = motor.getAppliedOutput() * motor.getBusVoltage();

            pidController.setReference(setpoint, ControlType.kVelocity);
        }
    }

    // Here's how you can do sim
    @Override
    public void simulationPeriodic() {
        simMotor.update(RobotConstants.PERIODIC_LOOP.in(Seconds));

        current = simMotor.getCurrentDrawAmps();
        velocity = simMotor.getAngularVelocityRPM();
        appliedVoltage =
                simFeedback.calculate(velocity, setpoint) + simFeedforward.calculate(setpoint);

        simMotor.setInputVoltage(appliedVoltage);
    }

    // =========================Velocity=========================

    public void setVelocity(Measure<Velocity<Angle>> velocity) {
        setpoint = velocity.in(RPM);
    }

    public double getVelocity() {
        return velocity;
    }
}
