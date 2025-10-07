package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.RobotConstants;

@Logged
public class Intake extends SubsystemBase {

  // Note that theres an @NotLogged before stuff that doesn't get logged
  // This is so it doesn't spam warnings that it cant log stuff

  // Also note that everything is final
  @NotLogged private final SparkMax motor;
  @NotLogged private final SparkMaxConfig config;

  @NotLogged private final RelativeEncoder encoder;

  @NotLogged private final SparkClosedLoopController controller;

  private AngularVelocity setpoint;
  private AngularVelocity velocity;

  private Current current;
  private Voltage appliedVoltage;

  public Intake() {
    motor =
        new SparkMax(
            IntakeConstants.MOTOR_PORT, com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless);

    config = new SparkMaxConfig();

    config.inverted(IntakeConstants.MOTOR_CONSTANTS.inverted);
    config.smartCurrentLimit(IntakeConstants.MOTOR_CONSTANTS.currentLimit);

    config.voltageCompensation(RobotConstants.NOMINAL_VOLTAGE.in(Volts));

    config.closedLoop.pidf(
        IntakeConstants.PID_GAINS.kp,
        IntakeConstants.PID_GAINS.ki,
        IntakeConstants.PID_GAINS.kd,
        IntakeConstants.FEEDFORWARD_GAINS.kv);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder = motor.getEncoder();

    controller = motor.getClosedLoopController();

    // Initialize logged values
    setpoint = RPM.of(0);
    velocity = RPM.of(0);

    current = Amps.of(0);
    appliedVoltage = Volts.of(0);
  }

  @Override
  public void periodic() {
    // This is how you set mutables
    velocity = RPM.of(encoder.getVelocity());

    current = Amps.of(motor.getOutputCurrent());
    appliedVoltage = Volts.of(motor.getAppliedOutput() * motor.getBusVoltage());
  }

  // Here's how you can do basic sim
  @Override
  public void simulationPeriodic() {
    velocity = setpoint;
  }

  // Note how this method is private, only used via the command
  private void setVelocity(AngularVelocity velocity) {
    // For velocity, the control is set and forget, the sparkmax takes care of the rest
    controller.setReference(velocity.in(RPM), ControlType.kVelocity);

    setpoint = velocity;
  }

  public Command setVelocityCommand(AngularVelocity velocity) {
    return Commands.runOnce(() -> this.setVelocity(velocity), this);
  }
}
