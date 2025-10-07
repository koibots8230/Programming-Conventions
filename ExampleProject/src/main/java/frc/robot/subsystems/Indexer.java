package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

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
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;
import frc.robot.Constants.RobotConstants;

@Logged
public class Indexer extends SubsystemBase {

  @NotLogged private final SparkMax motor;
  @NotLogged private final SparkMaxConfig config;

  @NotLogged private final RelativeEncoder encoder;

  @NotLogged private final SparkClosedLoopController pidController;

  @NotLogged private final DigitalInput distanceSwitch;

  private AngularVelocity setpoint;
  private AngularVelocity velocity;

  private Current current;
  private Voltage appliedVoltage;

  public Indexer() {
    motor = new SparkMax(IndexerConstants.MOTOR_PORT, MotorType.kBrushless);

    config = new SparkMaxConfig();

    config.inverted(IndexerConstants.MOTOR_CONSTANTS.inverted);
    config.smartCurrentLimit(IndexerConstants.MOTOR_CONSTANTS.currentLimit);

    config.voltageCompensation(RobotConstants.NOMINAL_VOLTAGE.in(Volts));

    config.closedLoop.pidf(
        IndexerConstants.PID_GAINS.kp,
        IndexerConstants.PID_GAINS.ki,
        IndexerConstants.PID_GAINS.kd,
        IndexerConstants.FEEDFORWARD_GAINS.kv);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder = motor.getEncoder();

    pidController = motor.getClosedLoopController();

    distanceSwitch = new DigitalInput(IndexerConstants.DISTANCE_SWITCH_PORT);

    setpoint = RPM.of(0);
    velocity = RPM.of(0);

    current = Amps.of(0);
    appliedVoltage = Volts.of(0);
  }

  @Override
  public void periodic() {
    current = Amps.of(motor.getOutputCurrent());
    velocity = RPM.of(encoder.getVelocity());
    appliedVoltage = Volts.of(motor.getAppliedOutput() * motor.getBusVoltage());

    pidController.setReference(setpoint.in(RPM), ControlType.kVelocity);
  }

  // Here's how you can do sim
  @Override
  public void simulationPeriodic() {
    velocity = setpoint;
  }

  private void setVelocity(AngularVelocity velocity) {
    setpoint = velocity;
  }

  public Command intake() {
    return Commands.sequence(
        Commands.runOnce(() -> this.setVelocity(IndexerConstants.INTAKE_SPEED), this),
        Commands.waitUntil(() -> distanceSwitch.get()),
        Commands.runOnce(() -> this.setVelocity(RPM.of(0)), this));
  }

  public Command setVelocityCommand(AngularVelocity velocity) {
    return Commands.runOnce(() -> this.setVelocity(velocity), this);
  }
}
