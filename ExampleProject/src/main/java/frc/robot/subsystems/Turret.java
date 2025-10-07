package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.TurretConstants;

@Logged
public class Turret extends SubsystemBase {

  @NotLogged private final SparkMax motor;
  @NotLogged private final SparkMaxConfig config;

  @NotLogged private final AbsoluteEncoder encoder;

  @NotLogged private final SparkClosedLoopController controller;

  @NotLogged private final SimpleMotorFeedforward feedforward;

  @NotLogged private final TrapezoidProfile profile;
  @NotLogged private TrapezoidProfile.State goalState;
  @NotLogged private TrapezoidProfile.State motorSetpoint;

  private Angle setpoint;
  private Angle position;

  private AngularVelocity velocity;

  private Current current;
  private Voltage appliedVoltage;

  public Turret() {
    motor = new SparkMax(TurretConstants.MOTOR_PORT, MotorType.kBrushless);

    config = new SparkMaxConfig();

    config.inverted(TurretConstants.MOTOR_CONSTANTS.inverted);
    config.smartCurrentLimit(TurretConstants.MOTOR_CONSTANTS.currentLimit);

    config.voltageCompensation(RobotConstants.NOMINAL_VOLTAGE.in(Volts));

    config.absoluteEncoder.positionConversionFactor(TurretConstants.CONVERSION_FACTOR);

    /* Something to note here is that I'm dividing by 60, that's because the default velocity unit is rotations
    per minute, whereas I want radians per second, so dividing by 60 converts those minutes to seconds */
    config.absoluteEncoder.velocityConversionFactor(TurretConstants.CONVERSION_FACTOR / 60.0);

    // Note how I don't set FF here
    config.closedLoop.pid(
        TurretConstants.PID_GAINS.kp, TurretConstants.PID_GAINS.ki, TurretConstants.PID_GAINS.kd);

    config.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);

    motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    encoder = motor.getAbsoluteEncoder();

    controller = motor.getClosedLoopController();

    // Seperate feedforward, use the appropritate one (if elevator use elevator ff, arm use arm ff)
    feedforward =
        new SimpleMotorFeedforward(
            TurretConstants.FEEDFORWARD_GAINS.ks, TurretConstants.FEEDFORWARD_GAINS.kv);

    /* Think of a trapezoidal profile as an optimized path for the motor to follow. In order to make the path for the motor,
    it needs to know the motor's limits, specifically how fast it can go and how much it can accelerate.*/
    profile =
        new TrapezoidProfile(
            new Constraints(
                TurretConstants.MAX_VELOCITY.in(RadiansPerSecond),
                TurretConstants.MAX_ACCELERATION.in(RadiansPerSecondPerSecond)));

    // So we have our path. However, we need to specify a destination. That's what goal state is,
    // its where we want to end up.
    goalState = new State(0, 0);

    // Think of motor setpoint as the steps along that path. It gets updated constantly by the
    // feedforward
    motorSetpoint = new State(0, 0);

    setpoint = Radians.of(0);
    position = Radians.of(0);

    velocity = RadiansPerSecond.of(0);

    current = Amps.of(0);
    appliedVoltage = Volts.of(0);
  }

  @Override
  public void periodic() {
    /* Takes three things (im listing them in reverse):
    - The goal, where we want to be (goalState)
    - The "current" state, but since it's a feedforward, we assume it follows perfectly, so we just give
        it the previous step along the path (motorSetpoint)
    - How much time to advance ahead along the profile (the robot's loop time, since its getting called every loop) */
    motorSetpoint =
        profile.calculate(RobotConstants.LOOP_TIME.in(Seconds), motorSetpoint, goalState);

    /* Instead of just set and forget, since we're running the profile on the rio (can't on sparkmax), we've
    got to update the controller every cycle. This is two-part, we've got the PID, which we just give the motorSetpoint's
    position, and the feedforward, which in this case only takes a velocity, so we give it the motorSetpoint's velocity,
    and give it to the sparkmax to add onto the PID

    The control type just tells the sparkmax what its controlling for, and the closed loop slot just leave as 0, REV is weird*/
    controller.setReference(
        motorSetpoint.position,
        ControlType.kPosition,
        ClosedLoopSlot.kSlot0,
        feedforward.calculate(motorSetpoint.velocity));

    // Logged stuff
    position = Radians.of(encoder.getPosition());
    velocity = RadiansPerSecond.of(encoder.getVelocity());

    current = Amps.of(motor.getOutputCurrent());
    appliedVoltage = Volts.of(motor.getAppliedOutput() * motor.getBusVoltage());
  }

  @Override
  public void simulationPeriodic() {
    position = Radians.of(motorSetpoint.position);
    velocity = RadiansPerSecond.of(motorSetpoint.velocity);
  }

  private void setPosition(Angle angle) {
    setpoint = angle;

    /* States have two components: position and velocity. We want it to go to a specified position
    and stay there (aka no velocity), so our goal is the position with 0 velocity */
    goalState = new State(angle.in(Radians), 0);
  }

  public Command setPositionCommand(Angle angle) {
    return Commands.runOnce(() -> this.setPosition(angle), this);
  }
}
