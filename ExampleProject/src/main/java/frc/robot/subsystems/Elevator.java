package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meters;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants.ElevatorConstants;
import monologue.Annotations.Log;
import monologue.Logged;

public class Elevator extends TrapezoidProfileSubsystem  implements Logged {

    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;
    private final AbsoluteEncoder elevatorEncoder;
    private final SparkPIDController pidController;
    private final ElevatorFeedforward elevatorFeedForward;

    /**
     * Set up logging
     * targetHeight is the target position computed by the TrapezoidProfile
     * currentHeight is the measured encoder position
     */
    @Log double currentHeight;
    @Log double targetHeight;

    /** Create a new ArmSubsystem. */
    public Elevator() {
        super(
            new TrapezoidProfile.Constraints(ElevatorConstants.kMaxVelocityRadPerSecond, ElevatorConstants.kMaxAccelerationRadPerSecSquared),
                ElevatorConstants.kArmOffsetRads);

        leftMotor = new CANSparkMax(ElevatorConstants.LEFT_MOTOR_PORT, MotorType.kBrushless);
        rightMotor = new CANSparkMax(ElevatorConstants.RIGHT_MOTOR_PORT, MotorType.kBrushless);
        // Do these motors run together or inverted? set invert?
        rightMotor.follow(leftMotor);

        elevatorEncoder = leftMotor.getAbsoluteEncoder();
        // Set the conversion factor so that: encoder ticks * conversion factor = elevator position in meters
        elevatorEncoder.setPositionConversionFactor(0.10);
        // Assert that elevator is fully stowed when powering up robot or restarting code
        elevatorEncoder.setZeroOffset(elevatorEncoder.getPosition());
        currentHeight = targetHeight = 0.0;

        // Run the PID Controll off of the Elevator Encoder, not the internal motor encoder
        pidController = leftMotor.getPIDController();
        pidController.setFeedbackDevice(elevatorEncoder);

        /**
         * This elevator is relatively light weight for two NEOs
         * The feed forward and P gain should suffice to have good control
         * Use https://www.reca.lc/linear to estimate kg, kv, ka 
         */
        pidController.setP(ElevatorConstants.PID.kP);

        elevatorFeedForward =
        new ElevatorFeedforward(
            ElevatorConstants.FEEDFORWARD_GAINS.ks,
            ElevatorConstants.FEEDFORWARD_GAINS.kg,
            ElevatorConstants.FEEDFORWARD_GAINS.kv,
            ElevatorConstants.FEEDFORWARD_GAINS.ka);
    }

    /**
     * Trigger the resetEncoder once when the limit switch goes from off to on
     */
    public void resetEncoder() {
        elevatorEncoder.setZeroOffset(elevatorEncoder.getPosition());    }

    /**
     * useState gets called every update by the TrapezoidProfileSubsystem periodic
     */
    @Override
    public void useState(TrapezoidProfile.State setpoint) {
        var f = elevatorFeedForward.calculate(setpoint.position, setpoint.velocity);
        pidController.setReference(setpoint.position, ControlType.kPosition, 0, f);

        // Log the current elevator position
        currentHeight = elevatorEncoder.getPosition();
        targetHeight = setpoint.position;
    }
  
    /**
     * Calls the setGoal(double) method on the TrapezoidProfileSubsystem
     */
    public Command setElevatorPosition(ElevatorPosition position) {
        return Commands.runOnce(() -> this.setGoal(position.position.in(Meters)));
    }

    /**
     * Use an enum to clearly specify target positions
     * This should probably be in Constants
     */
    public enum ElevatorPosition {
        STOWED(Meters.of(0.0)),
        AMP(Meters.of(1.0));
    
        public final Measure<Distance> position;
    
        private ElevatorPosition(Measure<Distance> position) {
            this.position = position;
        }
    }
}
