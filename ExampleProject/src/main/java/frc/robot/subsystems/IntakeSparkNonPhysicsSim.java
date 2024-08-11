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
 * 4 - The simulated motor velocity should accelerate to and hold steady at the target velocity
 * 5 - The subsystem provides a method to check if the current real/sim velocity is in range of the target velocity
 */

public class IntakeSparkNonPhysicsSim extends SubsystemBase {

    private final CANSparkMax motor;
    private final SparkPIDController pidController;
    private final RelativeEncoder encoder;

    private Measure<Velocity<Angle>> targetVelocity;

    // Set up data for logging
    @Log private double targetVelocityRPM;
    @Log private double currentVelocityRPM;

    // Set up data for simulation
    private final CANSparkMaxSIM canSparkMaxSIM;

    public IntakeSparkNonPhysicsSim(boolean isReal) {
        motor = new CANSparkMax(IntakeConstants.MOTOR_PORT, MotorType.kBrushless);

        motor.setInverted(IntakeConstants.MOTOR_CONSTANTS.inverted);
        motor.setSmartCurrentLimit(IntakeConstants.MOTOR_CONSTANTS.currentLimit);

        pidController = motor.getPIDController();
        pidController.setP(IntakeConstants.MOTOR_PID.kP);

        encoder = motor.getEncoder();

        // Instantiate Simulation Objects
        if (isReal) {
            // Set to null to force null pointer exceptions if called during real robot actions
            canSparkMaxSIM = null;
        } else {
            canSparkMaxSIM = new CANSparkMaxSIM();
        }

    }

    @Override
    public void periodic() {
        pidController.setReference(targetVelocity.in(RPM), ControlType.kVelocity);

        // Data Logging
        currentVelocityRPM = encoder.getVelocity();
    }
    
    /**
     * Simulation periodic calls parallel methods on the simulation objects
     */
    @Override
    public void simulationPeriodic() {
        canSparkMaxSIM.setReference(targetVelocity.in(RPM), ControlType.kVelocity);
        currentVelocityRPM = canSparkMaxSIM.getVeloicty();
    }


    // =========================Velocity=========================

    public void setVelocity(Measure<Velocity<Angle>> velocity) {
        this.targetVelocity = velocity;
    }

    /**
     * Need to consider possible approaches to resolving the real/sim distinction here
     */
    public boolean isInRange() {
        // This version uses the real objects
        return Math.abs(targetVelocity.in(RPM) - encoder.getVelocity()) <= IntakeConstants.AllowedRange;

        // This version uses the sim objects
        //return Math.abs(targetVelocity.in(RPM) - canSparkMaxSIM.getVelocity()) <= IntakeConstants.AllowedRange;

        // This version uses the common logging objects to avoid distinguishing real/sim
        //return Math.abs(targetVelocityRPM - currentVelocityRPM) <= IntakeConstants.AllowedRange;

    }

    /**
     * Motor simulation accounting for acceleration
     * Intakes normally do not need ramping since they are low inertia and do not need to worry about acceleration.
     */
    private class CANSparkMaxSIM {
        // Configurable settings for max velocity and acceleration

        // Varialbes for current position and velocity

        /**
         * Create sim motor.
         * Configurable settings should be passed in
         */
        private CANSparkMaxSIM() {

        }

        /**
         * The CANSparkMAX has two methods that return a PID controller object and an encoder object.
         * Rather than have two child objects to work with the sim, combine methods here.
         */

         // PID Controller method, setReference
         public void setReference(double value, ControlType ctrl) {
            // Set the target for the motor
         }

         // Encoder method, getVelocity
         public double getVeloicty() {
            // Return the current motor velocity
            return 0.0;
         }

         /**
          * Update the simulation
          * This method takes in a time step for updating the member variables.
          * Use the current velocity, target velocity, and max acceleration to compute the new velocity at the current time step.
          * Use the new velocity to compute the distance traveled over the time step.
          */
          public void update(double timeStep) {

          }

    }

}
