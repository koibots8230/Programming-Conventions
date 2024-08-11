package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.*;
import frc.lib.util.FeedForwardConstants;
import frc.lib.util.MotorConstants;
import frc.lib.util.PIDConstants;

public class Constants {

    public class DriveConstants {

        public static final Measure<Velocity<Distance>> MAX_VELOCITY = MetersPerSecond.of(4);

        public static final PIDConstants FEEDBACK_REAL = new PIDConstants(0.005, 0, 0);
        public static final FeedForwardConstants FEEDFORWARD_REAL =
                new FeedForwardConstants(0, 0.005);

        public static final PIDConstants FEEDBACK_SIM = new PIDConstants(1.5, 0, 0);
        public static final FeedForwardConstants FEEDFORWARD_SIM = new FeedForwardConstants(0, 10);

        public static final MotorConstants MOTOR_CONSTANTS = new MotorConstants(false, 60);

        public static final int FRONT_LEFT_MOTOR = 1;
        public static final int BACK_LEFT_MOTOR = 2;

        public static final int FRONT_RIGHT_MOTOR = 3;
        public static final int BACK_RIGHT_MOTOR = 4;

        public static final Measure<Distance> WHEEL_CIRCUMFERENCE = Inches.of(6 * Math.PI);
    }

    public class IntakeConstants {
        public static final Measure<Velocity<Angle>> INTAKE_SPEED = RPM.of(600);

        public static final PIDConstants FEEDBACK_REAL = new PIDConstants(0.005, 0, 0);
        public static final FeedForwardConstants FEEDFORWARD_REAL =
                new FeedForwardConstants(0, 0.005);

        public static final PIDConstants FEEDBACK_SIM = new PIDConstants(1.5, 0, 0);
        public static final FeedForwardConstants FEEDFORWARD_SIM = new FeedForwardConstants(0, 10);

        public static final MotorConstants MOTOR_CONSTANTS = new MotorConstants(false, 60);
        public static final PIDConstants MOTOR_PID = new PIDConstants(0.01, 0.0, 0.0);
        public static final Measure<Velocity<Angle>> MAX_VELOCITY = RPM.of(1200); 
        public static final Measure<Velocity<Angle>> MAX_ACCELERATION = RPM.of(1200); 

        public static final int MOTOR_PORT = 5;

        public static final double AllowedRange = 0.1;
    }

    public class RobotConstants {
        public static final Measure<Time> PERIODIC_LOOP = Milliseconds.of(20);

        public static final boolean LOGGING_FILE_ONLY = false;
        public static final boolean LAZY_LOGGING = false;

        public static final double JOYSTICK_DEADZONE = 0.05;
    }
}
