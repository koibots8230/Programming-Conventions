package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.*;
import frc.lib.util.FeedforwardGains;
import frc.lib.util.MotorConstants;
import frc.lib.util.PIDGains;

public class Constants {

  public class DriveConstants {

    public static final LinearVelocity MAX_VELOCITY = MetersPerSecond.of(4);

    public static final PIDGains PID_GAINS = new PIDGains.Builder().kp(0.0).build();
    public static final FeedforwardGains FEEDFORWARD_GAINS =
        new FeedforwardGains.Builder().kv(0.0).build();

    public static final MotorConstants FRONT_LEFT_MOTOR_CONSTANTS = new MotorConstants(false, 60);
    public static final MotorConstants BACK_LEFT_MOTOR_CONSTANTS = new MotorConstants(false, 60);
    public static final MotorConstants FRONT_RIGHT_MOTOR_CONSTANTS = new MotorConstants(false, 60);
    public static final MotorConstants BACK_RIGHT_MOTOR_CONSTANTS = new MotorConstants(false, 60);

    public static final Distance WHEEL_CIRCUMFERENCE = Inches.of(6 * Math.PI);
    public static final double GEARING = 1.0 / 1.0;

    public static final int FRONT_LEFT_MOTOR = 1;
    public static final int BACK_LEFT_MOTOR = 2;

    public static final int FRONT_RIGHT_MOTOR = 3;
    public static final int BACK_RIGHT_MOTOR = 4;

    public static final int GYRO_PORT = 5;
  }

  public class IntakeConstants {
    public static final AngularVelocity INTAKE_SPEED = RPM.of(300);

    public static final PIDGains PID_GAINS = new PIDGains.Builder().kp(0.0).build();
    public static final FeedforwardGains FEEDFORWARD_GAINS =
        new FeedforwardGains.Builder().kv(0.0).build();

    public static final MotorConstants MOTOR_CONSTANTS = new MotorConstants(false, 60);

    public static final int MOTOR_PORT = 6;
  }

  public class IndexerConstants {
    public static final AngularVelocity INTAKE_SPEED = RPM.of(600);
    public static final AngularVelocity SHOOTING_SPEED = RPM.of(1200);

    public static final PIDGains PID_GAINS = new PIDGains.Builder().kp(0.0).build();
    public static final FeedforwardGains FEEDFORWARD_GAINS =
        new FeedforwardGains.Builder().kv(0.0).build();

    public static final MotorConstants MOTOR_CONSTANTS = new MotorConstants(false, 60);

    public static final int MOTOR_PORT = 7;

    public static final int DISTANCE_SWITCH_PORT = 1;
  }

  public class TurretConstants {
    // You would name these whatever is appropriate instead of position one/two
    public static final Angle POSITION_ONE = Radians.of(0);
    public static final Angle POSITION_TWO = Radians.of(Math.PI);

    public static final AngularVelocity MAX_VELOCITY = RadiansPerSecond.of(Math.PI * 20);
    public static final AngularAcceleration MAX_ACCELERATION =
        RadiansPerSecondPerSecond.of(Math.PI * 18);

    public static final PIDGains PID_GAINS = new PIDGains.Builder().kp(0.0).build();
    public static final FeedforwardGains FEEDFORWARD_GAINS =
        new FeedforwardGains.Builder().kv(0.0).build();

    public static final double CONVERSION_FACTOR = Math.PI * 2;

    public static final MotorConstants MOTOR_CONSTANTS = new MotorConstants(false, 60);

    public static final int MOTOR_PORT = 8;
  }

  public class RobotConstants {
    public static final Time LOOP_TIME = Seconds.of(0.02);

    public static final Voltage NOMINAL_VOLTAGE = Volts.of(12);

    public static final double JOYSTICK_DEADZONE = 0.05;
  }
}
