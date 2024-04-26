package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.subsystems.TankDrive;
import java.util.function.DoubleSupplier;

public class DriveCommand extends Command {

    private final TankDrive tankDrive;

    private final DoubleSupplier leftStick;
    private final DoubleSupplier rightStick;

    public DriveCommand(TankDrive tankDrive, DoubleSupplier leftStick, DoubleSupplier rightStick) {
        this.tankDrive = tankDrive;
        this.leftStick = leftStick;
        this.rightStick = rightStick;

        addRequirements(tankDrive);
    }

    @Override
    public void execute() {
        tankDrive.setVelocity(
                DriveConstants.MAX_VELOCITY.times(
                        MathUtil.applyDeadband(
                                leftStick.getAsDouble(), RobotConstants.JOYSTICK_DEADZONE)),
                DriveConstants.MAX_VELOCITY.times(
                        MathUtil.applyDeadband(
                                rightStick.getAsDouble(), RobotConstants.JOYSTICK_DEADZONE)));
    }
}
