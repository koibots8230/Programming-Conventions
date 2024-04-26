// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.TankDrive;
import monologue.Logged;
import monologue.Monologue;

public class RobotContainer implements Logged {

    private final XboxController driveController = new XboxController(0);
    private final GenericHID operatorPad = new GenericHID(1);

    // These are capitalized just to make the logs look nicer
    private final TankDrive TankDrive;
    private final Intake Intake;

    public RobotContainer(boolean isReal) {
        TankDrive = new TankDrive(isReal);
        Intake = new Intake(isReal);

        Monologue.setupMonologue(
                this, "Robot", RobotConstants.LOGGING_FILE_ONLY, RobotConstants.LAZY_LOGGING);

        configureBindings();
    }

    public void updateLogs() {
        Monologue.updateAll();
    }

    private void configureBindings() {

        // =================================Driver=================================

        // Note how the TankDrive subsystem instance is passed to DriveCommand
        TankDrive.setDefaultCommand(
                new DriveCommand(
                        TankDrive,
                        () -> driveController.getLeftY(),
                        () -> driveController.getRightY()));

        // =================================Operator=================================

        final Trigger intakeTrigger = new Trigger(() -> operatorPad.getRawButton(1));
        intakeTrigger.onTrue(
                new InstantCommand(() -> Intake.setVelocity(IntakeConstants.INTAKE_SPEED), Intake));
        intakeTrigger.onTrue(new InstantCommand(() -> Intake.setVelocity(RPM.of(0)), Intake));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
