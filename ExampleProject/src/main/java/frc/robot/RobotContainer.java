// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.TurretConstants;
import frc.robot.commands.IntakeCommands;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.TankDrive;
import frc.robot.subsystems.Turret;

@Logged
public class RobotContainer {

    @NotLogged private final CommandXboxController driveController = new CommandXboxController(0);

    private final TankDrive tankDrive;
    private final Intake intake;
    private final Indexer indexer;
    private final Turret turret;

    public RobotContainer() {
        tankDrive = new TankDrive();
        intake = new Intake();
        indexer = new Indexer();
        turret = new Turret();

        configureBindings();
    }

    private void configureBindings() {
        tankDrive.setDefaultCommand(tankDrive.driveCommand(
            driveController::getLeftY, driveController::getRightY));

        Trigger intakeTrigger = driveController.axisGreaterThan(XboxController.Axis.kRightTrigger.value, 0.15);
        intakeTrigger.onTrue(IntakeCommands.autoIntake(intake, indexer));
        intakeTrigger.onFalse(Commands.parallel(
            intake.setVelocityCommand(RPM.of(0)),
            indexer.setVelocityCommand(RPM.of(0))
        ));

        Trigger moveTurret = driveController.a();
        moveTurret.onTrue(turret.setPositionCommand(TurretConstants.POSITION_TWO));
        moveTurret.onFalse(turret.setPositionCommand(TurretConstants.POSITION_ONE));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
