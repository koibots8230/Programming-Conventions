package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class IntakeCommands {
    public static Command autoIntake(Intake intake, Indexer indexer) {
        return Commands.parallel(
            intake.setVelocityCommand(IntakeConstants.INTAKE_SPEED),
            indexer.intake()
        );
    }
}
