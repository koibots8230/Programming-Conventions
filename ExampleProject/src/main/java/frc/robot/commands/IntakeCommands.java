package frc.robot.commands;

import static edu.wpi.first.units.Units.RPM;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

public class IntakeCommands {
  public static Command autoIntake(Intake intake, Indexer indexer) {
    return Commands.sequence(
        Commands.parallel(
            intake.setVelocityCommand(IntakeConstants.INTAKE_SPEED), indexer.intake()),
        intake.setVelocityCommand(RPM.of(0)));
  }
}
