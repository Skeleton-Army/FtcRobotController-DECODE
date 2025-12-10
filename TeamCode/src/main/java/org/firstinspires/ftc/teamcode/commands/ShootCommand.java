package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.config.DriveConfig.*;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

public class ShootCommand extends SequentialCommandGroup {
    private final Shooter shooter;
    private final Intake intake;
    private final Transfer transfer;
    private final Drive drive;

    public ShootCommand(int numberOfArtifacts, Shooter shooter, Intake intake, Transfer transfer, Drive drive) {
        addRequirements(shooter, intake, transfer);

        this.shooter = shooter;
        this.intake = intake;
        this.transfer = transfer;
        this.drive = drive;

        addCommands(new InstantCommand(() -> drive.setMovementSpeed(SHOOTING_MOVEMENT_SPEED)));

        for (int i = 0; i < numberOfArtifacts; i++) {
            addCommands(cycle());
        }

        addCommands(new InstantCommand(() -> drive.setMovementSpeed(MOVEMENT_SPEED)));
    }

    private Command cycle() {
        return new SequentialCommandGroup(
                new WaitUntilCommand(shooter::reachedRPM),

                new InstantCommand(() -> {
                    if (!transfer.isArtifactDetected()) {
                        intake.collect();
                        transfer.toggleTransfer(true);
                    }
                }),

                // Either artifact gets detected OR the timeout elapses
                new WaitUntilCommand(transfer::isArtifactDetected)
                        .withTimeout(1500),

                // If we got here but artifact is still NOT detected, it timed out -> cancel the whole ShootCommand
                new InstantCommand(() -> {
                    if (!transfer.isArtifactDetected()) {
                        intake.stop();
                        transfer.toggleTransfer(false);
                        cancel();
                    }
                }),

                new InstantCommand(intake::stop),
                new InstantCommand(() -> transfer.toggleTransfer(false)),

                new WaitCommand(300),

                transfer.kick()
        );
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        // Ensure everything is off and in its place when the command gets interrupted
        if (interrupted) {
            transfer.setKickerPosition(false);
            transfer.toggleTransfer(false);
            drive.setMovementSpeed(MOVEMENT_SPEED);
        }
    }
}
