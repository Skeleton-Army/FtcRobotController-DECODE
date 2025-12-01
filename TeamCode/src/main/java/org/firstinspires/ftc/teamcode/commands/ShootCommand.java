package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;

public class ShootCommand extends SequentialCommandGroup {
    private final Shooter shooter;
    private final Intake intake;
    private final Transfer transfer;

    public ShootCommand(int numberOfArtifacts, Shooter shooter, Intake intake, Transfer transfer) {
        addRequirements(shooter, intake, transfer);

        this.shooter = shooter;
        this.intake = intake;
        this.transfer = transfer;

        for (int i = 0; i < numberOfArtifacts; i++) {
            addCommands(cycle());
        }
    }

    private Command cycle() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    if (!transfer.isArtifactDetected()) {
                        intake.collect();
                        transfer.toggleTransfer(true);
                    }
                }),

                // Either artifact gets detected OR the timeout elapses
                new WaitUntilCommand(transfer::isArtifactDetected)
                        .withTimeout(1000),

                // If we got here but artifact is still NOT detected, it timed out -> cancel the whole ShootCommand
                new InstantCommand(() -> {
                    if (!transfer.isArtifactDetected()) {
                        intake.stop();
                        transfer.toggleTransfer(false);
                        cancel();
                    }
                }),

                new WaitUntilCommand(shooter::reachedRPM),

                new InstantCommand(intake::stop),
                new InstantCommand(() -> transfer.toggleTransfer(false)),

                transfer.kick()
        );
    }
}
