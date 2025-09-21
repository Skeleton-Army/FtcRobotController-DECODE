package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class AdjustAnglesCommand extends ParallelCommandGroup {

    private final Shooter shooterSubSystem;
    public AdjustAnglesCommand(Shooter subsystem) {
        addRequirements(subsystem);

        shooterSubSystem = subsystem;

        addCommands(new ParallelCommandGroup(
                new InstantCommand(() -> shooterSubSystem.updateHorizontalAngle()),
                new InstantCommand(() -> shooterSubSystem.updateVerticalAngle())
        ));

    }
}
