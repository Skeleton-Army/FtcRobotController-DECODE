package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class ShootCommand extends SequentialCommandGroup {
    public ShootCommand(Shooter subsystem) {
        addRequirements(subsystem);

        addCommands(
                // TODO: Add commands here
                // 1) Set shooter velocity
                // 2) Wait for shooter to reach velocity
                // 3) Put bol in
                // 4) Pew pew
        );
    }
}
