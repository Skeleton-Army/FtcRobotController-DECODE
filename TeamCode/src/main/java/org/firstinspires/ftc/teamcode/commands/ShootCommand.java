package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.config.ShooterConfig.KICK_TIME;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.RepeatCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;

public class ShootCommand extends SequentialCommandGroup {
    public ShootCommand(Shooter shooter, Intake intake) {
        addRequirements(shooter, intake);

        addCommands(
                cycle(shooter, intake),
                cycle(shooter, intake),
                cycle(shooter, intake),
                new InstantCommand(intake::stop)
        );
    }

    private Command cycle(Shooter shooter, Intake intake) {
        return new SequentialCommandGroup(
                new InstantCommand(shooter::spinUp),
                new WaitUntilCommand(shooter::reachedRPM),
                new InstantCommand(intake::collect),
                new InstantCommand(() -> shooter.toggleTransfer(true)),
                new WaitCommand(500),
                new InstantCommand(() -> shooter.toggleTransfer(false)),
                new InstantCommand(shooter::kick),
                new WaitCommand(KICK_TIME)
        );
    }
}
