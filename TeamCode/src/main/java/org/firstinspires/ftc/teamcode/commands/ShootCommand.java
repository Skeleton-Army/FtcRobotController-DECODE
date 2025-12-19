package org.firstinspires.ftc.teamcode.commands;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
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

    public ShootCommand(Shooter shooter, Intake intake, Transfer transfer, Drive drive) {
        addRequirements(shooter, intake, transfer);

        this.shooter = shooter;
        this.intake = intake;
        this.transfer = transfer;
        this.drive = drive;

        addCommands(new InstantCommand(() -> drive.setShootingMode(true)));
        addCommands(cycle());
        addCommands(new InstantCommand(() -> drive.setShootingMode(false)));
    }

    private Command cycle() {
        return new SequentialCommandGroup(
                waitUntilCanShoot(),

                new InstantCommand(intake::collect),
                shootWithTransfer(),

                // Skip if first shot timed-out (only one artifact inside robot)
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                waitUntilCanShoot(),
                                shootWithTransfer()
                        ),
                        new InstantCommand(),
                        () -> !shooter.reachedRPM()
                ),

                new InstantCommand(() -> transfer.toggleTransfer(true)),
                new WaitUntilCommand(transfer::isArtifactDetected)
                        .withTimeout(1000),
                new ParallelCommandGroup(
                        waitUntilCanShoot(),
                        new WaitCommand(1000)
                ),

                new InstantCommand(intake::stop),
                new InstantCommand(() -> transfer.toggleTransfer(false)),
                transfer.kick()
        );
    }

    public Command waitUntilCanShoot() {
        return new SequentialCommandGroup(
                new WaitUntilCommand(() -> shooter.reachedRPM() || shooter.getVerticalManualMode()),
                new WaitUntilCommand(() -> shooter.reachedAngle() || shooter.getHorizontalManualMode())
        );
    }

    public Command shootWithTransfer() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> transfer.toggleTransfer(true)),
                new WaitUntilCommand(() -> !shooter.reachedRPM())
                        .withTimeout(400),
                new InstantCommand(() -> transfer.toggleTransfer(false))
        );
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        // Ensure everything is off and in its place when the command ends
        transfer.setKickerPosition(false);
        transfer.toggleTransfer(false);
        drive.setShootingMode(false);
    }
}
