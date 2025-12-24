package org.firstinspires.ftc.teamcode.commands;

import android.util.Log;

import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.mechanism.LoggedMechanism2d;

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
                //waitUntilCanShoot(),

                new ParallelCommandGroup(
                        new SequentialCommandGroup(
                                new InstantCommand(intake::collect),
                                new WaitCommand(500),
                                new InstantCommand(intake::stop)
                        ),
                        shootWithTransfer()
                ),

                new InstantCommand(intake::collect),

                // Skip if first shot timed-out (only one artifact inside robot)
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                //waitUntilCanShoot(),
                                shootWithTransfer()
                        ),
                        new InstantCommand(),
                        () -> !shooter.reachedRPM()
                ),

                new InstantCommand(() -> transfer.toggleTransfer(true)),
                new WaitUntilCommand(transfer::isArtifactDetected)
                        .withTimeout(1000),
                new ParallelCommandGroup(
                        //kWaitUntilCanShoot(),
                        new WaitCommand(10)
                ),

                new InstantCommand(intake::stop),
                new InstantCommand(() -> transfer.toggleTransfer(false)),
                transfer.kick(),
                new InstantCommand(this::recordShot)
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
                        .withTimeout(700),
                new InstantCommand(() -> transfer.toggleTransfer(false))
        );
    }

    public void recordShot() {
        Logger.recordOutput("Shot/RPM: ", shooter.getRPM());
        Logger.recordOutput("Shot/Angle hood: ", shooter.getHoodAngle());
        Logger.recordOutput("Shot/Turret angle error (deg): ", shooter.wrapped - shooter.getTurretAngle(AngleUnit.DEGREES));
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
