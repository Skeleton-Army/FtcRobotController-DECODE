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

        addCommands(
                waitUntilCanShoot(),
                new InstantCommand(this::recordShot),
                new InstantCommand(() -> drive.setShootingMode(true)),

                new InstantCommand(transfer::release),
                new InstantCommand(intake::collect),
                new WaitCommand(1000),
                new ConditionalCommand(
                        transfer.kick(),
                        new InstantCommand(),
                        transfer::isArtifactDetected
                ),
                new InstantCommand(transfer::block),
                new InstantCommand(intake::stop),

                new InstantCommand(() -> drive.setShootingMode(false))
        );
    }

    public Command waitUntilCanShoot() {
        return new SequentialCommandGroup(
                new WaitUntilCommand(() -> shooter.getCanShoot() || shooter.getVerticalManualMode())
//                new WaitUntilCommand(() -> shooter.reachedAngle() || shooter.getHorizontalManualMode())
        );
    }

    public void recordShot() {
        Logger.recordOutput("Shot/RPM: ", shooter.getRPM());
        Logger.recordOutput("Shot/Angle hood: ", shooter.getHoodAngleDegrees());
        Logger.recordOutput("Shot/Turret angle error (deg): ", shooter.wrapped - shooter.getTurretAngle(AngleUnit.DEGREES));
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        // Ensure everything is off and in its place when the command ends
        transfer.setKickerPosition(false);
        transfer.block();
        drive.setShootingMode(false);
    }
}
