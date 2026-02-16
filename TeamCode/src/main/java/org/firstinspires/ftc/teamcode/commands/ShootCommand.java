package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.config.IntakeConfig.INTAKE_POWER;
import static org.firstinspires.ftc.teamcode.config.IntakeConfig.SHOOTING_POWER;
import static org.firstinspires.ftc.teamcode.config.IntakeConfig.SLOW_SHOOTING_POWER;

import android.util.Log;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.ConditionalCommand;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelCommandGroup;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.skeletonarmy.marrow.OpModeManager;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.calculators.ShootingSolution;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.utilities.TrajectoryCalculator;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.mechanism.LoggedMechanism2d;
import org.psilynx.psikit.core.wpi.math.Pose2d;
import org.psilynx.psikit.core.wpi.math.Translation3d;

import java.util.function.BooleanSupplier;

public class ShootCommand extends SequentialCommandGroup {
    private final Shooter shooter;
    private final Intake intake;
    private final Transfer transfer;
    private final Drive drive;

    private final double inchesToMeters = 0.0254;
    public ShootCommand(Shooter shooter, Intake intake, Transfer transfer, Drive drive) {
        this(shooter, intake, transfer, drive, () -> false);
    }

    public ShootCommand(Shooter shooter, Intake intake, Transfer transfer, Drive drive, BooleanSupplier dontUpdate) {
        addRequirements(shooter, intake, transfer);

        this.shooter = shooter;
        this.intake = intake;
        this.transfer = transfer;
        this.drive = drive;

        addCommands(
                waitUntilCanShoot(),
                new InstantCommand(this::recordShot),
                new InstantCommand(() -> drive.setShootingMode(true)),
                new InstantCommand(() -> intake.setIntakeSpeed(SHOOTING_POWER)),
//                new InstantCommand(() -> { if (dontUpdate.getAsBoolean()) shooter.setUpdateHood(false); }),
                new InstantCommand(() -> { if (dontUpdate.getAsBoolean()) intake.setIntakeSpeed(SLOW_SHOOTING_POWER); }),

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

                new InstantCommand(() -> drive.setShootingMode(false)),
                new InstantCommand(() -> intake.setIntakeSpeed(INTAKE_POWER))
//                new InstantCommand(() -> { if (dontUpdate.getAsBoolean()) shooter.setUpdateHood(true); })
        );
    }

    public Command waitUntilCanShoot() {
        return new SequentialCommandGroup(
                new WaitUntilCommand(() -> shooter.getCanShoot() || shooter.getVerticalManualMode()),
                new WaitUntilCommand(() -> shooter.reachedAngle() || shooter.getHorizontalManualMode())
        ).withTimeout(2000);
    }

    public void recordShot() {
        OpModeManager.getTelemetry().addData("Shot/RPM", shooter.getRPM());
        OpModeManager.getTelemetry().addData("Shot/Angle hood", shooter.getHoodAngleDegrees());
        OpModeManager.getTelemetry().addData("Shot/Turret angle error (deg)", shooter.wrapped - shooter.getTurretAngle(AngleUnit.DEGREES));
        Logger.recordOutput("Shot/RPM: ", shooter.getRPM());
        Logger.recordOutput("Shot/Angle hood: ", shooter.getHoodAngleDegrees());
        Logger.recordOutput("Shot/Turret angle error (deg): ", shooter.wrapped - shooter.getTurretAngle(AngleUnit.DEGREES));


        // simulates the ball trajectory accounting for air resistance
        ShootingSolution solution = shooter.solution;
        Pose shotPos = shooter.currentPose.getAsCoordinateSystem(FTCCoordinates.INSTANCE);
        Logger.recordOutput("Shot/Trajectory" , TrajectoryCalculator.generateTrajectory(new Translation3d(shotPos.getX() * inchesToMeters, shotPos.getY() * inchesToMeters, 0.4), solution.getExitVel(), solution.getVerticalAngle(), solution.getHorizontalAngle() - Math.PI, 2, 0.1));
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        // Ensure everything is off and in its place when the command ends
        transfer.setKickerPosition(false);
        transfer.block();
        drive.setShootingMode(false);
        shooter.setUpdateHood(true);
        intake.setIntakeSpeed(INTAKE_POWER);
    }
}
