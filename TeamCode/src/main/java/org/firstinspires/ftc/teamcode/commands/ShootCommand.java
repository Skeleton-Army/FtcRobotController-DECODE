package org.firstinspires.ftc.teamcode.commands;

import static org.firstinspires.ftc.teamcode.config.IntakeConfig.INTAKE_POWER;
import static org.firstinspires.ftc.teamcode.config.IntakeConfig.SHOOTING_POWER;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.Pose;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.CommandBase;
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.skeletonarmy.marrow.OpModeManager;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.calculators.ShootingSolution;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.utilities.TrajectoryCalculator;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.wpi.math.Translation3d;

public class ShootCommand extends SequentialCommandGroup {
    private final Shooter shooter;
    private final Intake intake;
    private final Transfer transfer;
    private final Drive drive;

    private final double inchesToMeters = 0.0254;

    public ShootCommand(Shooter shooter, Intake intake, Transfer transfer, Drive drive) {
        this(shooter, intake, transfer, drive, SHOOTING_POWER);
    }

    public ShootCommand(Shooter shooter, Intake intake, Transfer transfer, Drive drive, int waitMillis) {
        this(shooter, intake, transfer, drive, SHOOTING_POWER, waitMillis, true);
    }

    public ShootCommand(Shooter shooter, Intake intake, Transfer transfer, Drive drive, double intakeSpeed) {
        this(shooter, intake, transfer, drive, intakeSpeed, true);
    }

    public ShootCommand(Shooter shooter, Intake intake, Transfer transfer, Drive drive, double intakeSpeed, int waitMillis) {
        this(shooter, intake, transfer, drive, intakeSpeed, waitMillis, true);
    }

    public ShootCommand(Shooter shooter, Intake intake, Transfer transfer, Drive drive, double intakeSpeed, boolean waitUntilReady) {
        this(shooter, intake, transfer, drive, intakeSpeed, 600, waitUntilReady);
    }

    public ShootCommand(Shooter shooter, Intake intake, Transfer transfer, Drive drive, double intakeSpeed, int waitMillis, boolean waitUntilReady) {
        addRequirements(shooter, intake, transfer);

        this.shooter = shooter;
        this.intake = intake;
        this.transfer = transfer;
        this.drive = drive;

        addCommands(
                waitUntilReady ? waitUntilCanShoot() : new InstantCommand(),

                new InstantCommand(() -> {
                    recordShot();
                    drive.setShootingMode(true);
                    transfer.release();
                }),
                new WaitUntilCommand(shooter::getCanShoot).withTimeout(2000), // don't start feeding until stable
                new InstantCommand(() -> intake.setIntakeSpeed(intakeSpeed)),
                new WaitCommand(100), // wait for stopper to open

                new FeedWhileCanShootCommand(intakeSpeed, waitMillis),

                new InstantCommand(() -> {
                    transfer.block();
                    intake.stop();
                    drive.setShootingMode(false);
                    intake.setIntakeSpeed(INTAKE_POWER);
                })
        );
    }

    public Command waitUntilCanShoot() {
        return new SequentialCommandGroup(
                new WaitUntilCommand(() -> shooter.getCanShootRPMCalc() || shooter.getVerticalManualMode()),
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
        Logger.recordOutput("Shot/Trajectory" , TrajectoryCalculator.generateTrajectory(new Translation3d(-shotPos.getX() * inchesToMeters, -shotPos.getY() * inchesToMeters, 0.4), solution.getExitVel(), solution.getVerticalAngle(), solution.getHorizontalAngle() - Math.PI, 1, 0.1));
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);

        // Ensure everything is off and in its place when the command ends
        transfer.setKickerPosition(false);
        drive.setShootingMode(false);
        shooter.setUpdateHood(true);
        intake.setIntakeSpeed(INTAKE_POWER);
        transfer.block();
    }

    /**
     * Feeds artifacts through the intake for a total of {@code waitMillis} of
     * ACTIVE (canShoot == true) time. If canShoot becomes false mid-shot, the
     * intake is paused (not stopped/cancelled) and the elapsed-time clock
     * freezes until canShoot becomes true again, at which point feeding resumes
     * and the clock keeps counting toward waitMillis.
     */
    private class FeedWhileCanShootCommand extends CommandBase {
        private static final long DEBOUNCE_MS = 50;

        private final double intakeSpeed;
        private final int waitMillis;

        private long accumulatedMs;
        private long lastTimestamp;

        // Debounce state
        private boolean canShootFalseSeen;
        private long canShootFalseSince;
        private boolean paused;

        FeedWhileCanShootCommand(double intakeSpeed, int waitMillis) {
            this.intakeSpeed = intakeSpeed;
            this.waitMillis = waitMillis;
            addRequirements(intake);
        }

        @Override
        public void initialize() {
            accumulatedMs = 0;
            lastTimestamp = System.currentTimeMillis();
            canShootFalseSeen = false;
            canShootFalseSince = 0;
            paused = false;
        }

        @Override
        public void execute() {
            long now = System.currentTimeMillis();
            long delta = now - lastTimestamp;
            lastTimestamp = now;

            boolean canShoot = shooter.getCanShoot();

            if (canShoot) {
                // Reset debounce tracking the instant canShoot is true again
                canShootFalseSeen = false;
                paused = false;
            } else {
                if (!canShootFalseSeen) {
                    canShootFalseSeen = true;
                    canShootFalseSince = now;
                } else if (!paused && (now - canShootFalseSince) >= DEBOUNCE_MS) {
                    // canShoot has been continuously false for >= 100ms, actually pause
                    paused = true;
                }
            }

            if (!paused) {
                intake.setIntakeSpeed(intakeSpeed);
                intake.collect();
                accumulatedMs += delta;
            } else {
                intake.stop(); // debounced pause: canShoot has been false long enough
            }
        }

        @Override
        public boolean isFinished() {
            return accumulatedMs >= waitMillis;
        }

        @Override
        public void end(boolean interrupted) {
            intake.stop();
        }
    }
}