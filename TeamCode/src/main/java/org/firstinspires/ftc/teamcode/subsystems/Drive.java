package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.command.Command;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;

public class Drive extends SubsystemBase {
    private final Follower follower;

    private Command currentCommand;

    public Drive(Follower follower) {
        this.follower = follower;
    }

    public Command getCurrentCommand() {
        return currentCommand;
    }

    public Command goToBase() {
        PathChain parking = follower
                            .pathBuilder()
                            .addPath(
                                    new BezierLine(
                                            follower.getPose(),
                                            new Pose(38.5,33.5)
                                    )
                            )
                            .setLinearHeadingInterpolation(follower.getHeading(), Math.toRadians(getClosestRightAngle(follower)))
                            .build();

        currentCommand = new FollowPathCommand(follower, parking);

        return currentCommand;
    }

    public void joystickDrive(Gamepad gamepad) {
        follower.setTeleOpDrive(-gamepad.left_stick_y, -gamepad.left_stick_x, -gamepad.right_stick_x, true);
    }

    @Override
    public void periodic() {
        follower.update();
    }

    private int getClosestRightAngle(Follower follower) {
        double heading = follower.getHeading();
        heading = ((heading % 360) + 360) % 360; // normalize to 0–360

        int closest = 0;
        double minDiff = 360;

        int[] rightAngles = {0, 90, 180, 270};
        for (int angle : rightAngles) {
            double diff = Math.abs(heading - angle);
            diff = Math.min(diff, 360 - diff); // handle wrap-around (e.g. 359° to 0°)
            if (diff < minDiff) {
                minDiff = diff;
                closest = angle;
            }
        }

        return closest;
    }
}
