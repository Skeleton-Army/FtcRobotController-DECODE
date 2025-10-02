package org.firstinspires.ftc.teamcode.opModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.psilynx.psikit.core.rlog.RLOGServer;
import org.psilynx.psikit.core.rlog.RLOGWriter;
import org.psilynx.psikit.core.Logger;

import org.psilynx.psikit.core.wpi.Pose2d;
import org.psilynx.psikit.core.wpi.Rotation2d;
import org.psilynx.psikit.ftc.PsiKitOpMode;

@TeleOp(name="ConceptPsiKitLogger")
class ConceptPsiKitLogger extends PsiKitOpMode {

    Follower follower;
    @Override
    public void runOpMode() {
        psiKitSetup();
        RLOGServer server = new RLOGServer();
        RLOGWriter writer = new RLOGWriter("logs.rlog");
        server.start();
        writer.start();
        Logger.addDataReceiver(server);
        Logger.addDataReceiver(writer);
        Logger.recordMetadata("some metadata", "string value");
        Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
        Logger.periodicAfterUser(0, 0);

        follower = Constants.createFollower(hardwareMap);
        follower.startTeleopDrive(true);
        follower.setPose(new Pose(0,0,0));

        // AKA init loop
        while(!getPsiKitIsStarted()){
            Logger.periodicBeforeUser();

            processHardwareInputs();
            // this MUST come before any logic

         /*

          Init logic goes here

         */

            Logger.periodicAfterUser(0.0, 0.0);
            // logging these timestamps is completely optional
        }

        // alternately the waitForStart() function works as expected.

        // AKA loop
        while(!getPsiKitIsStopRequested()) {

            double beforeUserStart = Logger.getTimestamp();
            Logger.periodicBeforeUser();
            double beforeUserEnd = Logger.getTimestamp();

            processHardwareInputs();
            // this MUST come before any logic

         /*

          OpMode logic goes here

         */

            follower.update();
            follower.setTeleOpDrive(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, true);

            Pose2d pose2d = new Pose2d(follower.getPose().getX(), follower.getPose().getY(), new Rotation2d(follower.getPose().getHeading()));
            Logger.recordOutput("OpMode/example", 2.0);
            Logger.recordOutput("OpMode/Pose2D", pose2d);
            // example

            double afterUserStart = Logger.getTimestamp();
            Logger.periodicAfterUser(
                    afterUserStart - beforeUserEnd,
                    beforeUserEnd - beforeUserStart
            );
            // alternately, keep track of how long some things are taking. up to
            // you on what you want to do
        }
        Logger.end();
    }
}
