package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.ClawExtender;
import org.firstinspires.ftc.teamcode.Intanke;
import org.firstinspires.ftc.teamcode.Lift;

@Autonomous(name = "RedSideBackAuto")
public class RedSide extends LinearOpMode {

    private Intanke intanke = null;
    private Lift lift = null;
    private SampleMecanumDrive drive = null;
    private ClawExtender extender = null;

    @Override
    public void runOpMode() {
        // Initialize hardware
        intanke = new Intanke(hardwareMap);
        lift = new Lift(hardwareMap);
        extender = new ClawExtender(hardwareMap);
        drive = new SampleMecanumDrive(hardwareMap);

        // Set starting pose
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER); // Better for control

        // Define trajectories
        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .strafeLeft(8)
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .forward(25)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .back(20)
                .build();

        waitForStart();

        if (isStopRequested()) return;

        // Follow trajectories
        drive.followTrajectory(traj1);
        debugPose(drive);

        drive.followTrajectory(traj2);
        debugPose(drive);

        drive.followTrajectory(traj3);
        debugPose(drive);

        // Main update loop
        while (opModeIsActive()) {
            intanke.update();
            lift.update();
            drive.update();

            // Optional live pose debugging
            debugPose(drive);
        }
    }

    private void debugPose(SampleMecanumDrive drive) {
        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("X", poseEstimate.getX());
        telemetry.addData("Y", poseEstimate.getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(poseEstimate.getHeading()));
        telemetry.update();
    }
}
