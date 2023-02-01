package org.firstinspires.ftc.teamcode.auto.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.Constants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import org.firstinspires.ftc.teamcode.auto.Motion;
import org.firstinspires.ftc.teamcode.drive.Jeb;

import java.util.ArrayList;

@Autonomous(group = "drive")
public class RRTesting extends OpMode {
    private int stage = 0;
    private ArrayList<Motion> motions = new ArrayList<>();
    private ArrayList<TrajectorySequence> trajectories = new ArrayList<>();

    private Pose2d startPose = new Pose2d(-36, -64.5, Math.toRadians(135));

    @Override
    public void init() {
        Jeb drive = new Jeb(hardwareMap);

        drive.setPoseEstimate(startPose);
        trajectories.add(drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(90))
                .splineToLinearHeading(new Pose2d(-17, -58, Math.toRadians(90)), Math.toRadians(45),
                Jeb.getVelocityConstraint(Constants.MAX_VEL * 0.6, Constants.MAX_ANG_VEL, Constants.TRACK_WIDTH),
                Jeb.getAccelerationConstraint(Constants.MAX_ACCEL))
                .setTangent(Math.toRadians(45))
                .splineToLinearHeading(new Pose2d(-17, -27, Math.toRadians(45)), Math.toRadians(90))
                .build());

        motions.add(new Motion() {
            @Override
            public boolean isEnd() {
                return true;
            }

            @Override
            public void init() {
                drive.followTrajectorySequence(trajectories.get(0));
            }

            @Override
            public void run() {
            }

            @Override
            public void cleanup() { }
        });

        motions.add(new Motion() {
            @Override
            public boolean isEnd() {
                return drive.distanceSensorBack.getDistance(DistanceUnit.CM) < 50;
            }

            @Override
            public void init() {
            }

            @Override
            public void run() {
                drive.gyroDrive(0.1,0,0);
            }

            @Override
            public void cleanup() { drive.stopAllDriveMotors(); }
        });
    }

    @Override
    public void start() {
        motions.get(0).init();
    }

    @Override
    public void loop() {
        motions.get(stage).run();

        if (motions.get(stage).isEnd()) {
            motions.get(stage).cleanup();
            if (stage < motions.size() - 1) motions.get(++stage).init();
            else stop();
        }
    }
}
