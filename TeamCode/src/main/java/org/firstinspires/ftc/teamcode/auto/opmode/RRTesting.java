package org.firstinspires.ftc.teamcode.auto.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.auto.Motion;
import org.firstinspires.ftc.teamcode.drive.Constants;
import org.firstinspires.ftc.teamcode.drive.JebRunner;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;

@Autonomous(group = "drive")
public class RRTesting extends OpMode {
    private int stage = 0;
    private ArrayList<Motion> motions = new ArrayList<>();
    private ArrayList<TrajectorySequence> trajectories = new ArrayList<>();

    private JebRunner drive;

    private double POLE_DIST_CM = 30.0;

    @Override
    public void init() {
        drive = new JebRunner(hardwareMap);

        // Trajectory from start to tall pole
        Pose2d startPose = new Pose2d(-36, -64.5, Math.toRadians(135));
        trajectories.add(drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(90))
                .splineTo(new Vector2d(-17, -58), Math.toRadians(45))
                .splineToSplineHeading(new Pose2d(-15, -27, Math.toRadians(45)), Math.toRadians(90))
                .build());
//        JebRunner.getVelocityConstraint(Constants.MAX_VEL * 0.6, Constants.MAX_ANG_VEL, Constants.TRACK_WIDTH),
//                JebRunner.getAccelerationConstraint(Constants.MAX_ACCEL))


        // Trajectory from tall pole to cone stack
        Pose2d motion4StartPose = new Pose2d(-16, -24, Math.toRadians(45));
        trajectories.add(drive.trajectorySequenceBuilder(motion4StartPose)
                .setTangent(Math.toRadians(90))
                .addDisplacementMarker(2, () -> {
                    drive.slideMotor.setPower(-Constants.MIN_ARM_POWER);
                })
                .splineTo(new Vector2d(-24,-15), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-60,-12, Math.toRadians(225)), Math.toRadians(180))
                .build());

        // Follow trajectory 0 from start to tall pole
        motions.add(new Motion() {
            @Override
            public boolean isEnd() {
                return true;
            }

            @Override
            public void init() {
                drive.slideMotor.setTargetPosition(Constants.HIGH_ARM_POS);
                drive.slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.slideMotor.setVelocity(Constants.ARM_TPS);

                drive.setPoseEstimate(startPose);
                drive.followTrajectorySequence(trajectories.get(0));
            }

            @Override
            public void run() { }

            @Override
            public void cleanup() { }
        });

        // Line up at tall pole
        motions.add(new Motion() {
            @Override
            public boolean isEnd() {
                return drive.distanceSensorBack.getDistance(DistanceUnit.CM) < 50;
            }

            @Override
            public void init() { }

            @Override
            public void run() {
                drive.gyroDrive(0.1,0, Math.toRadians(-90));
            }

            @Override
            public void cleanup() { drive.stopAllDriveMotors(); }
        });

        // Place Minion above pole
        motions.add(new Motion() {
            @Override
            public boolean isEnd() {
                return Math.abs(drive.distanceSensorBack.getDistance(DistanceUnit.CM) - POLE_DIST_CM) < 1;
            }

            @Override
            public void init() { }

            @Override
            public void run() {
                double dist = drive.distanceSensorBack.getDistance(DistanceUnit.CM);
                // go forward if too close to rear pole, backward if too far
                drive.gyroDrive(
                        0,
                        -0.1*Math.signum(POLE_DIST_CM - dist),
                        Math.toRadians(-90));
            }

            @Override
            public void cleanup() { drive.stopAllDriveMotors(); }
        });

        // Follow trajectory 1 from tall pole to cone stack
        motions.add(new Motion() {
            @Override
            public boolean isEnd() {
                return drive.limitSlide.isPressed() || drive.intakeDistanceSensor.getDistance(DistanceUnit.CM) < Constants.INTAKE_CONE_DISTANCE;
            }

            @Override
            public void init() {
                drive.setPoseEstimate(motion4StartPose);
                drive.followTrajectorySequence(trajectories.get(1));
            }

            @Override
            public void run() { }

            @Override
            public void cleanup() { }
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
