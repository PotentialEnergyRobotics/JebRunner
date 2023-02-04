package org.firstinspires.ftc.teamcode.auto.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.auto.Motion;
import org.firstinspires.ftc.teamcode.drive.Constants;
import org.firstinspires.ftc.teamcode.drive.JebRunner;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Autonomous(group = "drive")
public class RRTesting extends OpMode {
    private JebRunner drive;

    private int stage = 0;
    private ArrayList<Motion> motions = new ArrayList<>();
    private ArrayList<TrajectorySequence> trajectories = new ArrayList<>();

    private int parkTarget = 1;
    private List<String> coneLabelsArrayList = new ArrayList<String>(Arrays.asList(Constants.CONE_LABELS));
    private VuforiaLocalizer vulo;
    private TFObjectDetector tfod;
    private WebcamName camfr;

    private double POLE_DIST_CM = 28.0;


    @Override
    public void init() {
        drive = new JebRunner(hardwareMap);

        camfr = hardwareMap.get(WebcamName.class, "cam fr");
        vulo = drive.initVuforia(hardwareMap, camfr);
        tfod = drive.initTfod(hardwareMap, vulo);

        tfod.loadModelFromFile(Constants.CONE_MODEL_FILE, Constants.CONE_LABELS);
        tfod.activate();


        // Trajectory from start to tall pole
        Pose2d startPose = new Pose2d(-36, -64.5, Math.toRadians(135));
        trajectories.add(drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(90))
                .splineTo(new Vector2d(-17, -58), Math.toRadians(45))
                .splineToSplineHeading(new Pose2d(-15, -27, Math.toRadians(45)), Math.toRadians(90))
                .build());
//        JebRunner.getVelocityConstraint(Constants.MAX_VEL * 0.6, Constants.MAX_ANG_VEL, Constants.TRACK_WIDTH),
//                JebRunner.getAccelerationConstraint(Constants.MAX_ACCEL))

        // todos
        // add failsafe to still park if does not detect cone on intake cone from cone stack

        // Trajectory from tall pole to cone stack
        Pose2d motion4StartPose = new Pose2d(-16, -24, Math.toRadians(45));
        trajectories.add(drive.trajectorySequenceBuilder(motion4StartPose)
                .setTangent(Math.toRadians(90))
                .addDisplacementMarker(2, () -> {
                    drive.slideMotor.setTargetPosition(Constants.LOW_ARM_POS);
                    drive.slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    drive.slideMotor.setVelocity(Constants.ARM_TPS);
                })
                .splineTo(new Vector2d(-24,-15), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-64,-10, Math.toRadians(225)), Math.toRadians(180))
                .build());

        trajectories.add(drive.trajectorySequenceBuilder(trajectories.get(1).end())
                .setTangent(Math.toRadians(0))
                .waitSeconds(1)
                .splineToSplineHeading(new Pose2d(-28,-11, Math.toRadians(135)), Math.toRadians(0))
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
                drive.gyroDrive(-0.1,0, Math.toRadians(-90));
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
                        0.1*Math.signum(POLE_DIST_CM - dist),
                        Math.toRadians(-90));
            }

            @Override
            public void cleanup() { drive.stopAllDriveMotors(); }
        });

        // Release cone
        motions.add(new Motion() {
            @Override
            public boolean isEnd() {
                return runtime.seconds() > 2;
            }

            @Override
            public void init() {
                runtime.reset();
                drive.clawServoA.setPower(-Constants.DEFAULT_ARM_POWER);
                drive.clawServoB.setPower(Constants.DEFAULT_ARM_POWER);
            }

            @Override
            public void run() {

            }

            @Override
            public void cleanup() {
                drive.clawServoA.setPower(0);
                drive.clawServoB.setPower(0);
            }
        });

        // Follow trajectory 1 from tall pole to cone stack
        motions.add(new Motion() {
            @Override
            public boolean isEnd() {
                return drive.limitSlide.isPressed()
                        || drive.intakeDistanceSensor.getDistance(DistanceUnit.CM) < Constants.INTAKE_CONE_DISTANCE
                        || runtime.seconds() > 4;
            }

            @Override
            public void init() {
                drive.setPoseEstimate(motion4StartPose);
                drive.followTrajectorySequence(trajectories.get(1));

                runtime.reset();

                drive.slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                drive.slideMotor.setPower(-.4);

                drive.clawServoA.setPower(Constants.DEFAULT_ARM_POWER);
                drive.clawServoB.setPower(-Constants.DEFAULT_ARM_POWER);
            }

            @Override
            public void run() { }

            @Override
            public void cleanup() {
                drive.slideMotor.setTargetPosition(Constants.HIGH_ARM_POS);
                drive.slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                drive.slideMotor.setVelocity(Constants.ARM_TPS);

                drive.clawServoA.setPower(0);
                drive.clawServoB.setPower(0);
            }
        });


        // Follow trajectory to tall pole
        motions.add(new Motion() {
            @Override
            public boolean isEnd() {
                return true;
            }

            @Override
            public void init() {
                drive.followTrajectorySequence(trajectories.get(2));
            }

            @Override
            public void run() { }

            @Override
            public void cleanup() { }
        });

        // Align with tall poll using gyro drive
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
                drive.gyroDrive(-0.1,0, Math.toRadians(0));
            }

            @Override
            public void cleanup() { }
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
                        0.1*Math.signum(POLE_DIST_CM - dist),
                        Math.toRadians(0));
            }

            @Override
            public void cleanup() { drive.stopAllDriveMotors(); }
        });

        // Release cone
        motions.add(new Motion() {
            @Override
            public boolean isEnd() {
                return runtime.seconds() > 2;
            }

            @Override
            public void init() {
                runtime.reset();
                drive.clawServoA.setPower(-Constants.DEFAULT_ARM_POWER);
                drive.clawServoB.setPower(Constants.DEFAULT_ARM_POWER);
            }

            @Override
            public void run() {

            }

            @Override
            public void cleanup() {
                drive.clawServoA.setPower(0);
                drive.clawServoB.setPower(0);
            }
        });


        // Park
        motions.add(new Motion() {
            @Override
            public boolean isEnd() {
                return true;
            }

            @Override
            public void init() {
                Pose2d lastStartPose = new Pose2d(-24, -12, Math.toRadians(135));

                drive.setPoseEstimate(lastStartPose);
                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(lastStartPose)
                    .setTangent(Math.toRadians(270))
                    .addDisplacementMarker(2, () -> {
                        drive.slideMotor.setTargetPosition(Constants.PICKUP_ARM_POS);
                        drive.slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                        drive.slideMotor.setVelocity(Constants.ARM_TPS);
                    })
                    .splineToLinearHeading(new Pose2d((parkTarget == 0 ? -58 : (parkTarget == 2 ? -12 : -36)),
                            -15, Math.toRadians(315)), Math.toRadians(180))
                    .build()
                );
            }

            @Override
            public void run() {

            }

            @Override
            public void cleanup() {

            }
        });

    }

    @Override
    public void init_loop() {
        if (tfod != null) {
            List<Recognition> recognitions = tfod.getRecognitions();
            telemetry.addData("Objects Detected", recognitions.size());

            for (Recognition recognition : recognitions) {
                double col = (recognition.getLeft() + recognition.getRight()) / 2 ;
                double row = (recognition.getTop()  + recognition.getBottom()) / 2 ;
                double width  = Math.abs(recognition.getRight() - recognition.getLeft()) ;
                double height = Math.abs(recognition.getTop()  - recognition.getBottom()) ;

                parkTarget = coneLabelsArrayList.indexOf(recognition.getLabel());
                telemetry.addData("park target", parkTarget);

                telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100 );
                telemetry.addData("- Position (Row/Col)","%.0f / %.0f", row, col);
                telemetry.addData("- Size (Width/Height)","%.0f / %.0f", width, height);


            }
            telemetry.update();
        }
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
