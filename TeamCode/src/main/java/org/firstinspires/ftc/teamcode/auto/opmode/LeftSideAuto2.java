package org.firstinspires.ftc.teamcode.auto.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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
public class LeftSideAuto2 extends OpMode {
    private JebRunner drive;

    private int stage = 0;
    private final ArrayList<Motion> motions = new ArrayList<>();
    private final ArrayList<TrajectorySequence> trajectories = new ArrayList<>();

    private int parkTarget = 1;
    private final List<String> coneLabelsArrayList = new ArrayList<String>(Arrays.asList(Constants.CONE_LABELS));
    private VuforiaLocalizer vulo;
    private TFObjectDetector tfod;
    private WebcamName camfr;

    private final double POLE_DIST_CM = 28;

    private static final int X_MOD = 1; // -1
    private static final int TAN_MOD = 0; // -180
    private static final int HEAD_MOD = 0; // -270
    private static final int POLE_OFFSET = 0; // -8

    double initialHeading; // set in start()
    double dist;
    int distCount;

    @Override
    public void init() {
        drive = new JebRunner(hardwareMap);
        telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        camfr = hardwareMap.get(WebcamName.class, "cam fr");
        vulo = drive.initVuforia(hardwareMap, camfr);
        tfod = drive.initTfod(hardwareMap, vulo);

        tfod.loadModelFromFile(Constants.CONE_MODEL_FILE, Constants.CONE_LABELS);
        tfod.activate();

        // Trajectory from start to tall pole
        Pose2d startPose = new Pose2d(-36 * X_MOD, -64.5, Math.toRadians(X_MOD * (135 + HEAD_MOD)));
        trajectories.add(drive.trajectorySequenceBuilder(startPose)
                .setTangent(Math.toRadians(X_MOD * (90 + TAN_MOD)))
                .splineTo(new Vector2d(-17 * X_MOD, -58), Math.toRadians(X_MOD * (45 + TAN_MOD)))
                .splineToSplineHeading(new Pose2d(-15 * X_MOD, -29 + POLE_OFFSET, Math.toRadians(X_MOD * (45 + HEAD_MOD))), Math.toRadians(X_MOD * (90 + TAN_MOD)))
                .build());
//        JebRunner.getVelocityConstraint(Constants.MAX_VEL * 0.6, Constants.MAX_ANG_VEL, Constants.TRACK_WIDTH),
//                JebRunner.getAccelerationConstraint(Constants.MAX_ACCEL))

        // Trajectory from tall pole to cone stack
        Pose2d firstPoleStartPose = new Pose2d(-9 * X_MOD, -24, Math.toRadians(X_MOD * (45 + HEAD_MOD)));
        trajectories.add(drive.trajectorySequenceBuilder(firstPoleStartPose)
                .setTangent(Math.toRadians(X_MOD * (90 + TAN_MOD)))
                .addDisplacementMarker(2, () -> {
                    drive.slideMotor.setTargetPosition(Constants.PICKUP_ARM_POS + 200);
                    drive.slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    drive.slideMotor.setVelocity(Constants.ARM_TPS);
                })
                .splineTo(new Vector2d(-24 * X_MOD,-15), Math.toRadians(X_MOD * (180 + TAN_MOD)))
                .splineToSplineHeading(new Pose2d(-58 * X_MOD,-12, Math.toRadians(X_MOD * (225 + HEAD_MOD))), Math.toRadians(X_MOD * (180 + TAN_MOD)))
                .build());

        // Trajectory from cone stack to centerline tall pole
        Pose2d coneStackStartPose = new Pose2d(-61 * X_MOD,-12, Math.toRadians(X_MOD * (225 + HEAD_MOD)));
        trajectories.add(drive.trajectorySequenceBuilder(coneStackStartPose)
                .setTangent(Math.toRadians(X_MOD * (0 + TAN_MOD)))
                .waitSeconds(1)
                .splineToSplineHeading(new Pose2d(-20 * X_MOD + POLE_OFFSET,-15, Math.toRadians(X_MOD * (135 + HEAD_MOD))), Math.toRadians(X_MOD * (0 + TAN_MOD)))
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
                dist = drive.distanceSensorBack.getDistance(DistanceUnit.CM);
                if (dist < 50) distCount++;
                return distCount > 4;
            }

            @Override
            public void init() { distCount = 0; }

            @Override
            public void run() {
                drive.gyroDrive(-0.12,0,
                        initialHeading + Math.toRadians(-90 * X_MOD));
            }

            @Override
            public void cleanup() { drive.stopAllDriveMotors(); }
        });

        // Place Minion over pole
        motions.add(new Motion() {
            @Override
            public boolean isEnd() {
                return true;
            }

            @Override
            public void init() {
                double startX = trajectories.get(0).end().getX();
                Pose2d lastStartPose = new Pose2d(startX, -24, Math.toRadians(X_MOD * (45 + HEAD_MOD)));

                drive.setPoseEstimate(lastStartPose);
                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(lastStartPose)
                        .lineTo(new Vector2d(startX + X_MOD * (POLE_DIST_CM - dist)/2.54, lastStartPose.getY()))
                        .build()
                );
            }

            @Override
            public void run() { }

            @Override
            public void cleanup() { }
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
                drive.clawServoA.setPower(Constants.DEFAULT_ARM_POWER);
                drive.clawServoB.setPower(-Constants.DEFAULT_ARM_POWER);
            }

            @Override
            public void run() {
                drive.stopAllDriveMotors();
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
                return true;
            }

            @Override
            public void init() {
                drive.setPoseEstimate(firstPoleStartPose);
                drive.followTrajectorySequence(trajectories.get(1));
            }

            @Override
            public void run() { }

            @Override
            public void cleanup() { }
        });

        // Approach cone stack using front distance sensor
        motions.add(new Motion() {
            @Override
            public boolean isEnd() {
                return drive.distanceSensorFront.getDistance(DistanceUnit.CM) < 15;
            }

            @Override
            public void init() { }

            @Override
            public void run() {
                drive.gyroDrive(0,0.12,
                        initialHeading + Math.toRadians(90 * X_MOD));
            }

            @Override
            public void cleanup() { drive.stopAllDriveMotors(); }
        });

        // Pick up cone
        motions.add(new Motion() {
            @Override
            public boolean isEnd() {
                return drive.limitSlide.isPressed()
                        || drive.intakeDistanceSensor.getDistance(DistanceUnit.CM) < Constants.INTAKE_CONE_DISTANCE
                        || runtime.seconds() > 3;
            }

            @Override
            public void init() {
                runtime.reset();

                drive.slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                drive.slideMotor.setPower(-.4);

                drive.clawServoA.setPower(-Constants.DEFAULT_ARM_POWER);
                drive.clawServoB.setPower(Constants.DEFAULT_ARM_POWER);
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
                drive.setPoseEstimate(coneStackStartPose);
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
                dist = drive.distanceSensorBack.getDistance(DistanceUnit.CM);
                if (dist < 50) distCount++;
                return distCount > 4;
            }

            @Override
            public void init() { distCount = 0; }

            @Override
            public void run() {
                drive.gyroDrive(-0.12,0, initialHeading);
            }

            @Override
            public void cleanup() { drive.stopAllDriveMotors(); }
        });

        // Place Minion over pole
        motions.add(new Motion() {
            @Override
            public boolean isEnd() {
                return true;
            }

            @Override
            public void init() {
                double startY = trajectories.get(2).end().getY();
                Pose2d lastStartPose = new Pose2d(-24 * X_MOD, startY, Math.toRadians(X_MOD * (135 + HEAD_MOD)));

                drive.setPoseEstimate(lastStartPose);
                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(lastStartPose)
                        .lineTo(new Vector2d(lastStartPose.getX(),startY + (POLE_DIST_CM - dist)/2.54))
                        .build()
                );
            }

            @Override
            public void run() { }

            @Override
            public void cleanup() { }
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
                drive.clawServoA.setPower(Constants.DEFAULT_ARM_POWER);
                drive.clawServoB.setPower(-Constants.DEFAULT_ARM_POWER);
            }

            @Override
            public void run() {
                drive.stopAllDriveMotors();
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
                Pose2d lastStartPose = new Pose2d(-24 * X_MOD, -9, Math.toRadians(X_MOD * (135 + HEAD_MOD)));

                drive.setPoseEstimate(lastStartPose);
                drive.followTrajectorySequence(drive.trajectorySequenceBuilder(lastStartPose)
                        .setTangent(Math.toRadians(X_MOD * (270 + TAN_MOD)))
                        .addDisplacementMarker(2, () -> {
                            drive.slideMotor.setTargetPosition(Constants.PICKUP_ARM_POS);
                            drive.slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            drive.slideMotor.setVelocity(Constants.ARM_TPS);
                        })
                        .splineToLinearHeading(new Pose2d((parkTarget == 0 ? -58 * X_MOD : (parkTarget == 2 ? -12 * X_MOD : -36 * X_MOD)),
                                -12, Math.toRadians(X_MOD * (315 + HEAD_MOD))), Math.toRadians(X_MOD * (180 + TAN_MOD)))
                        .build()
                );
            }

            @Override
            public void run() { }

            @Override
            public void cleanup() { }
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
        telemetry.addData("IMU YAW", Math.toDegrees(drive.getRawExternalHeading()));
    }

    @Override
    public void start() {
        initialHeading = drive.getRawExternalHeading();
        motions.get(0).init();
    }

    @Override
    public void loop() {
        // region telem
        telemetry.addData("distance sensor front", drive.distanceSensorBack.getDistance(DistanceUnit.CM));
        telemetry.addData("distance sensor back", drive.distanceSensorBack.getDistance(DistanceUnit.CM));
        telemetry.addData("distance sensor intake", drive.intakeDistanceSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("slide height", drive.slideMotor.getCurrentPosition());

        //endregion

        //region motions
        motions.get(stage).run();

        if (motions.get(stage).isEnd()) {
            motions.get(stage).cleanup();
            if (stage < motions.size() - 1) motions.get(++stage).init();
            else stop();
        }
        //endregion
    }
}