package org.firstinspires.ftc.teamcode.auto.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.auto.Motion;
import org.firstinspires.ftc.teamcode.drive.JebRunner;

import java.util.ArrayList;

@Autonomous(group = "Drive")
public class LeftRightTesting extends OpMode {
    private JebRunner drive;

    private int stage = 0;
    private ArrayList<Motion> motions = new ArrayList<>();

    @Override
    public void init() {
        drive = new JebRunner(hardwareMap);
        Telemetry telemetry = new MultipleTelemetry(this.telemetry, FtcDashboard.getInstance().getTelemetry());

        // Align with cone stack
        motions.add(new Motion() {
            boolean isEnd = false;
            @Override
            public boolean isEnd() {
                return isEnd || drive.intakeDistanceSensor.getDistance(DistanceUnit.CM) < 25;
            }

            @Override
            public void init() {
                runtime.reset();
            }

            @Override
            public void run() {
                telemetry.addData("intake distance", drive.intakeDistanceSensor.getDistance(DistanceUnit.CM));
                if (runtime.milliseconds() < 1000) {
                    drive.gyroDrive(
                            0.1,
                            0,
                            drive.getRawExternalHeading());
                }
                if (runtime.milliseconds() >= 1000 && runtime.milliseconds() < 3000) {
                    drive.gyroDrive(
                            -0.1,
                            0,
                            drive.getRawExternalHeading());
                }
                if (runtime.milliseconds() >= 3000 && runtime.milliseconds() < 4000) {
                    drive.gyroDrive(
                            0.1,
                            0,
                            drive.getRawExternalHeading());
                }
                if (runtime.milliseconds() > 4000) {
                    drive.stopAllDriveMotors();
                    isEnd = true;
                }
            }

            @Override
            public void cleanup() {
                drive.stopAllDriveMotors();
            }
        });
        motions.add(new Motion() {
            @Override
            public boolean isEnd() {
                return false;
            }

            @Override
            public void init() {
                drive.stopAllDriveMotors();

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
