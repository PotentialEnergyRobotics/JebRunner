package org.firstinspires.ftc.teamcode.auto.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.JebRunner;

@Autonomous(group = "drive")
public class DistanceSensorTesting extends OpMode {
    JebRunner jeb;

    @Override
    public void init() {
        jeb = new JebRunner(hardwareMap);

    }

    @Override
    public void loop() {
        telemetry.addData("distance", jeb.distanceSensorBack.getDistance(DistanceUnit.CM));
        telemetry.addData("distance", jeb.intakeDistanceSensor.getDistance(DistanceUnit.CM));
        telemetry.addData("IMU yaw", jeb.getRawExternalHeading());

//        jeb.gyroDrive(0.1,0.1,Math.toRadians(90));
    }
}
