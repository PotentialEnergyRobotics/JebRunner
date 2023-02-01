package org.firstinspires.ftc.teamcode.auto.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.Jeb;

@Autonomous(group = "drive")
public class DistanceSensorTesting extends OpMode {
    Jeb jeb;

    @Override
    public void init() {
        jeb = new Jeb(hardwareMap);

    }

    @Override
    public void loop() {
        telemetry.addData("distance", jeb.distanceSensorBack.getDistance(DistanceUnit.CM));

    }
}
