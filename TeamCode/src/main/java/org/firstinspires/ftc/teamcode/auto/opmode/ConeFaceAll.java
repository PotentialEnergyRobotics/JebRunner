package org.firstinspires.ftc.teamcode.auto.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.auto.Motion;
import org.firstinspires.ftc.teamcode.drive.Constants;
import org.firstinspires.ftc.teamcode.drive.Jeb;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.*;
import java.util.Arrays;
import java.util.List;

@Autonomous(name = "Cone Face All", group = "Assisted")
public class ConeFaceAll extends OpMode {
    private Jeb jeb;

    private VuforiaLocalizer vulo;
    private TFObjectDetector tfod;
    private WebcamName camfr;

    private List<String> coneLabelsArrayList = new ArrayList<String>(Arrays.asList(Constants.CONE_LABELS));
    private int parkTarget = 1;

    private int stage = 0;
    private ArrayList<Motion> motions = new ArrayList<>();

    @Override
    public void init() {
        jeb = new Jeb(hardwareMap, telemetry);
        jeb.awake();
        jeb.resetDriveEncoders();

        camfr = hardwareMap.get(WebcamName.class, "cam fr");
        vulo = jeb.initVuforia(camfr);
        tfod = jeb.initTfod(vulo);

        tfod.loadModelFromFile(Constants.CONE_MODEL_FILE, Constants.CONE_LABELS);
        tfod.activate();

        motions.add(new Motion() {
            @Override
            public boolean isEnd() {
                return parkTarget == 1 || runtime.seconds() > 3;
            }

            @Override
            public void init() {
                runtime.reset();
                telemetry.addData("direction", "right");
                jeb.driveCentimeters(parkTarget == 0 ? 0.7 * Constants.CM_PER_TILE : -0.8 * Constants.CM_PER_TILE, 0, Constants.MOVE_TPS);
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
                return runtime.seconds() > 3;
            }

            @Override
            public void init() {
                runtime.reset();
                telemetry.addData("direction", "forward");
                jeb.driveCentimeters(0, Constants.CM_PER_TILE, Constants.MOVE_TPS);
            }

            @Override
            public void run() {
            }

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
