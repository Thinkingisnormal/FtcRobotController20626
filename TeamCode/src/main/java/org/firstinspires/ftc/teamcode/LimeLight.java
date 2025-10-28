package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.limelightvision.*;
import com.qualcomm.hardware.limelightvision.LLResultTypes.*;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import java.util.List;

@Autonomous(name = "LimeLight Auto", group = "Vision")
public class LimeLight extends LinearOpMode {

    private Limelight3A limelight;
    private BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        // --- Initialize Limelight and IMU ---
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);

        telemetry.addLine("Limelight initialized — waiting for start...");
        telemetry.update();

        waitForStart();

        //Main autonomous vision logic
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            telemetry.addData("Pipeline", result.getPipelineIndex());

            double tx = result.getTx();
            double ty = result.getTy();
            double ta = result.getTa();

            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
            telemetry.addData("Target Area", ta);

            // Example: Use tx to align the robot
            if (tx < -5) {
                telemetry.addLine("Target is left — turn left");
                // yourMotorControlMethod(-0.3, 0.3);
            } else if (tx > 5) {
                telemetry.addLine("Target is right — turn right");
                // yourMotorControlMethod(0.3, -0.3);
            } else {
                telemetry.addLine("Target centered — move forward");
                // yourMotorControlMethod(0.4, 0.4);
            }

            //Python integration
            double[] inputs = {1.0, 2.0, 3.0, 4.0};
            limelight.updatePythonInputs(inputs);

            double[] pythonOutputs = result.getPythonOutput();
            if (pythonOutputs != null && pythonOutputs.length > 0) {
                telemetry.addData("Python output[0]", pythonOutputs[0]);
            }

            //Pose estimation
            Pose3D botpose = result.getBotpose();
            if (botpose != null) {
                telemetry.addData("BotPose MT1", "(%.2f, %.2f)",
                        botpose.getPosition().x, botpose.getPosition().y);
            }

            double robotYaw = imu.getAngularOrientation().firstAngle;
            limelight.updateRobotOrientation(robotYaw);

            Pose3D botpose_mt2 = result.getBotpose_MT2();
            if (botpose_mt2 != null) {
                telemetry.addData("BotPose MT2", "(%.2f, %.2f)",
                        botpose_mt2.getPosition().x, botpose_mt2.getPosition().y);
            }

            //Fiducials (AprilTags)
            List<FiducialResult> fiducials = result.getFiducialResults();
            for (FiducialResult fiducial : fiducials) {
                int id = fiducial.getFiducialId();
                double fx = fiducial.getTargetXDegrees();
                double fy = fiducial.getTargetYDegrees();
                double distance = fiducial.getRobotPoseTargetSpace().getY();
                telemetry.addData("Fiducial " + id,
                        String.format("X: %.1f Y: %.1f Dist: %.2f m", fx, fy, distance));
            }

            //Color Results
            List<ColorResult> colorTargets = result.getColorResults();
            for (ColorResult colorTarget : colorTargets) {
                telemetry.addData("Color Target",
                        String.format("X: %.1f Y: %.1f Area: %.1f%%",
                                colorTarget.getTargetXDegrees(),
                                colorTarget.getTargetYDegrees(),
                                colorTarget.getTargetArea()));
            }

            //Scan Results
            List<BarcodeResult> barcodes = result.getBarcodeResults();
            for (BarcodeResult barcode : barcodes) {
                telemetry.addData("Barcode", barcode.getData() + " (" + barcode.getFamily() + ")");
            }

            //Classifier Results
            List<ClassifierResult> classifications = result.getClassifierResults();
            for (ClassifierResult classification : classifications) {
                telemetry.addData("Classifier",
                        classification.getClassName() + " (" + classification.getConfidence() + "%)");
            }

            //Detector Results
            List<DetectorResult> detections = result.getDetectorResults();
            for (DetectorResult detection : detections) {
                telemetry.addData("Detector",
                        detection.getClassName() + String.format(" (%.1f, %.1f)°",
                                detection.getTargetXDegrees(), detection.getTargetYDegrees()));
            }

            //Staleness check
            long staleness = result.getStaleness();
            telemetry.addData("Data Age (ms)", staleness);
            telemetry.addData("Data Status", (staleness < 100) ? "Fresh" : "Old");

            //Field map upload example
            LLFieldMap fieldMap = new LLFieldMap();
            boolean success = limelight.uploadFieldmap(fieldMap, null);
            telemetry.addData("Field Map Upload", success ? "Success" : "Failed");

            //Snapshot
            limelight.captureSnapshot("auto_snapshot");
        } else {
            telemetry.addData("Limelight", "No targets detected!");
        }

        telemetry.update();

        //Optional cleanup
        limelight.deleteSnapshots();
    }
}