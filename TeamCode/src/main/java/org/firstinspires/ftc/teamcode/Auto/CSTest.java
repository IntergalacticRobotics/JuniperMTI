package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Mechanism.ColorSensorMech;
import org.firstinspires.ftc.teamcode.Mechanism.ConeTransporter;
import org.firstinspires.ftc.teamcode.Auto.RR.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Auto.RR.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Auto.RR.trajectorysequence.TrajectorySequence;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class CSTest extends LinearOpMode {
    private OpenCvCamera camera;
    private Detection detection;
    private BNO055IMU imu;
    public static double imuAngle;
    static final double FEET_PER_METER = 3.28084;
    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;
    // UNITS ARE METERS
    double tagsize = 0.166;
    // Tag ID 1, 2, 3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;
    AprilTagDetection tagOfInterest = null;
    private SampleMecanumDrive drive;
    private ConeTransporter coneTransporter;
    private ColorSensorMech colorSensorMech;
    private ElapsedTime timer;
    public boolean coneTransportedSetup = false;
    public double startHeading;
    private boolean runLoop = true;
    private int numberOfCycles = 1;
    private int numberOfCones = 15;
    private ElapsedTime imuTimer = new ElapsedTime();
    private double lastIMUCall = 0.0;
    private double strafeVal = 0.0001;
    private double colorCorrection;

    /* Auto Constant Variables: **/
    private double startX = 0; // Start pos X
    private double startY = 0; // Start pos Y
    private double preJCTX = 48.0; // Preload junction deposit X value
    private double preJCTY = 12.0; // Preload junction deposit Y value
    private double stackX = 58.0; // Stack X value
    private double stackY = 12.0; // Stack Y value
    private double cycleJCTX = 48.0; // Cycle junction deposit X value
    private double cycleJCTY = 12.0; // Cycle junction deposit XY value

    /** Robot Tuning Variables: **/
    private double startXOff = -0.5; // Start pos X offset
    private double startYOff = 0.0; // Start pos Y offset
    private double preXOff = -.75; // Preload junction X offset
    private double preYOff = 3.0; // Preload junction Y offset
    private double stackXOff = 1.75; // Stack X offset
    private double stackYOff = 0.0; // Stack Y offset
    private double cycleXOff = 0.5; // Cycle junction X offset
    private double cycleYOff = 4.0; // Cycle junction X offset

    //TODO: Field Tuning Variables:
    private double autoDelay = 	0.0	; //TODO: Delay before auto starts
    private double F_preXOff = 	-0.5	; //TODO: Field Preload junction X offset
    private double F_preYOff =	0.0	; //TODO: Field Preload junction Y offset

    private double F_stackXOff1 = 	0.55	; //TODO: Stack X offset Cycle 1
    private double F_stackYOff1 = 	0.0	; //TODO: Stack Y offset Cycle 1
    private double F_stackAngOff1 = 	0.0	; //TODO: Stack Angle offset Cycle 1
    private double F_cycleXOff1 = 	-0.0	; //TODO: Field cycle junction X offset Cycle 1
    private double F_cycleYOff1 = 	-0.5	; //TODO: Field cycle junction Y offset Cycle 1
    private double F_cycleAngOff1 = 	0.0	; //TODO: cycle Angle offset Cycle 1

    private double F_stackXOff2 = 	0.55	; //TODO: Stack X offset Cycle 2
    private double F_stackYOff2 = 	0.0	; //TODO: Stack Y offset Cycle 2
    private double F_stackAngOff2 = 	0.0	; //TODO: Stack Angle offset Cycle 2
    private double F_cycleXOff2 = 	-0.0	; //TODO: Field cycle junction X offset Cycle 2
    private double F_cycleYOff2 = 	-0.5	; //TODO: Field cycle junction Y offset Cycle 2
    private double F_cycleAngOff2 = 	0.0	; //TODO: cycle Angle offset Cycle 2

    private double F_stackXOff3 = 	0.55	; //TODO: Stack X offset Cycle 3
    private double F_stackYOff3 = 	0.0	; //TODO: Stack Y offset Cycle 3
    private double F_stackAngOff3 = 	0.0	; //TODO: Stack Angle offset Cycle 3
    private double F_cycleXOff3 = 	0.0	; //TODO: Field cycle junction X offset Cycle 3
    private double F_cycleYOff3 = 	-0.5	; //TODO: Field cycle junction Y offset Cycle 3
    private double F_cycleAngOff3 = 	0.0	; //TODO: cycle Angle offset Cycle 3

    @Override
    public void runOpMode() {
        drive = new SampleMecanumDrive(hardwareMap);
        coneTransporter = new ConeTransporter(telemetry, hardwareMap);
        colorSensorMech = new ColorSensorMech(telemetry, hardwareMap);
        timer = new ElapsedTime();
        imu = this.hardwareMap.get(BNO055IMU.class, "imu");

        initializeIMU();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        detection = new Detection(tagsize, fx, fy, cx, cy);
        camera.setPipeline(detection);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode) {
            }
        });
        telemetry.setMsTransmissionInterval(50);

        while (!isStarted() && !isStopRequested()) {
            for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
                module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            }
            ArrayList<AprilTagDetection> currentDetections = detection.getLatestDetections();
            if (!coneTransportedSetup) {
                coneTransporter.unretractOdometryServos();
                coneTransporter.linearSlides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                coneTransporter.setGripperPosition(.75);
                sleep(2000);
                sleep(2000);
                coneTransportedSetup = true;
            }

            if (currentDetections.size() != 0) {
                boolean tagFound = false;
                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }
                if (tagFound) {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                } else {
                    telemetry.addLine("Don't see tag of interest :(");
                    if (tagOfInterest == null) {
                        telemetry.addLine("(The tag has never been seen)");
                    } else {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }
            } else {
                telemetry.addLine("Don't see tag of interest :(");
                if (tagOfInterest == null) {
                    telemetry.addLine("(The tag has never been seen)");
                } else {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }
            }
            telemetry.update();
            sleep(20);
        }
        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */
        /* Update the telemetry */
        if (tagOfInterest != null) {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        } else {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }
        /* Start Loop */
        double numericalTag = 0;
        if (tagOfInterest != null) {
            if (tagOfInterest.id == LEFT) {
                numericalTag = tagOfInterest.id - 2;
            } else if (tagOfInterest.id == MIDDLE) {
                numericalTag = 0.001;
            } else if (tagOfInterest.id == RIGHT) {
                numericalTag = tagOfInterest.id - 2;
            }
        } else{
            numericalTag = -1;
        }

        startHeading = Math.toRadians(270);
        drive.setPoseEstimate(new Pose2d(startX, startY, startHeading));
        drive.setExternalHeading(startHeading);
        TrajectorySequence ColorTest = drive.trajectorySequenceBuilder(new Pose2d(startX, startY, startHeading))
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    strafeVal = colorSensorMech.isDetectingBlueLine();
                    telemetry.addData("ColorSensorStatus", strafeVal);
                    telemetry.addData("RightCS", colorSensorMech.rightBlue);
                    telemetry.addData("LeftCS", colorSensorMech.leftBlue);
                    telemetry.addData("MiddleCS", colorSensorMech.middleBlue);
                })
                .waitSeconds(2)
                .strafeRight(strafeVal)
                .waitSeconds(3)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    strafeVal = colorSensorMech.isDetectingBlueLine();
                    telemetry.addData("ColorSensorStatus", strafeVal);
                    telemetry.addData("RightCS", colorSensorMech.rightBlue);
                    telemetry.addData("LeftCS", colorSensorMech.leftBlue);
                    telemetry.addData("MiddleCS", colorSensorMech.middleBlue);
                })
                .strafeRight(strafeVal)
                .waitSeconds(3)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {
                    strafeVal = colorSensorMech.isDetectingBlueLine();
                    telemetry.addData("ColorSensorStatus", strafeVal);
                    telemetry.addData("RightCS", colorSensorMech.rightBlue);
                    telemetry.addData("LeftCS", colorSensorMech.leftBlue);
                    telemetry.addData("MiddleCS", colorSensorMech.middleBlue);
                    telemetry.update();
                })
                .strafeRight(strafeVal)

                .build();
        drive.followTrajectorySequenceAsync(ColorTest);

        /*startHeading = Math.toRadians(0);
        drive.setPoseEstimate(new Pose2d(startX, startY, startHeading));
        drive.setExternalHeading(startHeading);
        TrajectorySequence ColorTest = drive.trajectorySequenceBuilder(new Pose2d(startX, startY, startHeading))
                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                    drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY()+colorSensorMech.isDetectingBlueLine(), drive.getPoseEstimate().getHeading()));
                })
                .forward(4)
                .build();
        drive.followTrajectorySequenceAsync(ColorTest);
*/
        while(opModeIsActive()){
            idle();
            for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
                module.clearBulkCache();
            }
            //coneTransporter.retractOdometryServos();
            imuAngle = readFromIMU();
            coneTransporter.loop();
            //telemetry.update();
            drive.update();
            //if(imuTimer.time() - lastIMUCall >= .1 && drive.getPoseVelocity().vec().norm() < 5.0) {
            //telemetry.addData("IMU Angle", IMUHeading.imuAngle);
            //lastIMUCall = imuTimer.time();
            //drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), drive.getExternalHeading()));
            //}
        }
    }
    void tagToTelemetry(AprilTagDetection detection) {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }
    public double readFromIMU() {
        return drive.getRawExternalHeading();
    }
    public void initializeIMU() {
        // don't touch please
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
    }



}