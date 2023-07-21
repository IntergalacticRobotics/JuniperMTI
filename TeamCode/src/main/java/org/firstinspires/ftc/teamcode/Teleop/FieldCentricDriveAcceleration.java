package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Core.HWMap;

public class FieldCentricDriveAcceleration extends HWMap {

    public double STRAFE_TOGGLE_FACTOR = 0.5;
    public double ROTATION_TOGGLE_FACTOR = 0.5;
    public double imuMeasure;
    public double leftBackPower;
    public double rightBackPower;
    public double rightFrontPower;
    public double leftFrontPower;
    public double leftBackEncoder;
    public double rightBackEncoder;
    public double rightFrontEncoder;
    public double leftFrontEncoder;

    //    public double leftBackPIDVel;
//    public double rightBackPIDVel;
//    public double rightFrontPIDVel;
//    public double leftFrontPIDVel;
//    double lfp = 0, lfi = 0, lfd;
//    double lbp = 0, lbi = 0, lbd;
//    double rfp = 0, rfi = 0, rfd;
//    double rbp = 0, rbi = 0, rbd;
    public double rotationEffectivness = 0.7;
    public double xyEffectivness = 0.9;
    public float globalPitchAngle;
    //Max Accel/Decel in Power Per Second
    public static double maxAccel = 0.05;
    public static double maxDecel = 0.05;

    // Constants for feedforward control
    private static final double ROBOT_MASS_KG = 20.0; // Adjust this value based on your robot's weight
    private static final double DESIRED_LINEAR_VELOCITY = 1.0; // Adjust this value to your desired linear velocity (meters per second)
    private static final double K_FEEDFORWARD_X = 0.2; // Feedforward coefficient for X direction
    private static final double K_FEEDFORWARD_Y = 0.2; // Feedforward coefficient for Y direction
    private static final double K_FEEDFORWARD_TURN = 0.1; // Feedforward coefficient for rotational motion

    // PID control parameters (placeholders, tune them for your robot)
    private static final double K_P = 1.0;
    private static final double K_I = 0.0;
    private static final double K_D = 0.0;

    // Variables to store previous errors for PID control
    private double prevErrorX = 0.0;
    private double prevErrorY = 0.0;
    private double prevErrorTurn = 0.0;

    // Timer for PID control calculations
    private ElapsedTime pidTimer = new ElapsedTime();

    private ElapsedTime loopTimer = new ElapsedTime();
    private PIDVelo PID = new PIDVelo();

    Orientation lastAngle = new Orientation();

    public FieldCentricDriveAcceleration(Telemetry telemetry, HardwareMap hardwareMap) {
        super(telemetry, hardwareMap);
    }

    /* public void checkifrobotnottipping() {
         if (globalPitchAngle <= 65) {
             //Here it checks if the tip angle exceeds 8 degrees
             rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
             leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
             leftFrontMotor.setPower(1.0);
             rightFrontMotor.setPower(1.0);
         } else if (globalPitchAngle >= 90) {
             rightBackMotor.setDirection(DcMotor.Direction.REVERSE);
             leftBackMotor.setDirection(DcMotor.Direction.FORWARD);
             rightBackMotor.setPower(1.0);
             leftBackMotor.setPower(1.0);
         } else {
             rightBackMotor.setDirection(DcMotor.Direction.FORWARD);
             rightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
             leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
             leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
         }
     }

     public float getPitch() {
         Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
         float deltaPitchAngle = angles.thirdAngle - lastAngle.thirdAngle;//This is subtracting roll angle
         // It's going to record angles between -180 and 180
         globalPitchAngle += deltaPitchAngle;
         lastAngle = angles;
         return globalPitchAngle;
     }
 */
    public void addTelemetry() {
        telemetry.addData("Left Front", leftFrontPower);
        telemetry.addData("Right Front", rightFrontPower);
        telemetry.addData("Left Back", leftBackPower);
        telemetry.addData("Right Back", rightBackPower);
        telemetry.addData("Left Front MAH", leftFrontMotor.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("Right Front MAH", rightFrontMotor.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("Left Back MAH", leftBackMotor.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("Right Back MAH", rightBackMotor.getCurrent(CurrentUnit.MILLIAMPS));
    }

    public void drive(double gamepadX, double gamepadY, double gamepadRot, boolean rotationToggle, boolean strafeToggle) {
        if (rotationToggle) {
            gamepadRot *= ROTATION_TOGGLE_FACTOR;
        }
        if (strafeToggle) {
            gamepadX *= STRAFE_TOGGLE_FACTOR;
            gamepadY *= STRAFE_TOGGLE_FACTOR;
        }
        // Calculate feedforward terms
        double feedforwardX = ROBOT_MASS_KG * DESIRED_LINEAR_VELOCITY * K_FEEDFORWARD_X;
        double feedforwardY = ROBOT_MASS_KG * DESIRED_LINEAR_VELOCITY * K_FEEDFORWARD_Y;
        double feedforwardTurn = ROBOT_MASS_KG * DESIRED_LINEAR_VELOCITY * K_FEEDFORWARD_TURN;

        // gamepadRot is negated because in math, a counterclockwise rotation is positive
        // (think unit circle), but on the controller, we expect the robot to rotate clockwise when
        // we push the stick to the right. Pushing the stick to the right outputs a positive value.
        double turn = -gamepadRot * rotationEffectivness + feedforwardTurn;
        double controllerX = gamepadX * xyEffectivness + feedforwardX;
        double controllerY = gamepadY * xyEffectivness + feedforwardY;
        double[] controllerVector = {controllerX, controllerY};
//        telemetry.addData("controllerVector[0]: ", controllerVector[0]);
//        telemetry.addData("controllerVector[1]: ", controllerVector[1]);

        imuMeasure = readFromIMU();

        double[] rotatedVector = rotate(controllerVector, imuMeasure);
        double rotatedX = rotatedVector[0];
        double rotatedY = rotatedVector[1];
//        telemetry.addData("rotatedX: ", rotatedX);
//        telemetry.addData("rotatedY: ", rotatedY);


        double theta = Math.atan2(rotatedY, rotatedX);
//        telemetry.addData("theta: ", theta);
        double power = Math.hypot(rotatedX, rotatedY);
//        telemetry.addData("power: ", power);
        double sin = Math.sin(theta - Math.PI / 4);
//        telemetry.addData("sin: ", sin);
        double cos = Math.cos(theta - Math.PI / 4);
//        telemetry.addData("cos: ", cos);
        double max = Math.max(Math.abs(sin), Math.abs(cos));
//        telemetry.addData("max: ", max);

        leftBackPower = power * sin / max + turn;
        leftFrontPower = power * cos / max + turn;
        rightBackPower = power * cos / max - turn;
        rightFrontPower = power * sin / max - turn;
        // Encoder values (312 RPM Motors)
        leftBackEncoder = leftBackPower;
        leftFrontEncoder = leftFrontPower;
        rightBackEncoder = rightBackPower;
        rightFrontEncoder = rightFrontPower;


        if ((power + Math.abs(turn)) > 1) {
            leftFrontPower /= power + Math.abs(turn);
            rightFrontPower /= power + Math.abs(turn);
            leftBackPower /= power + Math.abs(turn);
            rightBackPower /= power + Math.abs(turn);
        }
//        leftBackPIDVel = PID.Controller(leftBackPower, PID.ticksToSpeed(leftBackMotor.getVelocity()), lbp, lbi, lbd);
//        leftFrontPIDVel = PID.Controller(leftFrontPIDVel, PID.ticksToSpeed(leftFrontMotor.getVelocity()), lfp, lfi, lfd);
//        rightBackPIDVel = PID.Controller(rightBackPower, PID.ticksToSpeed(rightBackMotor.getVelocity()), rbp, rbi, rbd);
//        rightFrontPIDVel = PID.Controller(rightFrontPower, PID.ticksToSpeed(rightFrontMotor.getVelocity()), rfp, rfi, rfd);


        leftBackMotor.setPower(leftBackPower);
        leftFrontMotor.setPower(leftFrontPower);
        rightBackMotor.setPower(rightBackPower);
        rightFrontMotor.setPower(rightFrontPower);
        //leftBackMotor.setPower(leftBackPIDVel);
//        leftFrontMotor.setPower(leftFrontPIDVel);
//        rightBackMotor.setPower(rightBackPIDVel);
//        rightFrontMotor.setPower(rightFrontPIDVel);
        loopTimer.reset();
    }

    public static double[] rotate(double[] vector, double angle) {
        final double[] newVector = {0, 0};
        newVector[0] = Math.cos(angle) * vector[0] + (-Math.sin(angle)) * vector[1];
        newVector[1] = Math.sin(angle) * vector[0] + Math.cos(angle) * vector[1];
        return newVector;
    }
}