package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Core.HWMap;

public class FieldCentricDriveAccelerationControl extends HWMap {

    public double STRAFE_TOGGLE_FACTOR = 0.5;
    public double ROTATION_TOGGLE_FACTOR = 0.5;
    public double imuMeasure;
    public double leftBackPower;
    public double rightBackPower;
    public double rightFrontPower;
    public double leftFrontPower;
    public double rotationEffectivness = 0.7;
    public double xyEffectivness = 0.9;

    //This variable is going to store the current speed of all three directions.
    private double VectorX = 0.0;
    private double VectorY = 0.0;
    private double VectorRot = 0.0;

    // This variable will store the max accel
    private final double maxAccelDecelXY = 0.3; // VARIABLE NEEDS TO BE CHANGED: As our robot's motor speeds range from 0 to 1 the robot will accel at (maxAccelDecelXY) of the max speed per second. This is for moving forwards and backwards
    private final double maxAccelDecelRot = 0.5; // VARIABLE NEEDS TO BE CHANGED: As our robot's motor speeds range from 0 to 1 the robot will accel at (maxAccelDecelRot) of the max speed per second. This is for turning
    public void addTelemetry(){
        telemetry.addData("Left Front", leftFrontPower);
        telemetry.addData("Right Front", rightFrontPower);
        telemetry.addData("Left Back", leftBackPower);
        telemetry.addData("Right Back", rightBackPower);
        telemetry.addData("Left Front MAH", leftFrontMotor.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("Right Front MAH", rightFrontMotor.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("Left Back MAH", leftBackMotor.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("Right Back MAH", rightBackMotor.getCurrent(CurrentUnit.MILLIAMPS));
    }
    public FieldCentricDriveAccelerationControl(Telemetry telemetry, HardwareMap hardwareMap) {
        super(telemetry, hardwareMap);
    }

    public void drive(double gamepadX, double gamepadY, double gamepadRot, boolean rotationToggle, boolean strafeToggle) {
        if (rotationToggle) {
            gamepadRot *= ROTATION_TOGGLE_FACTOR;
        }
        if (strafeToggle) {
            gamepadX *= STRAFE_TOGGLE_FACTOR;
            gamepadY *= STRAFE_TOGGLE_FACTOR;
        }
        //This will store the target speed which is adjusted based on acceleration and is provided by the controller inputs
        double targetSpeedX = gamepadX * xyEffectivness;
        double targetSpeedY = gamepadY * xyEffectivness;
        double targetSpeedRot = -gamepadRot * rotationEffectivness; // Negated because the joystick works in the other direction
        // Calculate acceleration and deceleration values for each axis
        double accelX = calculateAccelDecel(VectorX, targetSpeedX, maxAccelDecelXY);
        double accelY = calculateAccelDecel(VectorY, targetSpeedY, maxAccelDecelXY);
        double accelRot = calculateAccelDecel(VectorRot, targetSpeedRot, maxAccelDecelRot);

        // Update current speeds based on acceleration and deceleration values
        VectorX += accelX;
        VectorY += accelY;
        VectorRot += accelRot;


        //Final calculations for motor speed and direction
        double adjustedTurn = VectorRot;

        double[] controllerVector = {VectorX, VectorY};

        imuMeasure = readFromIMU();

        double[] rotatedVector = rotate(controllerVector, imuMeasure);
        double rotatedX = rotatedVector[0];
        double rotatedY = rotatedVector[1];

        double theta = Math.atan2(rotatedY, rotatedX);
        double power = Math.hypot(rotatedX, rotatedY);
        double sin = Math.sin(theta - Math.PI / 4);
        double cos = Math.cos(theta - Math.PI / 4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));


        leftBackPower = power * sin / max + adjustedTurn;
        leftFrontPower = power * cos / max + adjustedTurn;
        rightBackPower = power * cos / max - adjustedTurn;
        rightFrontPower = power * sin / max - adjustedTurn;


        if ((power + Math.abs(adjustedTurn)) > 1) {
            leftFrontPower /= power + Math.abs(adjustedTurn);
            rightFrontPower /= power + Math.abs(adjustedTurn);
            leftBackPower /= power + Math.abs(adjustedTurn);
            rightBackPower /= power + Math.abs(adjustedTurn);
        }

        leftBackMotor.setPower(leftBackPower);
        leftFrontMotor.setPower(leftFrontPower);
        rightBackMotor.setPower(rightBackPower);
        rightFrontMotor.setPower(rightFrontPower);
    }

    private double calculateAccelDecel(double currentSpeed, double targetSpeed, double maxRate) {
        double speedDifference = targetSpeed - currentSpeed;

        // Determine the direction of acceleration (positive or negative). Shorthand way to write and if-else statement. Also, known as a ternary operator.
        int sign = speedDifference >= 0 ? 1 : -1;

        // Calculate the absolute value of the acceleration
        double accelDecel = Math.min(maxRate, Math.abs(speedDifference));

        // Apply the direction to the acceleration
        return sign * accelDecel;
    }
    public static double[] rotate(double[] vector, double angle) {
        final double[] newVector = {0, 0};
        newVector[0] = Math.cos(angle) * vector[0] + (-Math.sin(angle)) * vector[1];
        newVector[1] = Math.sin(angle) * vector[0] + Math.cos(angle) * vector[1];
        return newVector;

    }
}