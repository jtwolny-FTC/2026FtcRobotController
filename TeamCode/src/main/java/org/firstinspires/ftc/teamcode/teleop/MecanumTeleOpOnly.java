package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "MecanumTeleOpOnly", group = "Linear OpMode")
public class MecanumTeleOpOnly extends LinearOpMode {

    // 1. Declare the 4 Mecanum base motors
    private DcMotor leftFront = null;
    private DcMotor rightFront = null;
    private DcMotor leftRear = null;
    private DcMotor rightRear = null;

    @Override
    public void runOpMode() {
        // 2. Initialize the hardware map
        leftFront  = hardwareMap.get(DcMotor.class, "left_front_motor");
        rightFront = hardwareMap.get(DcMotor.class, "right_front_motor");
        leftRear   = hardwareMap.get(DcMotor.class, "left_rear_motor");
        rightRear  = hardwareMap.get(DcMotor.class, "right_rear_motor");

        // 3. Reverse the left side motors so positive power moves the robot forward
        // Note: Adjust depending on your specific drivetrain's physical orientation
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // 4. Read gamepad inputs
            // y is reversed on gamepad (up is negative), so we negate it
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1; // Counteract strafe imperfection
            double rx = gamepad1.right_stick_x;

            // 5. Mecanum kinematics algorithm
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            double leftFrontPower = (y + x + rx) / denominator;
            double leftRearPower = (y - x + rx) / denominator;
            double rightFrontPower = (y - x - rx) / denominator;
            double rightRearPower = (y + x - rx) / denominator;

            // 6. Set calculated power values to the motors
            leftFront.setPower(leftFrontPower);
            leftRear.setPower(leftRearPower);
            rightFront.setPower(rightFrontPower);
            rightRear.setPower(rightRearPower);

            // 7. Send telemetry data to the Driver Station
            telemetry.addData("Status", "Running");
            telemetry.addData("Front Motors", "Left: %.2f, Right: %.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Rear Motors", "Left: %.2f, Right: %.2f", leftRearPower, rightRearPower);
            telemetry.update();
        }
    }
}