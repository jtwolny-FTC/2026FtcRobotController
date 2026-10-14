/* This is updated code as of 10/19/2025 */
package org.firstinspires.ftc.teamcode.teleop;

import static org.firstinspires.ftc.teamcode.ChassisConstants.LEFT_FRONT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.ChassisConstants.LEFT_REAR_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.ChassisConstants.RIGHT_FRONT_MOTOR_NAME;
import static org.firstinspires.ftc.teamcode.ChassisConstants.RIGHT_REAR_MOTOR_NAME;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.BallLoadingServo;
import org.firstinspires.ftc.teamcode.subsystems.IntakeServo;
import org.firstinspires.ftc.teamcode.subsystems.IntakeMotor;
//import org.firstinspires.ftc.teamcode.subsystems.ShootingDirectionServo;
import org.firstinspires.ftc.teamcode.subsystems.ShootingSystem;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

@TeleOp(name = "Robot Centric TeleOp")
public class RobotCentricTeleOp extends NextFTCOpMode {

    private ShootingSystem shootingSystem;
    //private ShootingDirectionServo shootingDirectionServo;
    private BallLoadingServo ballLoadingServo;
    private IntakeServo intakeSystem;
    private IntakeMotor intakeFeed;

    public RobotCentricTeleOp() {

        shootingSystem = ShootingSystem.getInstance(telemetry);
        intakeSystem = IntakeServo.getInstance(telemetry);
        intakeFeed = IntakeMotor.getInstance(telemetry);
        //shootingDirectionServo = ShootingDirectionServo.getInstance(telemetry);
        ballLoadingServo = BallLoadingServo.getInstance(telemetry);

        addComponents(
                new SubsystemComponent(shootingSystem),
                new SubsystemComponent(intakeSystem),
                new SubsystemComponent(intakeFeed),
                //new SubsystemComponent(shootingDirectionServo),
                new SubsystemComponent(ballLoadingServo),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }

    private final MotorEx frontLeftMotor = new MotorEx(LEFT_FRONT_MOTOR_NAME).reversed();
    private final MotorEx frontRightMotor = new MotorEx(RIGHT_FRONT_MOTOR_NAME);
    private final MotorEx backLeftMotor = new MotorEx(LEFT_REAR_MOTOR_NAME).reversed();
    private final MotorEx backRightMotor = new MotorEx(RIGHT_REAR_MOTOR_NAME);

    @Override
    public void onStartButtonPressed() {
        
        telemetry.addLine("START BUTTON PRESSED");
        telemetry.addData("IntakeMotor", intakeFeed != null);
        telemetry.addData("IntakeServo", intakeSystem != null);
        telemetry.addData("Shooter", shootingSystem != null);
        telemetry.update();
        
        Command driverControlled = new MecanumDriverControlled(
                frontLeftMotor,
                frontRightMotor,
                backLeftMotor,
                backRightMotor,
                Gamepads.gamepad1().leftStickY(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX()
        );

        driverControlled.schedule();

        // Shooter Controls
        // round buttons x,y,a,b
        // dpadUp,dpadDown,dpadLeft,dpadRight
        // leftBumper,rightBumper leftTrigger & rightTrigger
        // leftStickButton,rightStickButton

        Gamepads.gamepad2().a()
                .whenBecomesTrue(shootingSystem.startStop);

        Gamepads.gamepad2().leftBumper()
                .whenBecomesTrue(shootingSystem.decreaseShootingPower);

        Gamepads.gamepad2().rightBumper()
                .whenBecomesTrue(shootingSystem.increaseShootingPower);

        // Intake Servo Controls

        Gamepads.gamepad2().x()
                .whenBecomesTrue(intakeSystem.startStop);

        Gamepads.gamepad2().y()
                .whenBecomesTrue(intakeSystem.reverse);

        // NEW Intake Motor Controls

        Gamepads.gamepad2().leftStickButton()
                .whenBecomesTrue(intakeFeed.startStop);

        Gamepads.gamepad2().rightStickButton()
                .whenBecomesTrue(intakeFeed.reverse);

        // Stop Everything

        Gamepads.gamepad2().back()
                .whenBecomesTrue(shootingSystem.stopAllSubsystems);

        // Ball Loader

        Gamepads.gamepad2().dpadLeft()
                .whenBecomesTrue(ballLoadingServo.runBackward());

        Gamepads.gamepad2().dpadRight()
                .whenBecomesTrue(ballLoadingServo.stopContinuous());

        // Shooter Angle

//        Gamepads.gamepad2().dpadUp()
//                .whenBecomesTrue(shootingDirectionServo.downShootingServo);

//        Gamepads.gamepad2().dpadDown()
//                .whenBecomesTrue(shootingDirectionServo.upShootingServo);
    }
}
