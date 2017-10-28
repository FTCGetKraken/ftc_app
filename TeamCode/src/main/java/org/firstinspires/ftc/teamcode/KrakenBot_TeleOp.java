package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by eriche on 10/28/17.
 */
@TeleOp(name="KrakenBot:TeleOp", group="final")
public class KrakenBot_TeleOp extends OpMode {
    KrakenBot robot = new KrakenBot();

    double          clawOffset  = 0.0 ;                  // Servo mid position
    final double    CLAW_SPEED  = 0.02 ;                 // sets rate to move servo
    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        double left;
        double right;

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;

        robot.left_drive.setPower(left);
        robot.right_drive.setPower(right);

        // Use gamepad left & right Bumpers to open and close the claw
        if (gamepad1.right_bumper)
            clawOffset += CLAW_SPEED;
        else if (gamepad1.left_bumper)
            clawOffset -= CLAW_SPEED;

        // Move both servos to new position.  Assume servos are mirror image of each other.
        clawOffset = Range.clip(clawOffset, -0.5, 0.5);
        robot.left_claw.setPosition(robot.MID_SERVO + clawOffset);
        robot.right_claw.setPosition(robot.MID_SERVO - clawOffset);

        // Use gamepad buttons to move the arm up (Y) and down (A)
        if (gamepad1.y) {
            robot.upper_arm.setPower(-2*robot.ARM_UP_POWER);
            robot.lower_arm.setPower(robot.ARM_UP_POWER);
        }
        else if (gamepad1.a) {
            robot.upper_arm.setPower(2*robot.ARM_DOWN_POWER);
            robot.lower_arm.setPower(-robot.ARM_DOWN_POWER);
        }
        else {
            robot.upper_arm.setPower(0.0);
            robot.lower_arm.setPower(0.0);
        }

        // Send telemetry message to signify robot running;
        telemetry.addData("claw",  "Offset = %.2f", clawOffset);
        telemetry.addData("left",  "%.2f", left);
        telemetry.addData("right", "%.2f", right);
    }
}
