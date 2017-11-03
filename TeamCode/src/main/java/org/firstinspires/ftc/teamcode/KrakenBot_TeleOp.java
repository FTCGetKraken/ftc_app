package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by eriche on 10/28/17.
 */
@TeleOp(name="KrakenBot:TeleOp", group="final")
public class KrakenBot_TeleOp extends LinearOpMode {
    KrakenBot robot = new KrakenBot();

    double          clawOffset  = 0.0 ;                  // Servo mid position
    final double    CLAW_SPEED  = 0.02 ;                 // sets rate to move servo

    ElapsedTime e = new ElapsedTime();

    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();
        e.reset();
        //while(e.milliseconds()<1000&&opModeIsActive()) {}
        //robot.close_claw();
        while(opModeIsActive()) {
            double left;
            double right;

            double fwd;
            double up;

            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
            left = -gamepad1.left_stick_y;
            right = -gamepad1.right_stick_y;

            robot.left_drive.setPower(left);
            robot.right_drive.setPower(right);

            // Use gamepad left & right Bumpers to open and close the claw
            if (gamepad1.right_bumper)
                robot.open_claw();
            else if (gamepad1.left_bumper)
                robot.close_claw();

            fwd = gamepad2.left_stick_y;
            up = gamepad2.right_stick_y;

            robot.lower_arm.setPower(fwd/2);
            robot.upper_arm.setPower(up/2);

            // Send telemetry message to signify robot running;
            telemetry.addData("claw",  "Offset = %.2f", clawOffset);
            telemetry.addData("left",  "%.2f", left);
            telemetry.addData("right", "%.2f", right);
        }
    }
}
