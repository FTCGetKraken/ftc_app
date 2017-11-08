package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.*;

/**
 * Created by eriche on 10/28/17.
 */
@TeleOp(name="KrakenBot:TeleOp", group="final")
public class KrakenBot_TeleOp extends LinearOpMode {
    KrakenBot robot = new KrakenBot(new Orientation(true,true));

    double          clawOffset  = 0.0 ;                  // Servo mid position
    final double    CLAW_SPEED  = 0.05 ;                 // sets rate to move servo
    final double    MOTOR_REDUCTION = 5;
    boolean advanced_arm_mode = false;

    ElapsedTime e = new ElapsedTime();

    public void runOpMode() {
        robot.init(hardwareMap);
        waitForStart();
        e.reset();
        double defaultLpower = -gamepad1.left_stick_y;
        double defaultRpower = -gamepad1.right_stick_y;
        //while(e.milliseconds()<1000&&opModeIsActive()) {}
        //robot.close_claw();
        while(opModeIsActive()) {
            // Drive Motors' Power
            double left_drive_power;
            double right_drive_power;

            // Arm motors' power
            double lower_arm=0;
            double upper_arm=0;

            // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
            left_drive_power = -gamepad1.left_stick_y-defaultLpower;
            right_drive_power = -gamepad1.right_stick_y-defaultRpower;

            robot.left_drive.setPower(left_drive_power);
            robot.right_drive.setPower(right_drive_power);

            // Use gamepad left & right Bumpers to open and close the claw
            if (gamepad2.right_bumper) {
                //robot.open_claw();
                robot.left_claw.setPosition(robot.left_claw.getPosition() + CLAW_SPEED);
                robot.right_claw.setPosition(robot.right_claw.getPosition() - CLAW_SPEED);
            }
            else if (gamepad2.left_bumper) {
                //robot.close_claw();
                robot.left_claw.setPosition(robot.left_claw.getPosition() - CLAW_SPEED);
                robot.right_claw.setPosition(robot.right_claw.getPosition() + CLAW_SPEED);
            }

            if(gamepad2.a) {
                robot.color_sensing_arm.setPosition(0);
            }

            if(gamepad2.back) {
                advanced_arm_mode = !advanced_arm_mode;
            }

            if(advanced_arm_mode) {
                lower_arm = gamepad2.left_stick_y;
                upper_arm = gamepad2.right_stick_y;
            } else {
                if (gamepad2.dpad_left) {
                    // Retract tentacle
                    lower_arm = -6/MOTOR_REDUCTION;
                    //upper_arm = -0.3333/MOTOR_REDUCTION;
                } else if (gamepad2.dpad_right) {
                    // Extend tentacle
                    lower_arm = 4/MOTOR_REDUCTION;
                    upper_arm = 0.6666/MOTOR_REDUCTION;
                }
                if(gamepad2.dpad_up) {
                    // Raise tentacle
                    upper_arm = 1/MOTOR_REDUCTION;

                } else if(gamepad2.dpad_down) {
                    // Lower tentacle
                    upper_arm = -1/MOTOR_REDUCTION;
                }
            }

            robot.lower_arm.setPower(lower_arm/MOTOR_REDUCTION);
            robot.upper_arm.setPower(upper_arm/MOTOR_REDUCTION);


            // Send telemetry message to signify robot running;
            telemetry.addData("claw",  "Offset = %.2f", clawOffset);
            telemetry.addData("left",  "%.2f", left_drive_power);
            telemetry.addData("right", "%.2f", right_drive_power);
            telemetry.update();
        }
    }
}
