package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by eriche on 10/28/17.
 */
@Autonomous(name="KrakenBot:Auto",group="final")
public class KrakenBot_Auto extends LinearOpMode {
    public KrakenBot robot;
    public ElapsedTime runtime = new ElapsedTime();
    // Encoder constants
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.14159265358979323);

    // conditions
    static final double toJewelsPower = 0.1;
    static final double toJewelsDistance = 8.0;
    static final long toJewelsTimeout = 10;

    static final double backUpPower = 0.1;
    static final double backUpDistance = 2;
    static final long backUpTimeout = 10;

    static final double turnPower = 0.2;
    static final double turnDist = 10.0;
    static final long turnTimeout = 10;

    static final double rightAngleDist = 18*3.1415926/4;

    static final double horizSafeZonePower = 0.2;
    static final double horizSafeZoneDistance = 36;
    static final long horizSafeZoneTimeout = 10;
    
    public void runOpMode() {
        robot = new KrakenBot();
        robot.init(hardwareMap);
        waitForStart();

        // move forward
        encoder_drive(toJewelsPower,toJewelsDistance,toJewelsDistance,toJewelsTimeout);
        boolean redOnLeft=false;
        telemetry.addData("Left Sensor","R="+robot.left_sensor.red()+
                ", G="+robot.left_sensor.green()+
                ", B="+robot.left_sensor.blue());
        telemetry.addData("Right Sensor","R="+robot.right_sensor.red()+
                ", G="+robot.right_sensor.green()+
                ", B="+robot.right_sensor.blue());
        if(robot.left_sensor.red()>robot.right_sensor.red()) {
            telemetry.addData("Colors","Red, Blue");
            redOnLeft=true;
        } else if(robot.left_sensor.red()<robot.right_sensor.red()) {
            telemetry.addData("Colors","Blue, Red");
        }
        telemetry.update();

        // Back up
        encoder_drive(backUpPower,-backUpDistance,-backUpDistance,backUpTimeout);

        // Turn to knock over Jewel
        if(redOnLeft) {
            // Turn right
            encoder_drive(turnPower,-turnDist,turnDist,turnTimeout);
            // Turn back
            encoder_drive(turnPower,turnDist,-turnDist,turnTimeout);
        } else {
            // Turn left
            encoder_drive(turnPower,turnDist,-turnDist,turnTimeout);
            // Turn back
            encoder_drive(turnPower,-turnDist,turnDist,turnTimeout);
        }

        // Move backwards
        encoder_drive(toJewelsPower,-toJewelsDistance,-toJewelsDistance,toJewelsTimeout);

        //Turn 90 degrees
        encoder_drive(turnPower,rightAngleDist,-rightAngleDist,turnTimeout);

        //Move forward
        encoder_drive(horizSafeZonePower,horizSafeZoneDistance,horizSafeZoneDistance,horizSafeZoneTimeout);

        //Turn 90 degrees
        encoder_drive(turnPower,rightAngleDist,-rightAngleDist,turnTimeout);

        //Move forward
        encoder_drive(toJewelsPower,toJewelsDistance,toJewelsDistance,toJewelsTimeout);
    }

    public void encoder_drive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.left_drive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.right_drive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            robot.left_drive.setTargetPosition(newLeftTarget);
            robot.right_drive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.left_drive.setPower(Math.abs(speed));
            robot.right_drive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.left_drive.isBusy() || robot.right_drive.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Path1",  "Running to %7d :%7d", newLeftTarget,  newRightTarget);
                telemetry.addData("Path2",  "Running at %7d :%7d",
                        robot.left_drive.getCurrentPosition(),
                        robot.right_drive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.left_drive.setPower(0);
            robot.right_drive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }
}
