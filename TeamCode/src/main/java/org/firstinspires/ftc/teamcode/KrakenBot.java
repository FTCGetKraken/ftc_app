package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.*;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by eriche on 10/28/17.
 */

public class KrakenBot {
    // Drive Motors
    public DcMotor left_drive;
    public DcMotor right_drive;

    // Tentacle Motors
    public DcMotor lower_arm;
    public DcMotor upper_arm;

    // Claw Motors
    public Servo left_claw;
    public Servo right_claw;

    //Dropdown arm motor
    public Servo color_sensing_arm;

    // Color Sensors
    public LynxI2cColorRangeSensor color_sensor;

    // Constants
    public static final double CLAW_HOME=1;
    public static final double INV_CLAW_HOME=0;

    // Encoder constants
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.14159265358979323);

    // Autonomous conditions
    static final double toJewelsPower = 0.1;
    static final double toJewelsDistance = 8.0;
    static final long toJewelsTimeout = 10;

    static final double backUpPower = 0.1;
    static final double backUpDistance = 2;
    static final long backUpTimeout = 10;

    static final double turnPower = 0.2;
    static double turnDist = 3.0;
    static final long turnTimeout = 10;

    static double rightAngleDist = 6;

    static final double horizSafeZonePower = 0.2;
    static double horizSafeZoneDistance = 20;
    static final long horizSafeZoneTimeout = 10;

    static double finalDist = 6.0;

    // Local OpMode members
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();
    public ElapsedTime runtime = new ElapsedTime();
    private Orientation o;

    // Constructor
    public KrakenBot(Orientation o) {
        this.o=o;
    }

    // Initialize hardware
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Motors
        left_drive  = hwMap.get(DcMotor.class, "left_drive");
        right_drive = hwMap.get(DcMotor.class, "right_drive");
        upper_arm  = hwMap.get(DcMotor.class, "upper_arm");
        lower_arm = hwMap.get(DcMotor.class, "lower_arm");
        color_sensor = hwMap.get(LynxI2cColorRangeSensor.class, "color");
        color_sensor.initialize();
        left_drive.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        left_drive.setPower(0);
        right_drive.setPower(0);
        lower_arm.setPower(0);
        upper_arm.setPower(0);

        // Set drive motors to run with encoders
        left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lower_arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        upper_arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lower_arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        upper_arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Define and initialize ALL installed servos.
        left_claw = hwMap.get(Servo.class, "left_claw");
        right_claw = hwMap.get(Servo.class, "right_claw");
        color_sensing_arm = hwMap.get(Servo.class, "color_tentacle");
        left_claw.setPosition(CLAW_HOME);
        right_claw.setPosition(INV_CLAW_HOME);
        color_sensing_arm.setPosition(0.0);
    }

    public double servo_conv(double degrees){return degrees/180;}
    public void close_claw(){
        left_claw.setPosition(0.2);
        right_claw.setPosition(0.8);
    }
    public void open_claw(){
        left_claw.setPosition(CLAW_HOME);
        right_claw.setPosition(INV_CLAW_HOME);
    }
    
    public void autoWithJewels() {
        if(!this.o.red) {
            turnDist = -turnDist;
            rightAngleDist = -rightAngleDist;
            horizSafeZoneDistance = -horizSafeZoneDistance;
            finalDist = -finalDist;
        }
        // move forward
        color_sensing_arm.setPosition(0.5);
        runtime.reset();
        while(runtime.milliseconds()<1000){}
        boolean redOnLeft=false;
        if(this.color_sensor.red()>20) {
            redOnLeft=true;
        }

        //if(this.o.red) {
        if(redOnLeft) {
            // Turn left
            encoder_drive(turnPower,-turnDist,-turnDist,turnTimeout);
            color_sensing_arm.setPosition(0.0);
            // Turn back
            encoder_drive(turnPower,turnDist,turnDist,turnTimeout);

        } else {
            // Turn right
            encoder_drive(turnPower,turnDist,turnDist,turnTimeout);
            color_sensing_arm.setPosition(0.0);
            // Turn back
            encoder_drive(turnPower,-turnDist,-turnDist,turnTimeout);
        }
//        } else {
//            if(redOnLeft) {
//                // Turn right
//                encoder_drive(turnPower,turnDist,turnDist,turnTimeout);
//                // Turn back
//                encoder_drive(turnPower,-turnDist,-turnDist,turnTimeout);
//
//            } else {
//                // Turn left
//                encoder_drive(turnPower,-turnDist,-turnDist,turnTimeout);
//                // Turn back
//                encoder_drive(turnPower,turnDist,turnDist,turnTimeout);
//            }
//        }

        //Move forward
        encoder_drive(horizSafeZonePower,horizSafeZoneDistance,horizSafeZoneDistance,horizSafeZoneTimeout);

        //Turn 90 degrees
        encoder_drive(turnPower,-rightAngleDist,rightAngleDist,turnTimeout);

        //Move forward
        encoder_drive(horizSafeZonePower,finalDist,finalDist,horizSafeZoneTimeout);

        if(!this.o.red) {
            encoder_drive(turnPower,finalDist,-finalDist,horizSafeZoneTimeout);
        }

        //Turn 90 degrees
        encoder_drive(turnPower,rightAngleDist,-rightAngleDist,turnTimeout);

        encoder_motor(0.2,1,1,this.lower_arm);
        encoder_motor(0.2,-1,-1,this.lower_arm);
    }

    public void encoder_motor(double speed,
                              double inches,
                              double timeoutS, DcMotor m) {
        int newTarget;


        // Determine new target position, and pass to motor controller
        newTarget = this.left_drive.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH);

        // Turn On RUN_TO_POSITION
        m.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        m.setPower(Math.abs(speed));

        m.setTargetPosition(newTarget);

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the this will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the this continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while ((runtime.seconds() < timeoutS) &&
                m.isBusy()) {
        }

        // Stop all motion;
        m.setPower(0);

        // Turn off RUN_TO_POSITION
        m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void encoder_drive(double speed,
                              double leftInches, double rightInches,
                              double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
        

        // Determine new target position, and pass to motor controller
        newLeftTarget = this.left_drive.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
        newRightTarget = this.right_drive.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
        this.left_drive.setTargetPosition(newLeftTarget);
        this.right_drive.setTargetPosition(newRightTarget);

        // Turn On RUN_TO_POSITION
        this.left_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        this.right_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        this.left_drive.setPower(Math.abs(speed));
        this.right_drive.setPower(Math.abs(speed));

        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the this will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the this continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while ((runtime.seconds() < timeoutS) &&
                (this.left_drive.isBusy() || this.right_drive.isBusy())) {
        }

        // Stop all motion;
        this.left_drive.setPower(0);
        this.right_drive.setPower(0);

        // Turn off RUN_TO_POSITION
        this.left_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.right_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
