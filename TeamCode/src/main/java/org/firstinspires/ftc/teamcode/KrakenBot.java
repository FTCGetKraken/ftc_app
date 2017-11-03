package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.*;

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

    // Color Sensors
    public LynxI2cColorRangeSensor left_sensor;
    public LynxI2cColorRangeSensor right_sensor;

    // Constants
    public static final double CLAW_HOME=1;
    public static final double INV_CLAW_HOME=0;

    // Local OpMode members
    HardwareMap hwMap = null;
    private ElapsedTime period = new ElapsedTime();

    // Constructor
    public KrakenBot() {}

    // Initialize hardware
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Motors
        left_drive  = hwMap.get(DcMotor.class, "left_drive");
        right_drive = hwMap.get(DcMotor.class, "right_drive");
        upper_arm  = hwMap.get(DcMotor.class, "upper_arm");
        lower_arm = hwMap.get(DcMotor.class, "lower_arm");
        left_sensor = hwMap.get(LynxI2cColorRangeSensor.class, "left_color");
        right_sensor = hwMap.get(LynxI2cColorRangeSensor.class, "right_color");
        left_sensor.initialize();
        right_sensor.initialize();
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
        lower_arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Define and initialize ALL installed servos.
        left_claw = hwMap.get(Servo.class, "left_claw");
        right_claw = hwMap.get(Servo.class, "right_claw");
        left_claw.setPosition(CLAW_HOME);
        right_claw.setPosition(INV_CLAW_HOME);
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
}
