package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.EventLoopManager;


@TeleOp(name = "MyRobot", group = "Linear Opmode")
public class MyRobot extends LinearOpMode {
    private double deadband(double initialVal, double deadband) {
        if ((initialVal > deadband) || (initialVal < -deadband)) {
            return initialVal;
        } else {
            return 0;
        }
    }

    //MAKING VARIABLES
    protected double conversion; //CONVERSION NUMBER FROM TICKS>INCHES AND VICE VERSA, IMPORTANT !!!!!!!!!!!!!!!!!!!!!!!!!!!!

    private double fl;
    private double bl;
    private double fr;
    private double br;
    private double leftY;
    private double rightY;
    private double rightX;
    protected double myOpen = 100.0;
    protected double myClosed = 90.0;

    private boolean leftStick;
    private boolean rightStick;
    private boolean bothStick;
    private boolean leftTriggerGo;
    private boolean rightTriggerGo;

    MyRobot robot = new MyRobot();

    // Declare OpMode members/constants.
    private ElapsedTime runtime = new ElapsedTime();

    private static double MOTOR_ADJUST = 0.60;

    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backLeft = null;
    public DcMotor backRight = null;
    /*
    public DcMotor elevatorMotor = null; 
    public Servo clawLeft = null;
    public Servo clawRight = null;
    public DcMotor intakeMotor = null;
    public DcMotor armMotor = null;
    public Servo foundation1 = null;
    public Servo foundation2 = null;

    public double clawOpen = 1.0;
    public double clawClosed = 0.0;
    */


    public void init(HardwareMap hardwareMap) {

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        /* clawLeft = hardwareMap.get(Servo.class, "clawLeft");
        // clawRight = hardwareMap.get(Servo.class, "clawRight");
        elevatorMotor = hardwareMap.get(DcMotor.class, "elevatorMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        //armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
    */

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Brake immedietly after joystick hits 0 instead of coasting down
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
    }
    
    
    @Override
    public void runOpMode() {

        robot.init(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // ------------------------------------------------------------------- Start of
        // Match ---------------------------------------------------
        runtime.reset();

        while (opModeIsActive()) {

            // Moving the Bot
            // Mecanum drive

            double r = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = -gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;

            if(gamepad1.y) {
                MOTOR_ADJUST = .5;
            } else if(gamepad1.a) {
                MOTOR_ADJUST = .2;
            } else {
                MOTOR_ADJUST = .6;
            }

            robot.frontLeft.setPower(v1*MOTOR_ADJUST);
            robot.frontRight.setPower(v2*MOTOR_ADJUST);
            robot.backLeft.setPower(v3*MOTOR_ADJUST);
            robot.backRight.setPower(v4*MOTOR_ADJUST);

            // Weapons Things

/*
            //elevator 
            robot.elevatorMotor.setPower(deadband(gamepad2.right_stick_y , .03));

            //intake
            if (gamepad2.right_trigger > 0) {
                intakeMotor.setPower(-.7);
            } else if (gamepad2.right_bumper) {
                intakeMotor.setPower(.7);
            } else {
                intakeMotor.setPower(0);
            }
            */
            

            telemetry.addData("Motor Power:", "(%.2f) (%.2f) (%.2f) (%.2f)", fl, fr, bl, br);
            telemetry.addData("Predicted Motor Speed:", "(%.2f) (%.2f) (%.2f) (%.2f)" , v1 * MOTOR_ADJUST, v2 * MOTOR_ADJUST, v3 * MOTOR_ADJUST, v4 * MOTOR_ADJUST);
            //telemetry.addData("Claw Servo Degrees:", "(%.2f)", robot.clawRight.getCurrentPosition);
            //telemetry.addData("Ticks moved :", "(%.2f)", robot.frontRight.getPosition());
            telemetry.update();
        }
    }
    
        public void setAll(double power) { //set all motors to same speed, enter speed
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }
}
