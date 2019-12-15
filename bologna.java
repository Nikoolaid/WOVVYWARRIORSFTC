import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Servo.Direction;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import java.util.logging.Level;
import com.qualcomm.robotcore.hardware.configuration.UnspecifiedMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Light;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.hardware.bosch.BNO055IMU;

@TeleOp(name = "Bologna", group = "Linear Opmode")

public class Bologna extends LinearOpMode {

    private double deadband(double initialVal, double deadband) {
        if ((initialVal > deadband) || (initialVal < -deadband)) {
            return initialVal;
        } else {
            return 0;
        }
    }

    public void setAll(double power) { //set all motors to same speed, enter speed
        leftUpper.setPower(power);
        rightLower.setPower(power);
        leftLower.setPower(power);
        rightUpper.setPower(power);
    }

    protected double conversion; //CONVERSION NUMBER FROM TICKS>INCHES AND VICE VERSA, IMPORTANT !!!!!!!!!!!!!!!!!!!!!!!!!!!!

    private double fl;
    private double bl;
    private double fr;
    private double br;
    private double leftY;
    private double leftX;
    private double rightY;
    private double rightX;
    protected double myOpen = 100.0;
    protected double myClosed = 90.0;

    private boolean leftStick;
    private boolean rightStick;
    private boolean bothStick;
    private boolean leftTriggerGo;
    private boolean rightTriggerGo;

    protected static DcMotor frontLeft = null;
    protected static DcMotor frontRight = null;
    protected static DcMotor backLeft = null;
    protected static DcMotor backRight = null;
    protected static DcMotor elevatorMotor = null; 
    protected Servo clawLeft = null;
    protected Servo clawRight = null;

    // Declare OpMode members/constants.
    private ElapsedTime runtime = new ElapsedTime();

    private static double MOTOR_ADJUST = 0.60;

    @Override
    public void runOpMode() {

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        clawLeft = hardwareMap.get(Servo.class, "clawLeft");
        clawRight = hardwareMap.get(Servo.class, "clawRight");
        elevatorMotor = hardwareMap.get(DcMotor.class, "elevatorMotor");

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Brake immedietly after joystick hits 0 instead of coasting down
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // initIMU();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        // ------------------------------------------------------------------- Start of
        // Match ---------------------------------------------------
        runtime.reset();

        while (opModeIsActive()) {

            // Moving the Bot
            // Mecanum drive

            double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
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

            frontLeft.setPower(v1*MOTOR_ADJUST);
            backRight.setPower(v2*MOTOR_ADJUST);
            backLeft.setPower(v3*MOTOR_ADJUST);
            frontRight.setPower(v4*MOTOR_ADJUST);

            // Weapons Things
            // Claw Controls
            if (gamepad2.left_trigger > 0) {
                leftTriggerGo = true;
            }
            if (gamepad2.right_trigger > 0) {
                rightTriggerGo = true;
            }

            elevatorMotor.setPower(deadband(gamepad2.right_stick_y , .03));

            if (rightTriggerGo) {
                clawLeft.setPosition(myOpen);
                clawRight.setPosition(myOpen);
            } else if (leftTriggerGo) {
                clawRight.setPosition(myClosed);
                clawLeft.setPosition(myClosed);
            } else {
                clawRight.setPosition(clawRight.getPosition);
                clawLeft.setPosition(clawLeft.getPosition);
            }
            telemetry.addData("Motor Power:", "(%.2f) (%.2f) (%.2f) (%.2f)", fl, fr, bl, br);
            telemetry.addData("Predicted Motor Speed:", "(%.2f) (%.2f) (%.2f) (%.2f)" , v1 * MOTOR_ADJUST, v2 * MOTOR_ADJUST, v3 * MOTOR_ADJUST, v4 * MOTOR_ADJUST);
            telemetry.addData("Claw Servo Degrees:", "(%.2f)", clawRight.getCurrentPosition);
            telemetry.addData("Ticks moved :", "(%.2f)", frontRight.getPosition());
            telemetry.update();
        }
    }

}