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
    private double rightY;
    private double rightX;
    protected double myOpen = 100.0;
    protected double myClosed = 90.0;

    private boolean leftStick;
    private boolean rightStick;
    private boolean bothStick;
    private boolean leftTriggerGo;
    private boolean rightTriggerGo;

    Robot robot = new Robot();

    // Declare OpMode members/constants.
    private ElapsedTime runtime = new ElapsedTime();

    private static double MOTOR_ADJUST = 0.60;

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

            robot.frontLeft.setPower(v1*MOTOR_ADJUST);
            robot.backRight.setPower(v2*MOTOR_ADJUST);
            robot.backLeft.setPower(v3*MOTOR_ADJUST);
            robot.frontRight.setPower(v4*MOTOR_ADJUST);

            // Weapons Things
            // Claw Controls
            //bumper open, trigger close
            
                leftBumperGo = gamepad2.left_bumper;
            
            if (gamepad2.left_trigger > 0) {
                leftTriggerGo = true;
            }

            robot.elevatorMotor.setPower(deadband(gamepad2.right_stick_y , .03));

            if (leftBumperGo) {
                clawRight.setPosition(myOpen);
                clawLeft.setPosition(myOpen);
            } else if (leftTriggerGo) {
                clawRight.setPosition(myClosed);
                clawLeft.setPosition(myClosed);
            }


            telemetry.addData("Motor Power:", "(%.2f) (%.2f) (%.2f) (%.2f)", fl, fr, bl, br);
            telemetry.addData("Predicted Motor Speed:", "(%.2f) (%.2f) (%.2f) (%.2f)" , v1 * MOTOR_ADJUST, v2 * MOTOR_ADJUST, v3 * MOTOR_ADJUST, v4 * MOTOR_ADJUST);
            telemetry.addData("Claw Servo Degrees:", "(%.2f)", robot.clawRight.getCurrentPosition);
            telemetry.addData("Ticks moved :", "(%.2f)", robot.frontRight.getPosition());
            telemetry.update();
        }
    }

}