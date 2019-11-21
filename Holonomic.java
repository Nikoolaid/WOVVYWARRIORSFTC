
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Servo.Direction;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
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


@TeleOp(name="Holonomic", group="Linear Opmode")

public class Holonomic extends LinearOpMode {
    
    private static DcMotor leftUpper  = null;
    private static DcMotor rightUpper = null;
    private static DcMotor leftLower  = null;
    private static DcMotor rightLower = null;
    private static DcMotor liftMotor = null;
    private static Servo servoArm = null;

    enum PowerLevel {MAX, HALF, QUARTER, STOP}; 
    private Orientation angles;
    //BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");

    /*
    private void initIMU() {
        //BNO055IMU               imu;
        //imu = hardwareMap.get(BNO055IMU.class, "imu");

        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();

        imuParameters.mode                = BNO055IMU.SensorMode.IMU;
        imuParameters.angleUnit           = BNO055IMU.AngleUnit.RADIANS;
        imuParameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled      = false;
        imu.initialize(imuParameters);
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

    }

    Orientation             lastAngles = new Orientation();
    double globalAngle, power = .30, correction;
    //private Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);


    
    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
        globalAngle = 0;
    }

    private void getAngle() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
    }

    */
    /*
    private double getAngle() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
    
        BNO055IMU.Parameters imuParameters = new BNO055IMU.Parameters();

        imuParameters.mode                = BNO055IMU.SensorMode.IMU;
        imuParameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        imuParameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuParameters.loggingEnabled      = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(imuParameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.addData("start angle", getAngle());
        telemetry.update();
    }
*/

    // Declare OpMode members/constants.
    private ElapsedTime runtime = new ElapsedTime();

    private static double MOTOR_ADJUST = 0.60;
    
    private final long BILLION = 1000000000;
    
    private static double SIDEWAYS_DRIFT_CORRECTION = 1.0;
    
    @Override
    public void runOpMode() {
        
        PowerLevel powerLevel = PowerLevel.MAX;     //Starts the robot wheels at MAX power level
        
        leftUpper  = hardwareMap.get(DcMotor.class, "leftUpper");
        rightUpper = hardwareMap.get(DcMotor.class, "rightUpper");
        leftLower  = hardwareMap.get(DcMotor.class, "leftLower");
        rightLower = hardwareMap.get(DcMotor.class, "rightLower");
        liftMotor = hardwareMap.get(DcMotor.class, "liftMotor");
        servoArm = hardwareMap.get(Servo.class, "servoArm");
                
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftUpper.setDirection(DcMotor.Direction.FORWARD);
        rightUpper.setDirection(DcMotor.Direction.REVERSE);
        leftLower.setDirection(DcMotor.Direction.FORWARD);
        rightLower.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setDirection(DcMotor.Direction.FORWARD);
        
         //Brake immedietly after joystick hits 0 instead of coasting down
        leftUpper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightUpper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftLower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //initIMU();
            
        servoArm.setPosition(0);
 
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
//------------------------------------------------------------------- Start of Match ---------------------------------------------------
        runtime.reset();
        
        // Setup a variable for each drive wheel
                    double leftUpperPower = 0;
                    double rightUpperPower = 0;
                    double leftLowerPower = 0;
                    double rightLowerPower = 0;
        
        while (opModeIsActive()) {
            
//          ************************************************ GAMEPAD 1 CONTROLS ************************************************
            
/*
            //Setting the Wheel Motor Power
            if (gamepad1.y ) {
                powerLevel = PowerLevel.MAX;
            }
             if (gamepad1.b ) {
                powerLevel = PowerLevel.HALF;
            }
             if (gamepad1.x ) {
                powerLevel = PowerLevel.QUARTER;
            }
             if (gamepad1.a) {
                powerLevel = PowerLevel.STOP;
            }
            //Can't do it directly or else the driver must hold the button in order to keep the MOTOR_ADJUST active
                if (powerLevel == PowerLevel.MAX) {
                    MOTOR_ADJUST = .75;
                }
                if (powerLevel == PowerLevel.HALF){
                    MOTOR_ADJUST = .50; 
                }
                 if (powerLevel == PowerLevel.QUARTER){
                    MOTOR_ADJUST = .25; 
                }
                //Note: *** If Gamepad2 does [START + A] this will reset the powers to 0, so one must push [X, B, Y] to get power again 
                if (powerLevel == PowerLevel.STOP){
                    MOTOR_ADJUST = 0; //EMERGENCY STOP
                }
            */
            
            if (gamepad1.a) {
                liftMotor.setPower(1);
            }
            else if (gamepad1.b) {
                liftMotor.setPower(-0.25);
            }
            else {
                liftMotor.setPower (0);
       
            }
            



                
    //Moving the Bot        
        //Robot Oriented Drive
        
            double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = -gamepad1.right_stick_x;
            final double v1 = r * Math.cos(robotAngle) + rightX;
            final double v2 = r * Math.sin(robotAngle) - rightX;
            final double v3 = r * Math.sin(robotAngle) + rightX;
            final double v4 = r * Math.cos(robotAngle) - rightX;
            
            leftUpper.setPower(v1*MOTOR_ADJUST);
            rightLower.setPower(v2*MOTOR_ADJUST);
            leftLower.setPower(v3*MOTOR_ADJUST);
            rightUpper.setPower(v4*MOTOR_ADJUST);
       
          
       /* Field centric
            float rightX = -gamepad1.right_stick_x;
            float y = gamepad1.left_stick_y;
            float x = -gamepad1.left_stick_x;
            double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
            double theta = Math.atan2(y, x);
            getAngle();
            theta = theta + angles.firstAngle;
            
            double v1 = r * Math.cos((45*Math.PI/180)-theta) + rightX;
            double v2 = r * Math.sin((135*Math.PI/180)-theta) - rightX;
            double v3 = r * Math.sin((225*Math.PI/180)-theta) + rightX;
            double v4 = r * Math.cos((315*Math.PI/180)-theta) + rightX;
            
            
            leftUpper.setPower(v1*MOTOR_ADJUST);
            rightUpper.setPower(v2*MOTOR_ADJUST);
            leftLower.setPower(v3*MOTOR_ADJUST);
            rightLower.setPower(-v4*MOTOR_ADJUST);
*/
            //double vMax = Math.max(
            //Math.max(Math.abs(v1),Math.abs(v2)),
            //Math.max(Math.abs(v3),Math.abs(v4)));
         

//          ************************************************ GAMEPAD 2 CONTROLS ************************************************
    
            //Telemetry
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftUpperPower, rightUpperPower, leftLowerPower, rightLowerPower);
            telemetry.addData("G1 Left Joystick", "X (%.2f), Y (%.2f)", (gamepad1.left_stick_x), (gamepad1.left_stick_y));
            telemetry.addData("G1 Right Joystick", "X (%.2f), Y (%.2f)", (gamepad1.right_stick_x), (gamepad1.right_stick_y));
            telemetry.addData("Motor Power", "(%.2f) --- LU (%.2f), RU (%.2f), LL (%.2f), RL (%.2f)", MOTOR_ADJUST, leftUpper.getPower(), rightUpper.getPower(), leftLower.getPower(), rightLower.getPower());

            telemetry.update();
        }
    }
}