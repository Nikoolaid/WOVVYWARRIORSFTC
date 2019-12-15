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


@Autonomous(name = "autoAttempt", group = "Autonomous")

public class autoAttempt extends Bologna {

    private double deadband(double initialVal, double deadband) {
        if ((initialVal > deadband) || (initialVal < -deadband)) {
            return initialVal;
        } else {
            return 0;
        }
    }


    // Declare OpMode members/constants.
    private ElapsedTime runtime = new ElapsedTime();

    private static double MOTOR_ADJUST = 0.60;

    private double tickConvert = 1.0; //this is the number of ticks in an inch

    @Override
    public void runOpMode() {

        
        

        waitForStart();
        //start of match

        while (opModeIsActive()) {


            telemetry.addData("Motor Power:", "(%.2f) (%.2f) (%.2f) (%.2f)", fl, fr, bl, br);
            telemetry.addData("Predicted Motor Speed:", "(%.2f) (%.2f) (%.2f) (%.2f)" , v1 * MOTOR_ADJUST, v2 * MOTOR_ADJUST, v3 * MOTOR_ADJUST, v4 * MOTOR_ADJUST);
            telemetry.addData("Claw Servo Degrees:", "(%.2f)", clawRight.getCurrentPosition);
            telemetry.addData("Ticks moved :", "(%.2f)", frontRight.getPosition());
            telemetry.update();
        }
    }

    /**
     * 
     * @param inches - number of inches you would like to convert
     * @return number of ticks are in the entered number of inches
     */

    private double inchesToTicks(double inches) {
        return inches * tickConvert;
    }

    /**
     * 
     * @param ticks - number of ticks you would like to convert
     * @return number of inches that are in the entered number of ticks
     */

    private double ticksToInches(double ticks) {
        return ticks / tickConvert;
    }

    
    private void driveForward( double distance ) {
        double inTicks = inchesToTicks(distance);
        while (frontRight.getCurrentPosition() < inTicks) {
            bologna.setAll(.6);
        }
    }


}