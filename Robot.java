import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class Robot {
    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backLeft = null;
    public DcMotor backRight = null;
    public DcMotor elevatorMotor = null; 
    public Servo clawLeft = null;
    public Servo clawRight = null;
    public DcMotor intakeMotor = null;
    public Servo foundationMover = null; 
    public DcMotor armMotor = null;
    public DcMotor intakeMotor = null;
    public Servo foundation1 = null;
    public Servo foundation2 = null;

    public void init(HardwareMap hardwareMap) {

        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        clawLeft = hardwareMap.get(Servo.class, "clawLeft");
        clawRight = hardwareMap.get(Servo.class, "clawRight");
        elevatorMotor = hardwareMap.get(DcMotor.class, "elevatorMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        foundationMover = hardwareMap.get(Servo.class, "foundationMover");
        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        foundation1 = hardwareMap.get(Servo.class, "foundation1");
        foundation2 = hardwareMap.get(Servo.class, "foundation2");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Brake immedietly after joystick hits 0 instead of coasting down
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}