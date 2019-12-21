

@Autonomous (name = "Bartworst", group = "Linear Opmode")
public class Bartworst extends LinearOpMode {

    double distanceDriven;

    Robot robot = new Robot();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

    }

    public void setAll(double power) { //set all motors to same speed, enter speed
        robot.frontLeft.setPower(power);
        robot.backRight.setPower(power);
        robot.backLeft.setPower(power);
        robot.frontRight.setPower(power);
    }

    public void driveUntil(double DISTANCE) {
        distanceDriven = robot.frontRight.getCurrentPosition();
        while(distanceDriven < DISTANCE) {
            setAll(.5);
        } 
        setAll(0);
    }

    public void turn(double degrees) {
        if( degrees > 0 ) {
            while()

        }
        robot.frontRight.setPower(.5);
        robot.backRight.setPower(.5);
        robot.backLeft.setPower(-.5);
        robot.frontLeft.setPower(-.5);
    }


 //ticks = x * inches
 //inches = ticks / x
    public double toInches() {
        return (robot.frontRight.getCurrentPosition / conversion);
    }

    public double toTicks(double inches) {
        return (conversion * inches);
    }

    
}