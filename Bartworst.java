

@Autonomous (name = "Bartworst", group = "Linear Opmode")
public class Bartworst extends Bologna {

    double distanceDriven;

    @Override
    public void runOpMode() {

    }

    public void setAll(double power) { //set all motors to same speed, enter speed
        leftUpper.setPower(power);
        rightLower.setPower(power);
        leftLower.setPower(power);
        rightUpper.setPower(power);
    }

    public void driveUntil(double DISTANCE) {
        distanceDriven = frontRight.getCurrentPosition();
        while(distanceDriven < DISTANCE) {
            setAll(.5);
        } 
        setAll(0);
    }
 //ticks = x * inches
 //inches = ticks / x
    public double toInches() {
        return (frontRight.getCurrentPosition / conversion);
    }

    public double toTicks(double inches) {
        return (conversion * inches);
    }

    
}