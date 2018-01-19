package ca.mcgill.ecse211.lab1;

import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class PController implements UltrasonicController {

  /* Constants */
	private static final int MOTOR_SPEED = 200;
	private static final int FILTER_OUT = 10;
	
	public static final double PROPCONST = 4.0; // Proportionality constant
	public static final int MAXCORRECTION = 50; // Bound on correction to prevent stalling
	public static final int ERRORTOL = 2; // Error tolerance (cm)
	private static final int WALLDIST = 20;
	
	private final int bandCenter;
	private final int bandWidth;
	private int distance;
	private int filterControl;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;

	public PController(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, int bandCenter, int bandwidth) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.bandCenter = bandCenter;
		this.bandWidth = bandwidth;
		this.filterControl = 0;

		this.leftMotor.setSpeed(MOTOR_SPEED); // Initalize motor rolling forward
		this.rightMotor.setSpeed(MOTOR_SPEED);
		this.leftMotor.forward();
		this.rightMotor.forward();
	}

	@Override
	public void processUSData(int distance) {
		int diff;
        int leftspeed = 0;
        int rightspeed = 0;
		
		// rudimentary filter - toss out invalid samples corresponding to null
		// signal.
		// (n.b. this was not included in the Bang-bang controller, but easily
		// could have).
        if (distance >= 255 && filterControl < FILTER_OUT) {
            // bad value, do not set the distance var, however do increment the filter value
            filterControl++;
        } else if (distance >= 255) {
            // We have repeated large values, so there must actually be nothing there: leave the distance alone
            this.distance = (int) (distance / java.lang.Math.sqrt(2));
        } else {
            // distance went below 255: reset filter and leave distance alone.
            filterControl = 0;
            // adjust for sensor being at 45deg
            this.distance = (int) (distance / java.lang.Math.sqrt(2));
        }

		// Controller Actions

        if (this.distance > 60) {                     	//wall turns left, sharp turn
            this.leftMotor.setSpeed(MOTOR_SPEED/2);
            this.rightMotor.setSpeed(MOTOR_SPEED);
            this.leftMotor.forward();
            this.rightMotor.forward();

        }
        else if (this.distance < 5) { 					//emergency backup
            this.leftMotor.setSpeed(MOTOR_SPEED);
            this.rightMotor.setSpeed(MOTOR_SPEED);
            this.leftMotor.backward();
            this.rightMotor.backward();

        }
        else if (this.distance < 10) {                 	//wall turns right, sharp turn
            WallFollowingLab.leftMotor.setSpeed(MOTOR_SPEED);
            WallFollowingLab.rightMotor.setSpeed(MOTOR_SPEED/2);
            WallFollowingLab.leftMotor.forward();
            WallFollowingLab.rightMotor.forward();
        }
        else {           												// proportional correction section inspired by provided sample code                                         
            if(Math.abs(WALLDIST - this.distance) <= ERRORTOL) {        // within established acceptable error
                this.leftMotor.setSpeed(MOTOR_SPEED);
                this.rightMotor.setSpeed(MOTOR_SPEED);
                this.leftMotor.forward();
                this.rightMotor.forward();
            }
            else if((WALLDIST - this.distance) > 0) {                     // too close too wall
                diff = this.calcProp(WALLDIST - this.distance);
                leftspeed = MOTOR_SPEED + diff;
                rightspeed = MOTOR_SPEED - diff;
                this.leftMotor.setSpeed(leftspeed);
                this.rightMotor.setSpeed(rightspeed);
                this.leftMotor.forward();
                this.rightMotor.forward();
            }
            else if((WALLDIST - this.distance) < 0) {                     // too far from wall
                diff = this.calcProp(WALLDIST - this.distance);
                leftspeed = MOTOR_SPEED - diff;
                rightspeed = MOTOR_SPEED + diff;
                this.leftMotor.setSpeed(leftspeed);
                this.rightMotor.setSpeed(rightspeed);
                this.leftMotor.forward();
                this.rightMotor.forward();
            }
        }
    }


    @Override
    public int readUSDistance() {
        return this.distance;
    }
    
    //taken almost directly from provided sample code
    private int calcProp(int diff) {                                    // this method calculates the correction that has to be made
        int correction;

        if(diff < 0) {
            diff = -diff;
        }
        correction = (int) (PROPCONST * (double)diff);                 // this is where the proportionality constant is which determines how much the speed of the wheels will be affected by the distance
        if(correction >= MOTOR_SPEED) {
            correction = MAXCORRECTION;
        }
        return correction;
    }
}
