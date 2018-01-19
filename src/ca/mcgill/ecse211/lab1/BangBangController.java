package ca.mcgill.ecse211.lab1;

import lejos.hardware.motor.*;

public class BangBangController implements UltrasonicController {

	private static final int FILTER_OUT = 10;
	
	private final int bandCenter;
	private final int bandWidth;
	private final int motorLow;
	private final int motorHigh;

	private int distance;
	private int filterControl;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;

	public BangBangController(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, int bandCenter, int bandWidth, int motorLow, int motorHigh) {
		// Default Constructor
		this.bandCenter = bandCenter;
		this.bandWidth = bandWidth;
		
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.motorLow = motorLow;
		this.motorHigh = motorHigh;
		leftMotor.setSpeed(motorHigh); // Start robot moving forward
		rightMotor.setSpeed(motorHigh);
		leftMotor.forward();
		rightMotor.forward();
		filterControl = 0;
	}

	@Override
	public void processUSData(int distance) {
		/* Filter based on sample code provided
		 * if data is read more than maxFilter times, do not ignore it.
		 * Makes the distinction between gaps and the end of the wall.
		 */
		if (distance >= 255 && filterControl < FILTER_OUT) {
            // bad value, do not set the distance var, however do increment the
            // filter value
            filterControl++;
        } else if (distance >= 255) {
            // We have repeated large values, so there must actually be nothing
            // there: leave the distance alone
        	// our sensor is at 45deg, so this returns correct values
            this.distance = (int) (distance / java.lang.Math.sqrt(2));
        } else {
            // distance went below 255: reset filter and leave
            // distance alone.
            filterControl = 0;
            this.distance = (int) (distance / java.lang.Math.sqrt(2));
        }
		if (this.distance > 40) {                                        // sharp turn left
			this.rightMotor.setSpeed(this.motorHigh);
	        this.leftMotor.setSpeed(150);
	        this.leftMotor.forward();
	        this.rightMotor.forward();
	    } 
		else if (this.distance < 5) {                                    // emergency backup
			this.leftMotor.setSpeed(this.motorHigh);
	        this.rightMotor.setSpeed(this.motorHigh);
	        this.leftMotor.backward();
	        this.rightMotor.backward();
		}
	    else if (this.distance < 10) {                                   // sharp turn right
	        this.leftMotor.setSpeed(this.motorHigh);
	        this.rightMotor.setSpeed(0);
	        this.leftMotor.forward();
	        this.rightMotor.forward();
	    }
	    else if (this.distance < (this.bandCenter - this.bandWidth)) {   // small correction when too close
	        WallFollowingLab.leftMotor.setSpeed(this.motorHigh);
	        WallFollowingLab.rightMotor.setSpeed(this.motorLow);
	        WallFollowingLab.leftMotor.forward();
	        WallFollowingLab.rightMotor.forward();
	    }
	    else if (this.distance > this.bandCenter + this.bandWidth) {     // small correction when too far
	        WallFollowingLab.leftMotor.setSpeed(this.motorLow);
	        WallFollowingLab.rightMotor.setSpeed(this.motorHigh);
	        WallFollowingLab.leftMotor.forward();
	        WallFollowingLab.rightMotor.forward();
	    }
	    else if ((this.distance > (this.bandCenter - this.bandWidth)) && (this.distance < (this.bandCenter + this.bandWidth))) {     
	        //within margin of error
	    	WallFollowingLab.leftMotor.setSpeed(this.motorHigh);
	        WallFollowingLab.rightMotor.setSpeed(this.motorHigh);
	        WallFollowingLab.leftMotor.forward();
            WallFollowingLab.rightMotor.forward();
	    }
		
	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}
}
