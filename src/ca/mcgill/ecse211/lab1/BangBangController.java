package ca.mcgill.ecse211.lab1;

import lejos.hardware.motor.*;

public class BangBangController implements UltrasonicController {

	private final int bandCenter;
	private final int bandWidth;
	private final int motorLow;
	private final int motorHigh;
	private final int maxFilter = 9;
	private final int minDistance = 8;
	private final int maxDistance = 80;

	private int distance;
	private int filterCounter;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;

	public BangBangController(EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor, int bandCenter, int bandWidth, int motorLow, int motorHigh) {
		// Default Constructor
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.bandCenter = bandCenter;
		this.bandWidth = bandWidth;
		this.motorLow = motorLow;
		this.motorHigh = motorHigh;
		leftMotor.setSpeed(motorHigh); // Start robot moving forward
		rightMotor.setSpeed(motorHigh);
		leftMotor.forward();
		rightMotor.forward();
		filterCounter = 0;
	}

	@Override
	public void processUSData(int distance) {
		/* Filter based on sample code provided
		 * if data is read more than maxFilter times, do not ignore it.
		 * Makes the distinction between gaps and the end of the wall.
		 */
		if((distance > maxDistance) && (filterCounter < maxFilter)) {
			filterCounter++;	//ignore value
		}
		else if(distance > maxDistance) {
			this.distance = distance;	//not an error
		}
		else {
			this.distance = distance;
			filterCounter = 0;
		}
		
		if(distance < (bandCenter - bandWidth)) {
			if(distance <= minDistance) {
				//too close so turn around
				leftMotor.setSpeed(motorLow);				
				rightMotor.setSpeed(motorLow);				

				leftMotor.forward();		
				rightMotor.backward();
			} else {	
				//realign away from wall (right)
				leftMotor.setSpeed(motorHigh);
				rightMotor.setSpeed(motorLow);				

				leftMotor.forward();						
				rightMotor.forward();	

			}
		} else if (distance > bandCenter + bandWidth){
			//realign towards wall (left)
			leftMotor.setSpeed(motorLow);
			rightMotor.setSpeed(motorHigh);

			leftMotor.forward();
			rightMotor.forward();

		} else {
			//within margin of error, straight ahead
			leftMotor.setSpeed(motorHigh);
			rightMotor.setSpeed(motorHigh);

			leftMotor.forward();
			rightMotor.forward();
		}
		
	}

	@Override
	public int readUSDistance() {
		return this.distance;
	}
}
