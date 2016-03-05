#include <avr/EEPROM.h>
#include <phys253.h>
#include <LiquidCrystal.h>

int lastMotorTest;

//Pin Locations
//Anlog Pins
const int RIGHT_QRD_PIN = 2;
const int LEFT_QRD_PIN = 0;
const int MARKER_TAPE_QRD_PIN = 1;
const int ARM_POSITION_PIN = 5;
const int LEFT_IR_PIN = 3;
const int RIGHT_IR_PIN = 4;
const int PET_DISTANCE_SENSOR_PIN = 6;
//Digital Pins
const int RIGHT_WHEEL_ENCODER_PIN = 0;
const int LEFT_WHEEL_ENCODER_PIN = 1;
const int PET_SWITCH_PIN = 0;
const int FLORENCE_START_PIN = 3;
const int PET_6_RIGHT_SWITCH = 5;
const int PET_6_LEFT_SWITCH = 6;
//Motor Pins
const int LEFT_MOTOR_PIN = 1;
const int RIGHT_MOTOR_PIN = 0;
const int ARM_MOTOR_PIN = 3;
const int PET_DROP_SERVO_PIN = 0;
const int SWEEPER_ARM_PIN = 1;
const int PET_6_SERVO_PIN = 2;

//Pet Distance Constants
const int MIN_PET_DISTANCE = 100;
const int MAX_PET_DISTANCE = 300;
const int NOISE_THRESHOLD = 130;
const int MAX_REPOSITION_ATTEMPTS = 4;


//Servo Constant Positions
const int PET_DROP_INITIAL = 180;
const int PET_DROP_FINAL = 100;
const int SWEEPER_ARM_INTIAL = 0;
const int SWEEPER_ARM_FINAL = 180;

//IR Constants
const int MAX_IR_CORRECTION = 30;
const int IR_THRESHOLD = 50;
const int TAPE_SEARCH_THRESHOLD = 400;

//Arm Constants
const int VERTICAL = 90;
const int HORIZONTAL = 0;
const int PET5 = 45;
//EEPROM Values - Arm
int zeroPot;
int ninetyPot;
int armUpSpeed;
int armDownSpeed;


//Tape Following
const int STATE_FAR_RIGHT = 5;
const int STATE_FAR_LEFT = -5;
const int STATE_RIGHT = 1;
const int STATE_LEFT = -1;
const int STATE_ON_TAPE = 0;

const int PET_6_BEGIN = 20;
const int PET_6_GATE = 100;
const int PET_6_MOVE = 100;
const int PET_6_PET_4 = 100;
const int PET_6_SWEEP_MIN = 40;
const int PET_6_SWEEP_MAX = 180;
const int PET_6_RETRACT = 20;

//InternalVariables
//Allows tape following to continue across multiple functions
int state;
int lastState;
int stateCount = 0;
int p;
int d;
int correction;
int petCount = 0;

//EEPROM Stored - Tape
int tapeSpeed;
int tapeP;
int tapeD;
int tapeThreshold;
int markerThreshold;
int petDelay;
bool markerTapeDetected = false;
int markerDelay = 0;


//IR Following
int leftIRSensor;
int rightIRSensor;
int IRError;
int correctionIR;
//EEPROM Stored - IR            
int IRSpeed;
int IRP;
int IRD;




class MenuItem
{
public:
	String    Name;
	uint16_t  Value;
	uint16_t* EEPROMAddress;
	static uint16_t MenuItemCount;
	MenuItem(String name)
	{
		MenuItemCount++;
		EEPROMAddress = (uint16_t*)(2 * MenuItemCount);
		Name = name;
		Value = eeprom_read_word(EEPROMAddress);
	}
	void Save()
	{
		eeprom_write_word(EEPROMAddress, Value);
	}
};

uint16_t MenuItem::MenuItemCount = 0;
/* Add the menu items here */
MenuItem romTapeSpeed = MenuItem("Tape Speed");
MenuItem romTapeP = MenuItem("Tape P");
MenuItem romTapeD = MenuItem("Tape D");
MenuItem romTapeThreshold = MenuItem("Tape Thr");
MenuItem romMarkerThreshold = MenuItem("Marker Thr");
MenuItem romPetDelay = MenuItem("Pet Delay");
MenuItem romZeroPot = MenuItem("Min Arm");
MenuItem romNinetyPot = MenuItem("Max Arm");
MenuItem romArmUpSpeed = MenuItem("Arm Up");
MenuItem romArmDownSpeed = MenuItem("Arm Down");
MenuItem romIRSpeed = MenuItem("IR Speed");
MenuItem romIRP = MenuItem("IR P");
MenuItem romIRD = MenuItem("IR D");


MenuItem menuItems[] = { romTapeSpeed, romTapeP, romTapeD, romTapeThreshold, romMarkerThreshold, romPetDelay, romZeroPot, romNinetyPot, romArmUpSpeed, romArmDownSpeed, romIRSpeed, romIRP, romIRD };

//Initialize the TINAH
void setup()
{
#include <phys253setup.txt>
	LCD.clear();
	LCD.home();
	Serial.begin(9600);
	SetAllSettings();
	ChangeServo(PET_DROP_SERVO_PIN, PET_DROP_INITIAL);
	ChangeServo(SWEEPER_ARM_PIN, SWEEPER_ARM_INTIAL);
	ChangeServo(PET_6_SERVO_PIN, PET_6_BEGIN);
	ChangeArmPosition(VERTICAL);

}

//Main entrance into the program
void loop()
{
	MainMenu();
}

//Main menue to select program
void MainMenu(){


	while (true){
		LCD.clear(); LCD.home();
		LCD.print("Press Start to");
		LCD.setCursor(0, 1);
		int menuIndex = knob(7) * 10 / 1024;

		if (menuIndex == 0){
			LCD.print("Run Florence Code");
			if (IsSelected() == true){
				BeginFlorence();
			}
		}
		else if (menuIndex == 1){
			LCD.print("Run Machine Code");
			if (IsSelected() == true){
				BeginMachine();
			}
		}
		else if (menuIndex == 2){
			LCD.print("Change Settings");
			if (IsSelected() == true){
				SettingsMenu();
			}
		}
		else if (menuIndex == 3){
			LCD.print("Test TINAH");
			if (IsSelected() == true){
				TestComponents();
			}
		}
		else if (menuIndex == 4){
			LCD.print("Test Tape");
			if (IsSelected() == true){
				PureTapeFollowing();
			}
		}
		else if (menuIndex == 5){
			LCD.print("Test Arm");
			if (IsSelected() == true){
				ArmTest();
			}
		}
		else if (menuIndex == 6){
			LCD.print("Test IR");
			if (IsSelected() == true){
				FollowIR();
			}
		}
		else if (menuIndex == 7){
			LCD.print("Sweeper Arm");
			if (IsSelected() == true){
				SweeperArm();
			}
		}
		else if (menuIndex == 8){
			LCD.print("Pet4");
			if (IsSelected() == true){
				MachineTapeFollow();;
			}
		}
		else if (menuIndex == 9){
			LCD.print("Distance Readjust");
			if (IsSelected() == true){
				TestReposition();
			}
		}
		delay(100);
	}
}

//Checks to see if an option is selected
bool IsSelected(){
	if (startbutton() == true){

		delay(100);

		if (startbutton() == true){
			return true;
		}
	}
	return false;
}

//Preppare robot for Florence and Run the Florence Code
void BeginFlorence(){
	while (digitalRead(FLORENCE_START_PIN) == 0){}

	LCD.clear(); LCD.home();
	LCD.print("Running Florence");
	delay(3000);


	state = STATE_FAR_RIGHT;
	FlorenceTapeFollow();


}

//Prepare robot for Machince and Run the Machine Code
void BeginMachine(){
	LCD.clear(); LCD.home();
	LCD.print("Runing Machine");
	delay(1000);
	MoveLeft(300);
	MoveForward(200);
	MoveRight(100);
	ChangeServo(PET_6_SERVO_PIN, PET_6_GATE);

	lastState = STATE_LEFT;
	state = STATE_FAR_LEFT;

	Machine();

}

//Change the EEPROM Settings
void SettingsMenu()
{
	LCD.clear(); LCD.home();
	LCD.print("Entering menu");
	delay(1000);

	while (true)
	{
		/* Show MenuItem value and knob value */
		int menuIndex = knob(6) * (MenuItem::MenuItemCount) / 1024;
		LCD.clear(); LCD.home();
		LCD.print(menuItems[menuIndex].Name); LCD.print(" "); LCD.print(menuItems[menuIndex].Value);
		LCD.setCursor(0, 1);
		LCD.print("Set to "); LCD.print(knob(7)); LCD.print("?");
		delay(100);


		/* Press start button to save the new value */
		if (startbutton())
		{
			delay(500);
			if (startbutton() == false)
			{
				menuItems[menuIndex].Value = knob(7);
				menuItems[menuIndex].Save();
				delay(250);
			}
			else{
				menuItems[menuIndex].Value = FineTune(knob(7), menuItems[menuIndex].Name, menuItems[menuIndex].Value);
				menuItems[menuIndex].Save();
				delay(250);
			}
		}

		/* Press stop button to exit menu */
		if (stopbutton())
		{
			delay(100);
			if (stopbutton())
			{
				LCD.clear(); LCD.home();
				LCD.print("Leaving menu");
				delay(500);
				SetAllSettings();
				return;
			}
		}
	}
}

//Store all values of EEPROM into the program
void SetAllSettings(){
	tapeSpeed = menuItems[0].Value;
	tapeP = menuItems[1].Value;
	tapeD = menuItems[2].Value;
	tapeThreshold = menuItems[3].Value;
	markerThreshold = menuItems[4].Value;
	petDelay = menuItems[5].Value;
	zeroPot = menuItems[6].Value;
	ninetyPot = menuItems[7].Value;
	armUpSpeed = menuItems[8].Value;
	armDownSpeed = menuItems[9].Value;
	IRSpeed = menuItems[10].Value;
	IRP = menuItems[11].Value;
	IRD = menuItems[12].Value;
}

//Select a more percise EEPROM value
int FineTune(int center, String name, int currentValue){

	LCD.clear(); LCD.home();
	LCD.print("Fine Tune Value");
	delay(500);

	while (true){
		LCD.clear(); LCD.home();
		LCD.print(name); LCD.print(" "); LCD.print(currentValue);
		LCD.setCursor(0, 1);

		if ((center - 50) < 0){
			center = 50;
		}
		else if ((center + 50) > 1023){
			center = 1023 - 50;
		}

		int fineValue = map(knob(7), 0, 1023, center - 50, center + 50);
		LCD.print("Set fine to "); LCD.print(fineValue); LCD.print("?");
		delay(100);

		if (startbutton())
		{
			delay(100);
			if (startbutton())
			{
				return fineValue;
			}
		}
	}
}

//Menu to Test all components of the TINAH
void TestComponents(){

	LCD.clear();
	LCD.home();
	LCD.print("Entering Test");
	LCD.setCursor(0, 1);
	LCD.print("Menu");
	delay(500);

	while (true){
		int menuIndex = knob(6) * 3 / 1024;
		LCD.clear();
		LCD.home();
		LCD.print("Press Start to");
		LCD.setCursor(0, 1);
		LCD.print("Test: ");

		if (menuIndex == 0){
			LCD.print("Inputs");
		}
		else if (menuIndex == 1){
			LCD.print("Motors");
		}
		else if (menuIndex == 2){
			LCD.print("Servo");
		}
		delay(100);

		if (stopbutton() == true){

			delay(300);

			if (stopbutton() == true){
				LCD.clear();
				LCD.home();
				LCD.print("Entering Main");
				LCD.setCursor(0, 1);
				LCD.print("Menu");
				delay(500);
				return;
			}
		}

		if (startbutton() == true){

			delay(100);

			if (startbutton() == true){
				delay(100);
				if (menuIndex == 0){
					TestInputs();
				}
				else if (menuIndex == 1){
					TestMotor();
				}
				else if (menuIndex == 2){
					TestServo();
				}
			}
		}

	}

}

//Test all inputs
void TestInputs(){

	if (TestInstructions() == false){
		return;
	}

	int value;

	while (true){

		int pinIndex = (knob(6) * 24) / 1024;
		LCD.print("i");
		LCD.print(pinIndex);

		if (pinIndex <= 7){
			value = analogRead(pinIndex);
			LCD.clear();
			LCD.home();
			LCD.print("Pin:A");
			LCD.print(pinIndex);
			LCD.print(" Val :");
			LCD.print(value);
			delay(100);
			if (stopbutton() == true){
				delay(100);
				if (stopbutton() == true){
					return;
				}
			}
		}
		else{
			pinIndex = pinIndex - 8;
			value = digitalRead(pinIndex);
			LCD.clear();
			LCD.home();
			LCD.print("Pin:D");
			LCD.print(pinIndex);
			LCD.print(" Val:");
			LCD.print(value);
			delay(100);
		}

		if (stopbutton() == true){
			delay(100);
			if (stopbutton() == true){
				return;
			}
		}

	}
}

//Test the motors 0-3
void TestMotor(){

	if (TestInstructions() == false){
		return;
	}


	while (true){
		int motorIndex = knob(6) * 4 / 1024;
		if (motorIndex != lastMotorTest){
			motor.speed(lastMotorTest, 0);
			lastMotorTest = motorIndex;
		}

		delay(100);

		int motorSpeed = map(knob(7), 0, 1023, -255, 255);
		LCD.clear();
		LCD.home();
		LCD.print("Motor:");
		LCD.print(motorIndex);
		LCD.setCursor(0, 1);
		LCD.print("Speed:");
		LCD.print(motorSpeed);
		motor.speed(motorIndex, motorSpeed);

		if (stopbutton() == true){
			delay(100);
			if (stopbutton() == true){
				motor.stop_all();
				return;
			}
		}
	}
}

//Test the Servos 0-2
void TestServo(){

	if (TestInstructions() == false){
		return;
	}
	while (true){
		int servoIndex = knob(6) * 3 / 1024;
		delay(100);
		int angle = map(knob(7), 0, 1023, 0, 180);
		LCD.clear();
		LCD.home();
		LCD.print("Servo:");
		LCD.print(servoIndex);
		LCD.setCursor(0, 1);
		LCD.print("Angle:");
		LCD.print(angle);

		ChangeServo(servoIndex, angle);

		if (stopbutton() == true){

			delay(10);
			if (stopbutton() == true){
				return;
			}
		}
	}

}

//Checks to see if the user wants to leave the menu
bool TestInstructions(){
	LCD.clear();
	LCD.home();
	LCD.print("Press Start to Test");
	delay(100);
	while (startbutton() == false){
		if (stopbutton() == true){
			delay(100);
			if (stopbutton() == true){
				return false;
			}
		}
		return true;
	}
}

void Machine(){
	MachineTapeFollow();
	CollectPet4();
	MachineIRFollow5();
	returnArmPosition(VERTICAL);
	MachineIRFollow6();
	CollectPet6();
	ChangeServo(PET_6_SERVO_PIN, PET_6_RETRACT);
	MachineTurnAroundIR();
	IRReturn();
	FindTape();
	ChangeServo(PET_6_SERVO_PIN, PET_6_MOVE);
	PureTapeFollowing();

}

void IRReturn(){
	int lastError = 0;
	int positiveCount = 1;
	int negativeCount = 1;
	int count = 1;
	int cycles = 0;
	int attemptCount = 0;
	int petDistance;
	bool checkForPet = false;

	while (true){
		cycles++;
		leftIRSensor = analogRead(LEFT_IR_PIN);
		rightIRSensor = analogRead(RIGHT_IR_PIN);
		petDistance = analogRead(PET_DISTANCE_SENSOR_PIN);
		IRError = (leftIRSensor - rightIRSensor);

		//		OLD HARDCODING TIME TRIALS
		//		if ((cycles % 900 == 0) && (cycles >= 4500)){
		//			attemptCount++;
		//			motor.speed(LEFT_MOTOR_PIN, 0);
		//			motor.speed(RIGHT_MOTOR_PIN, 0);
		//			if (AttemptPet5() || (attemptCount > 4)){
		//				return;
		//			}
		//		}

		if ((analogRead(RIGHT_QRD_PIN) > TAPE_SEARCH_THRESHOLD) || (analogRead(LEFT_QRD_PIN) > TAPE_SEARCH_THRESHOLD) || (analogRead(MARKER_TAPE_QRD_PIN) > TAPE_SEARCH_THRESHOLD)){
			return;
		}


		if (lastError > 0){
			if (IRError > 0){
				positiveCount++;
				count = positiveCount;
			}
			else{
				negativeCount = 1;
				count = negativeCount;
			}
		}
		else if (lastError < 0){
			if (IRError < 0){
				negativeCount++;
				count = negativeCount;
			}
			else{
				positiveCount = 1;
				count = positiveCount;
			}
		}
		else{
			positiveCount = 1;
			negativeCount = 1;
			count = 1;
		}

		p = IRP*IRError / 5;
		d = (int)((float)IRD*(float)(IRError) / (float)count);
		correctionIR = p + d;

		if (correctionIR > MAX_IR_CORRECTION){
			correctionIR = MAX_IR_CORRECTION;
		}
		else if (correctionIR < -MAX_IR_CORRECTION){
			correctionIR = -MAX_IR_CORRECTION;
		}



		//Motor control 
		motor.speed(LEFT_MOTOR_PIN, (IRSpeed - correctionIR));
		motor.speed(RIGHT_MOTOR_PIN, (IRSpeed + correctionIR));

		if (stopbutton())
		{
			delay(100);
			if (stopbutton())
			{
				motor.speed(LEFT_MOTOR_PIN, 0);
				motor.speed(RIGHT_MOTOR_PIN, 0);
				return;
				LCD.clear();
				LCD.home();
			}
		}
	}


}

void FindTape(){

	while (true){

			motor.speed(RIGHT_MOTOR_PIN, 80);
			motor.speed(LEFT_MOTOR_PIN, -80);
			if ((analogRead(LEFT_QRD_PIN)> tapeThreshold) || (analogRead(RIGHT_QRD_PIN) > tapeThreshold)){
				if ((analogRead(LEFT_IR_PIN) > IR_THRESHOLD) && (analogRead(RIGHT_IR_PIN) > IR_THRESHOLD)){
					if (abs(analogRead(LEFT_IR_PIN) - (analogRead(RIGHT_IR_PIN))) < 70){
						return;
					}				
				}
			}
	}
}

void CollectPet6(){
	LCD.clear();
	LCD.home();
	LCD.write("Pet 6");

	for (int index = 0; index < 4; index++){
		for (int angle = PET_6_SWEEP_MAX; angle > PET_6_SWEEP_MIN; angle--){
			ChangeServo(PET_6_SERVO_PIN, angle);
			delay(5);
		}
		for (int angle = PET_6_SWEEP_MIN; angle < PET_6_SWEEP_MAX; angle++){
			ChangeServo(PET_6_SERVO_PIN, angle);
			delay(5);
		}
	}
}

void MachineTurnAroundIR(){
	ChangeServo(PET_6_SWEEP_MIN, PET_6_RETRACT);
	MoveBack(2000);
	//MoveForward(300);
	//MoveBack(500);



	motor.speed(LEFT_MOTOR_PIN, -130);
	motor.speed(RIGHT_MOTOR_PIN, 130);

	ChangeServo(PET_6_SWEEP_MIN, PET_6_RETRACT);

	delay(1000);


	while (true){
		motor.speed(LEFT_MOTOR_PIN, -130);
		motor.speed(RIGHT_MOTOR_PIN, 130);
		if ((analogRead(RIGHT_IR_PIN)> IR_THRESHOLD) || (analogRead(LEFT_IR_PIN)> IR_THRESHOLD)){
			return;
		}
	}
}

void MachineTapeFollow(){

	int leftWheelSpeed;
	int rightWheelSpeed;
	int MA_petCount = 0;
	bool checkForPet = false;
	int cycles = 0;
	int dLastState = STATE_ON_TAPE;
	int powerFactor = 1;
	int sensorValueRight;
	int sensorValueLeft;
	int sensorPetMarker;


	while (true){

		cycles++;

		//delay before checking for pets
		if (cycles == 5000){
			checkForPet = true;
			LCD.clear();
			LCD.home();
			LCD.print("Checking Pet Markers");

		}

		//Obtain values from QRD's
		sensorValueRight = analogRead(RIGHT_QRD_PIN);
		sensorValueLeft = analogRead(LEFT_QRD_PIN);
		sensorPetMarker = analogRead(MARKER_TAPE_QRD_PIN);

		//Check to see if pet tape is detected.
		if ((sensorPetMarker > markerThreshold) && checkForPet){
			markerTapeDetected = true;
			markerDelay = 0;

		}

		//If the pet was detected continue forward for a bit then collect
		if (markerTapeDetected == true){
			markerDelay++;
			if (markerDelay >= petDelay){
				markerTapeDetected = false;
				MA_petCount++;
				LCD.clear();
				LCD.home();
				LCD.print(MA_petCount);
			}
		}

		if (MA_petCount == 1){
			ChangeServo(PET_6_SERVO_PIN, PET_6_MOVE);
		}

		if (MA_petCount == 3){
			ChangeServo(PET_6_SERVO_PIN, PET_6_PET_4);
		}

		if (MA_petCount == 4){
			return;
		}

		//Determine current  state
		if ((sensorValueRight > tapeThreshold) && (sensorValueLeft > tapeThreshold)){
			state = STATE_ON_TAPE;

			stateCount = 0;
			lastState = STATE_ON_TAPE;
		}
		else if ((sensorValueLeft < tapeThreshold) && (sensorValueRight < tapeThreshold)){
			if (lastState == STATE_LEFT){
				state = STATE_FAR_LEFT;
			}
			else if (lastState == STATE_RIGHT){
				state = STATE_FAR_RIGHT;
			}
			stateCount++;
		}
		else if (sensorValueLeft < tapeThreshold){
			state = STATE_LEFT;
			stateCount++;
			lastState = STATE_LEFT;
		}
		else if (sensorValueRight < tapeThreshold){
			state = STATE_RIGHT;
			stateCount++;
			lastState = STATE_RIGHT;
		}

		//GAINS
		p = tapeP*state; //+ (int)state*extremeErrorCount/5;



		d = (int)((float)tapeD*(float)(state - dLastState) / (float)stateCount);
		correction = p + d;

		leftWheelSpeed = (int)(powerFactor)*(tapeSpeed - correction);
		rightWheelSpeed = (int)(powerFactor)*(tapeSpeed + correction);

		//Motor control 
		motor.speed(LEFT_MOTOR_PIN, leftWheelSpeed);
		motor.speed(RIGHT_MOTOR_PIN, rightWheelSpeed);

		//return to menu
		if (stopbutton())
		{
			delay(100);
			if (stopbutton())
			{
				motor.speed(LEFT_MOTOR_PIN, 0);
				motor.speed(RIGHT_MOTOR_PIN, 0);
				return;
				LCD.clear();
				LCD.home();
			}
		}
	}
}

void MachineIRFollow6(){
	int lastError = 0;
	int positiveCount = 1;
	int negativeCount = 1;
	int count = 1;
	int cycles = 0;
	int attemptCount = 0;
	int petDistance;
	bool checkForPet = false;

	while (true){
		cycles++;
		leftIRSensor = analogRead(LEFT_IR_PIN);
		rightIRSensor = analogRead(RIGHT_IR_PIN);
		petDistance = analogRead(PET_DISTANCE_SENSOR_PIN);
		IRError = (leftIRSensor - rightIRSensor);

		//		OLD HARDCODING TIME TRIALS
		//		if ((cycles % 900 == 0) && (cycles >= 4500)){
		//			attemptCount++;
		//			motor.speed(LEFT_MOTOR_PIN, 0);
		//			motor.speed(RIGHT_MOTOR_PIN, 0);
		//			if (AttemptPet5() || (attemptCount > 4)){
		//				return;
		//			}
		//		}

		if ((digitalRead(PET_6_LEFT_SWITCH) == 0) || (digitalRead(PET_6_RIGHT_SWITCH) == 0)){
			motor.speed(LEFT_MOTOR_PIN, 0);
			motor.speed(RIGHT_MOTOR_PIN, 0);
			return;
		}


		if (lastError > 0){
			if (IRError > 0){
				positiveCount++;
				count = positiveCount;
			}
			else{
				negativeCount = 1;
				count = negativeCount;
			}
		}
		else if (lastError < 0){
			if (IRError < 0){
				negativeCount++;
				count = negativeCount;
			}
			else{
				positiveCount = 1;
				count = positiveCount;
			}
		}
		else{
			positiveCount = 1;
			negativeCount = 1;
			count = 1;
		}

		p = IRP*IRError / 5;
		d = (int)((float)IRD*(float)(IRError) / (float)count);
		correctionIR = p + d;

		if (correctionIR > MAX_IR_CORRECTION){
			correctionIR = MAX_IR_CORRECTION;
		}
		else if (correctionIR < -MAX_IR_CORRECTION){
			correctionIR = -MAX_IR_CORRECTION;
		}



		//Motor control 
		motor.speed(LEFT_MOTOR_PIN, (IRSpeed - correctionIR));
		motor.speed(RIGHT_MOTOR_PIN, (IRSpeed + correctionIR));

		if (stopbutton())
		{
			delay(100);
			if (stopbutton())
			{
				motor.speed(LEFT_MOTOR_PIN, 0);
				motor.speed(RIGHT_MOTOR_PIN, 0);
				return;
				LCD.clear();
				LCD.home();
			}
		}
	}


}

void MachineIRFollow5(){
	int lastError = 0;
	int positiveCount = 1;
	int negativeCount = 1;
	int count = 1;
	int cycles = 0;
	int attemptCount = 0;
	int petDistance;
	bool checkForPet = false;

	while (true){
		cycles++;
		leftIRSensor = analogRead(LEFT_IR_PIN);
		rightIRSensor = analogRead(RIGHT_IR_PIN);
		petDistance = analogRead(PET_DISTANCE_SENSOR_PIN);
		IRError = (leftIRSensor - rightIRSensor);

		//		OLD HARDCODING TIME TRIALS
		//		if ((cycles % 900 == 0) && (cycles >= 4500)){
		//			attemptCount++;
		//			motor.speed(LEFT_MOTOR_PIN, 0);
		//			motor.speed(RIGHT_MOTOR_PIN, 0);
		//			if (AttemptPet5() || (attemptCount > 4)){
		//				return;
		//			}
		//		}

		//delay before checking for pets
		if (cycles == 4500){
			checkForPet = true;
			LCD.clear();
			LCD.home();
			LCD.print("555555!");

		}

		if ((petDistance > NOISE_THRESHOLD) && (checkForPet == true)){
			motor.speed(LEFT_MOTOR_PIN, 0);
			motor.speed(RIGHT_MOTOR_PIN, 0);
			if (AttemptPet5()){
				return;
			}
			MoveForward(3000);
		}

		if (lastError > 0){
			if (IRError > 0){
				positiveCount++;
				count = positiveCount;
			}
			else{
				negativeCount = 1;
				count = negativeCount;
			}
		}
		else if (lastError < 0){
			if (IRError < 0){
				negativeCount++;
				count = negativeCount;
			}
			else{
				positiveCount = 1;
				count = positiveCount;
			}
		}
		else{
			positiveCount = 1;
			negativeCount = 1;
			count = 1;
		}

		p = IRP*IRError / 5;
		d = (int)((float)IRD*(float)(IRError) / (float)count);
		correctionIR = p + d;

		if (correctionIR > MAX_IR_CORRECTION){
			correctionIR = MAX_IR_CORRECTION;
		}
		else if (correctionIR < -MAX_IR_CORRECTION){
			correctionIR = -MAX_IR_CORRECTION;
		}



		//Motor control 
		motor.speed(LEFT_MOTOR_PIN, (IRSpeed - correctionIR));
		motor.speed(RIGHT_MOTOR_PIN, (IRSpeed + correctionIR));

		if (stopbutton())
		{
			delay(100);
			if (stopbutton())
			{
				motor.speed(LEFT_MOTOR_PIN, 0);
				motor.speed(RIGHT_MOTOR_PIN, 0);
				return;
				LCD.clear();
				LCD.home();
			}
		}
	}


}

void MachineIRFollow4(){
	int lastError = 0;
	int positiveCount = 1;
	int negativeCount = 1;
	int count = 1;
	int cycles = 0;
	int attemptCount = 0;
	int petDistance;
	bool checkForPet = false;

	motor.speed(LEFT_MOTOR_PIN, 0);
	motor.speed(RIGHT_MOTOR_PIN, 0);

	SweeperArm();
	delay(250);

	while (true){
		cycles++;
		leftIRSensor = analogRead(LEFT_IR_PIN);
		rightIRSensor = analogRead(RIGHT_IR_PIN);
		petDistance = analogRead(PET_DISTANCE_SENSOR_PIN);
		IRError = (leftIRSensor - rightIRSensor);

		//		OLD HARDCODING TIME TRIALS
		//		if ((cycles % 900 == 0) && (cycles >= 4500)){
		//			attemptCount++;
		//			motor.speed(LEFT_MOTOR_PIN, 0);
		//			motor.speed(RIGHT_MOTOR_PIN, 0);
		//			if (AttemptPet5() || (attemptCount > 4)){
		//				return;
		//			}
		//		}

		//delay before checking for pets
		if (cycles == 2000){
			checkForPet = true;
			LCD.clear();
			LCD.home();
			LCD.print("Checking for Pet");

		}

		if ((petDistance > NOISE_THRESHOLD) && (checkForPet == true)){
			attemptCount++;
			motor.speed(LEFT_MOTOR_PIN, 0);
			motor.speed(RIGHT_MOTOR_PIN, 0);
			if (AttemptPet4() || (attemptCount > 1)){
				return;
			}
			MoveForward(3000);
		}

		if (lastError > 0){
			if (IRError > 0){
				positiveCount++;
				count = positiveCount;
			}
			else{
				negativeCount = 1;
				count = negativeCount;
			}
		}
		else if (lastError < 0){
			if (IRError < 0){
				negativeCount++;
				count = negativeCount;
			}
			else{
				positiveCount = 1;
				count = positiveCount;
			}
		}
		else{
			positiveCount = 1;
			negativeCount = 1;
			count = 1;
		}



		p = IRP*IRError / 5;
		d = (int)((float)IRD*(float)(IRError) / (float)count);
		correctionIR = p + d;


		if (correctionIR > MAX_IR_CORRECTION){
			correctionIR = MAX_IR_CORRECTION;
		}
		else if (correctionIR < -MAX_IR_CORRECTION){
			correctionIR = -MAX_IR_CORRECTION;
		}

		//Motor control 
		motor.speed(LEFT_MOTOR_PIN, (IRSpeed - correctionIR));
		motor.speed(RIGHT_MOTOR_PIN, (IRSpeed + correctionIR));

		if (stopbutton())
		{
			delay(100);
			if (stopbutton())
			{
				motor.speed(LEFT_MOTOR_PIN, 0);
				motor.speed(RIGHT_MOTOR_PIN, 0);
				return;
				LCD.clear();
				LCD.home();
			}
		}
	}


}

bool AttemptPet5(){
	//ChangeArmPosition(77);
	RCServo0.write(180);
	//delay(250);
	ChangeArmPosition(30);
	delay(500);
	if (returnArmPosition(78) == false){
		return true;
	}
	else{
		return true;
	}
}

bool AttemptPet4(){
	ChangeArmPosition(77);
	RCServo0.write(180);
	//delay(250);
	ChangeArmPosition(0);
	delay(500);
	if (returnArmPosition(78) == false){
		return false;
	}
	else{
		DropPet();
		return true;
	}
}

void CollectPet4(){
	motor.speed(LEFT_MOTOR_PIN, 0);
	motor.speed(RIGHT_MOTOR_PIN, 0);

	SweeperArm();
	//MoveOnTape(1, 500);
	//MoveOnTapeUntilSensor(1, 3000);
	//RepositionByDistance();
	motor.speed(LEFT_MOTOR_PIN, 0);
	motor.speed(RIGHT_MOTOR_PIN, 0);


	RCServo0.write(180);
	ChangeArmPosition(0);
	delay(250);
	if (returnArmPosition4(50) == false){
		MoveBack(500);
		ChangeArmPosition(0);
		delay(250);
		if (returnArmPosition4(50) == false){
			ChangeArmPosition(80);
			DropPet();
		}
		else{
			DropPet();
		}
	}
	else{
		DropPet();
	}

}

void MoveLeft(int delayTime){
	motor.speed(LEFT_MOTOR_PIN, 10);
	motor.speed(RIGHT_MOTOR_PIN, 80);
	delay(delayTime);
	motor.speed(LEFT_MOTOR_PIN, 0);
	motor.speed(RIGHT_MOTOR_PIN, 0);
}

void MoveRight(int delayTime){
	motor.speed(LEFT_MOTOR_PIN, 80);
	motor.speed(RIGHT_MOTOR_PIN, 10);
	delay(delayTime);
	motor.speed(LEFT_MOTOR_PIN, 0);
	motor.speed(RIGHT_MOTOR_PIN, 0);
}

void MoveForward(int delayTime){

	motor.speed(LEFT_MOTOR_PIN, 80);
	motor.speed(RIGHT_MOTOR_PIN, 80);
	delay(delayTime);
	motor.speed(LEFT_MOTOR_PIN, 0);
	motor.speed(RIGHT_MOTOR_PIN, 0);
}

void MoveBack(int delayTime){

	motor.speed(LEFT_MOTOR_PIN, -100);
	motor.speed(RIGHT_MOTOR_PIN, -100);
	delay(delayTime);
	motor.speed(LEFT_MOTOR_PIN, 0);
	motor.speed(RIGHT_MOTOR_PIN, 0);
}

void SweeperArm(){
	//	for (int angle = 0; angle <= 180; angle = angle + 20){
	//		RCServo1.write(angle);
	//		delay(10);
	//	}
	RCServo1.write(SWEEPER_ARM_FINAL);
	for (int index = 0; index < SWEEPER_ARM_FINAL; index++){
		ChangeServo(SWEEPER_ARM_PIN, index);
		delay(3);
	}
	delay(1000);
	RCServo1.write(SWEEPER_ARM_INTIAL);
}

void MachineTurnAround(){


	motor.speed(LEFT_MOTOR_PIN, -70);
	motor.speed(RIGHT_MOTOR_PIN, -70);

	delay(3000);

	motor.speed(LEFT_MOTOR_PIN, -130);
	motor.speed(RIGHT_MOTOR_PIN, 130);

	delay(1000);


	while (true){


		motor.speed(LEFT_MOTOR_PIN, -130);
		motor.speed(RIGHT_MOTOR_PIN, 130);
		if (analogRead(RIGHT_QRD_PIN)> tapeThreshold){
			petCount++;
			return;
		}
	}

}

void FlorenceTapeFollow(){
	int extremeErrorCount = 0;
	int leftWheelSpeed;
	int rightWheelSpeed;
	int sensorValueRight;
	int sensorValueLeft;
	int sensorPetMarker;
	petCount = 0;
	bool checkForPetFlorence = false;
	int cycles = 0;
	int dLastState = STATE_ON_TAPE;
	int powerFactor = 1;

	//int checkStateDelayCount = 0;
	//int delayState = STATE_FAR_RIGHT;
	lastState = STATE_RIGHT;


	while (true){


		//checkStateDelayCount++;

		cycles++;

		//delay before checking for pets
		if (cycles == 10000){
			checkForPetFlorence = true;
			LCD.clear();
			LCD.home();
			LCD.print("pet");

		}

		//Obtain values from QRD's
		sensorValueRight = analogRead(RIGHT_QRD_PIN);
		sensorValueLeft = analogRead(LEFT_QRD_PIN);
		sensorPetMarker = analogRead(MARKER_TAPE_QRD_PIN);

		//Determine current  state
		if ((sensorValueRight > tapeThreshold) && (sensorValueLeft > tapeThreshold)){
			state = STATE_ON_TAPE;

			stateCount = 0;
			lastState = STATE_ON_TAPE;
		}
		else if ((sensorValueLeft < tapeThreshold) && (sensorValueRight < tapeThreshold)){
			if (lastState == STATE_LEFT){
				state = STATE_FAR_LEFT;
			}
			else if (lastState == STATE_RIGHT){
				state = STATE_FAR_RIGHT;
			}
			stateCount++;
		}
		else if (sensorValueLeft < tapeThreshold){
			state = STATE_LEFT;
			stateCount++;
			lastState = STATE_LEFT;
		}
		else if (sensorValueRight < tapeThreshold){
			state = STATE_RIGHT;
			stateCount++;
			lastState = STATE_RIGHT;
		}

		//Check to see if pet tape is detected.
		if ((sensorPetMarker > markerThreshold) && checkForPetFlorence){
			markerTapeDetected = true;
			markerDelay = 0;

		}

		//If the pet was detected continue forward for a bit then collect
		if (markerTapeDetected == true){
			markerDelay++;
			if (markerDelay >= petDelay){
				motor.speed(LEFT_MOTOR_PIN, (0));
				motor.speed(RIGHT_MOTOR_PIN, (0));
				CollectPet();
				markerTapeDetected = false;
				LCD.clear();
				LCD.home();
				LCD.print(petCount);
			}
		}

		if (petCount == 3){
			TurnAround();
			checkForPetFlorence = false;
		}

		//if (lastWheelEncoderLeft == leftWheelEncoder){
		//	leftWheelCount++;
		//	if (leftWheelCount >= 300){
		//		powerFactor = 1.8;
		//		//				LCD.clear();
		//		//				LCD.home();
		//		//				LCD.print("Higher Power Factor");
		//	}
		//	else if (leftWheelCount <= 200){
		//		powerFactor = 1;
		//		//				LCD.clear();
		//		//				LCD.home();
		//		//				LCD.print("Lower Power Factor");
		//	}
		//}
		//else{
		//	leftWheelCount = 0;
		//}

		//if (lastWheelEncoderRight == rightWheelEncoder){
		//	rightWheelCount++;
		//	if (rightWheelCount >= 300){
		//		powerFactor = 1.8;
		//		//				LCD.clear();
		//		//				LCD.home();
		//		//				LCD.print("Higher Power Factor");
		//	}
		//	else if (rightWheelCount <= 200){
		//		powerFactor = 1;
		//		//				LCD.clear();
		//		//				LCD.home();
		//		//				LCD.print("Lower Power Factor");
		//	}
		//}
		//else{
		//	rightWheelCount = 0;
		//}


		////GAINS
		//if (checkStateDelayCount < 4){
		//	state = delayState;
		//}

		p = tapeP*state;


		if (stateCount != 0){
			d = (int)((float)tapeD*(float)(state - dLastState) / (float)stateCount);
		}
		correction = p + d;

		leftWheelSpeed = (int)(powerFactor)*(tapeSpeed - correction);
		rightWheelSpeed = (int)(powerFactor)*(tapeSpeed + correction);

		//Motor control 
		motor.speed(LEFT_MOTOR_PIN, leftWheelSpeed);
		motor.speed(RIGHT_MOTOR_PIN, rightWheelSpeed);

		//return to menu
		if (stopbutton())
		{
			delay(100);
			if (stopbutton())
			{
				motor.speed(LEFT_MOTOR_PIN, 0);
				motor.speed(RIGHT_MOTOR_PIN, 0);
				return;
				LCD.clear();
				LCD.home();
			}
		}
	}
}

void TurnAround(){

	motor.speed(LEFT_MOTOR_PIN, -70);
	motor.speed(RIGHT_MOTOR_PIN, 70);

	delay(750);


	while (true){


		motor.speed(LEFT_MOTOR_PIN, -70);
		motor.speed(RIGHT_MOTOR_PIN, 70);
		if (analogRead(RIGHT_QRD_PIN)> tapeThreshold){
			petCount++;
			return;
		}
	}
}

void LostTape(){
	LCD.clear();
	LCD.home();
	LCD.print("lost Tape mode");
}

void PureTapeFollowing(){
	int sensorValueRight;
	int sensorValueLeft;
	int sensorPetMarker;

	while (true){

		//Obtain values from QRD's
		sensorValueRight = analogRead(RIGHT_QRD_PIN);
		sensorValueLeft = analogRead(LEFT_QRD_PIN);
		sensorPetMarker = analogRead(MARKER_TAPE_QRD_PIN);

		//Determine current  state
		if ((sensorValueRight > tapeThreshold) && (sensorValueLeft > tapeThreshold)){
			state = STATE_ON_TAPE;

			stateCount = 0;
			lastState = STATE_ON_TAPE;
		}
		else if ((sensorValueLeft < tapeThreshold) && (sensorValueRight < tapeThreshold)){
			if (lastState == STATE_LEFT){
				state = STATE_FAR_LEFT;
			}
			else if (lastState == STATE_RIGHT){
				state = STATE_FAR_RIGHT;
			}
			stateCount++;
		}
		else if (sensorValueLeft < tapeThreshold){
			state = STATE_LEFT;
			stateCount++;
			lastState = STATE_LEFT;
		}
		else if (sensorValueRight < tapeThreshold){
			state = STATE_RIGHT;
			stateCount++;
			lastState = STATE_RIGHT;
		}

		//GAINS
		p = tapeP*state;
		d = (int)((float)tapeD*(float)(state - lastState) / (float)stateCount);
		correction = p + d;

		//Motor control 
		motor.speed(LEFT_MOTOR_PIN, (tapeSpeed - correction));
		motor.speed(RIGHT_MOTOR_PIN, (tapeSpeed + correction));

		if (stopbutton())
		{
			delay(100);
			if (stopbutton())
			{
				motor.speed(LEFT_MOTOR_PIN, 0);
				motor.speed(RIGHT_MOTOR_PIN, 0);
				return;
				LCD.clear();
				LCD.home();
			}
		}
	}
}

void CollectPet(){

	petCount++;

	RCServo0.write(180);
	ChangeArmPosition(0);
	delay(250);
	if (returnArmPosition(30) == false){
		MoveOnTape(1, 1000);
		ChangeArmPosition(0);
		delay(250);
		if (returnArmPosition(30) == false){
			ChangeArmPosition(90);
			DropPet();
		}
		else{
			DropPet();
		}
	}
	else{
		DropPet();
	}
}


void MoveOnTape(int roboDirection, int stopCount){
	int sensorValueRight;
	int sensorValueLeft;
	int sensorPetMarker;
	int leftWheelSpeed;
	int rightWheelSpeed;
	int cycles = 0;
	int dLastState = STATE_ON_TAPE;

	for (int index = 0; index <= stopCount; index++){

		//Obtain values from QRD's
		sensorValueRight = analogRead(RIGHT_QRD_PIN);
		sensorValueLeft = analogRead(LEFT_QRD_PIN);

		//Determine current  state
		if ((sensorValueRight > tapeThreshold) && (sensorValueLeft > tapeThreshold)){
			state = STATE_ON_TAPE;

			stateCount = 0;
			lastState = STATE_ON_TAPE;
		}
		else if ((sensorValueLeft < tapeThreshold) && (sensorValueRight < tapeThreshold)){
			if (lastState == STATE_LEFT){
				state = STATE_FAR_LEFT;
			}
			else if (lastState == STATE_RIGHT){
				state = STATE_FAR_RIGHT;
			}
			stateCount++;
		}
		else if (sensorValueLeft < tapeThreshold){
			state = STATE_LEFT;
			stateCount++;
			lastState = STATE_LEFT;
		}
		else if (sensorValueRight < tapeThreshold){
			state = STATE_RIGHT;
			stateCount++;
			lastState = STATE_RIGHT;
		}


		//GAINS
		p = tapeP*state; //+ (int)state*extremeErrorCount/5;



		d = (int)((float)tapeD*(float)(state - dLastState) / (float)stateCount);
		correction = p + d;

		leftWheelSpeed = (int)(roboDirection)*(tapeSpeed - correction) / 2;
		rightWheelSpeed = (int)(roboDirection)*(tapeSpeed + correction) / 2;

		//Motor control 
		motor.speed(LEFT_MOTOR_PIN, leftWheelSpeed);
		motor.speed(RIGHT_MOTOR_PIN, rightWheelSpeed);

		//return to menu
		if (stopbutton())
		{
			delay(100);
			if (stopbutton())
			{
				motor.speed(LEFT_MOTOR_PIN, 0);
				motor.speed(RIGHT_MOTOR_PIN, 0);
				return;
				LCD.clear();
				LCD.home();
			}
		}
	}
	motor.speed(LEFT_MOTOR_PIN, 0);
	motor.speed(RIGHT_MOTOR_PIN, 0);

}

void MoveOnTapeUntilSensor(int roboDirection, int stopCount){

	int sensorValueRight;
	int sensorValueLeft;
	int sensorPetMarker;
	int leftWheelSpeed;
	int rightWheelSpeed;
	int cycles = 0;
	int dLastState = STATE_ON_TAPE;
	int petDistance;

	for (int index = 0; index <= stopCount; index++){

		//Obtain values from QRD's
		sensorValueRight = analogRead(RIGHT_QRD_PIN);
		sensorValueLeft = analogRead(LEFT_QRD_PIN);
		petDistance = analogRead(PET_DISTANCE_SENSOR_PIN);

		if (petDistance > NOISE_THRESHOLD){

			motor.speed(LEFT_MOTOR_PIN, 0);
			motor.speed(RIGHT_MOTOR_PIN, 0);
			return;
		}




		//Determine current  state
		if ((sensorValueRight > tapeThreshold) && (sensorValueLeft > tapeThreshold)){
			state = STATE_ON_TAPE;

			stateCount = 0;
			lastState = STATE_ON_TAPE;
		}
		else if ((sensorValueLeft < tapeThreshold) && (sensorValueRight < tapeThreshold)){
			if (lastState == STATE_LEFT){
				state = STATE_FAR_LEFT;
			}
			else if (lastState == STATE_RIGHT){
				state = STATE_FAR_RIGHT;
			}
			stateCount++;
		}
		else if (sensorValueLeft < tapeThreshold){
			state = STATE_LEFT;
			stateCount++;
			lastState = STATE_LEFT;
		}
		else if (sensorValueRight < tapeThreshold){
			state = STATE_RIGHT;
			stateCount++;
			lastState = STATE_RIGHT;
		}


		//GAINS
		p = tapeP*state; //+ (int)state*extremeErrorCount/5;



		d = (int)((float)tapeD*(float)(state - dLastState) / (float)stateCount);
		correction = p + d;

		leftWheelSpeed = (int)(roboDirection)*(tapeSpeed - correction) / 2;
		rightWheelSpeed = (int)(roboDirection)*(tapeSpeed + correction) / 2;

		//Motor control 
		motor.speed(LEFT_MOTOR_PIN, leftWheelSpeed);
		motor.speed(RIGHT_MOTOR_PIN, rightWheelSpeed);

		//return to menu
		if (stopbutton())
		{
			delay(100);
			if (stopbutton())
			{
				motor.speed(LEFT_MOTOR_PIN, 0);
				motor.speed(RIGHT_MOTOR_PIN, 0);
				return;
				LCD.clear();
				LCD.home();
			}
		}
	}
	motor.speed(LEFT_MOTOR_PIN, 0);
	motor.speed(RIGHT_MOTOR_PIN, 0);

}

void DropPet(){

	if (petCount != 3){
		for (int index = 0; index <3; index++){

			RCServo0.write(100);
			delay(400);
			RCServo0.write(180);
			delay(400);

			if (digitalRead(PET_SWITCH_PIN) == 1){
				return;
			}
		}
	}

}

void ArmTest(){

	while (true){

		delay(100);
		int angle = map(knob(7), 0, 1023, 0, 90);
		LCD.clear();
		LCD.home();
		LCD.print("Testing Arm:");
		LCD.setCursor(0, 1);
		LCD.print("Angle:");
		LCD.print(angle);
		ChangeArmPosition(angle);
		delay(500);

		if (stopbutton() == true){

			delay(300);

			if (stopbutton() == true){
				LCD.clear();
				LCD.home();
				LCD.print("Entering Main");
				LCD.setCursor(0, 1);
				LCD.print("Menu");
				delay(500);
				return;
			}
		}
	}


}

void ChangeArmPosition(int angle){
	int armCorrection;
	int anglePotValue;

	anglePotValue = map(angle, 0, 90, zeroPot, ninetyPot);
	int armPositionSensor;
	int count;
	int armAngleError;
	while (true){
		armPositionSensor = analogRead(ARM_POSITION_PIN);
		armAngleError = armPositionSensor - anglePotValue;

		if (armAngleError < 5 && armAngleError > -5){
			motor.speed(ARM_MOTOR_PIN, 0);
			return;
		}
		else{

			if (armAngleError > 0){
				armCorrection = armDownSpeed;
			}
			else if (armAngleError < 0){
				armCorrection = -armUpSpeed;
			}

			motor.speed(ARM_MOTOR_PIN, armCorrection);
		}

		if (stopbutton())
		{
			delay(100);
			if (stopbutton())
			{
				motor.stop_all();
				return;
				LCD.clear();
				LCD.home();
			}
		}


	}
}

bool returnArmPosition(int checkAngle){
	int armCorrection;
	int anglePotValue;

	anglePotValue = map(90, 0, 90, zeroPot, ninetyPot);
	int checkForPet = map(checkAngle, 0, 90, zeroPot, ninetyPot);
	int count;
	int armPositionSensor;
	int armAngleError;
	while (true){
		armPositionSensor = analogRead(ARM_POSITION_PIN);
		armAngleError = armPositionSensor - anglePotValue;

		if (armPositionSensor >= checkForPet){
			if (digitalRead(PET_SWITCH_PIN) == 1){
				motor.speed(ARM_MOTOR_PIN, 0);
				return false;
			}
		}

		if (armAngleError < 5 && armAngleError > -5){
			motor.speed(ARM_MOTOR_PIN, 0);
			return true;
		}
		else{

			if (armAngleError > 0){
				armCorrection = armDownSpeed;
			}
			else if (armAngleError < 0){
				armCorrection = -armUpSpeed;
			}

			motor.speed(ARM_MOTOR_PIN, armCorrection);
		}

		if (stopbutton())
		{
			delay(100);
			if (stopbutton())
			{
				motor.stop_all();
				return true;
				LCD.clear();
				LCD.home();
			}
		}


	}
}

bool returnArmPosition4(int checkAngle){
	int armCorrection;
	int anglePotValue;
	anglePotValue = map(85, 0, 90, zeroPot, ninetyPot);
	int checkForPet = map(checkAngle, 0, 90, zeroPot, ninetyPot);
	int count;
	int armPositionSensor;
	int armAngleError;
	while (true){
		armPositionSensor = analogRead(ARM_POSITION_PIN);
		armAngleError = armPositionSensor - anglePotValue;

		if (armPositionSensor >= checkForPet){
			if (digitalRead(PET_SWITCH_PIN) == 1){
				motor.speed(ARM_MOTOR_PIN, 0);
				return false;
			}
		}

		if (armAngleError < 5 && armAngleError > -5){
			motor.speed(ARM_MOTOR_PIN, 0);
			return true;
		}
		else{

			if (armAngleError > 0){
				armCorrection = armDownSpeed;
			}
			else if (armAngleError < 0){
				armCorrection = -armUpSpeed;
			}

			motor.speed(ARM_MOTOR_PIN, armCorrection);
		}

		if (stopbutton())
		{
			delay(100);
			if (stopbutton())
			{
				motor.stop_all();
				return true;
				LCD.clear();
				LCD.home();
			}
		}


	}
}

void FollowIR(){
	int lastError = 0;
	int positiveCount = 1;
	int negativeCount = 1;
	int count = 1;

	while (true){
		leftIRSensor = analogRead(LEFT_IR_PIN);
		rightIRSensor = analogRead(RIGHT_IR_PIN);
		IRError = (leftIRSensor - rightIRSensor);

		if (lastError > 0){
			if (IRError > 0){
				positiveCount++;
				count = positiveCount;
			}
			else{
				negativeCount = 1;
				count = negativeCount;
			}
		}
		else if (lastError < 0){
			if (IRError < 0){
				negativeCount++;
				count = negativeCount;
			}
			else{
				positiveCount = 1;
				count = positiveCount;
			}
		}
		else{
			positiveCount = 1;
			negativeCount = 1;
			count = 1;
		}

		p = IRP*IRError / 5;
		d = (int)((float)IRD*(float)(IRError) / (float)count);
		correctionIR = p + d;

		if (correctionIR > MAX_IR_CORRECTION){
			correctionIR = MAX_IR_CORRECTION;
		}
		else if (correctionIR < -MAX_IR_CORRECTION){
			correctionIR = -MAX_IR_CORRECTION;
		}


		//Motor control 
		motor.speed(LEFT_MOTOR_PIN, (IRSpeed - correctionIR));
		motor.speed(RIGHT_MOTOR_PIN, (IRSpeed + correctionIR));

		if (stopbutton())
		{
			delay(100);
			if (stopbutton())
			{
				motor.stop_all();
				return;
				LCD.clear();
				LCD.home();
			}
		}
	}

}

//ChangeServo - allows for the easy change in Servo Pin Values
void ChangeServo(int servoPin, int angle){

	if (servoPin == 0){
		RCServo0.write(angle);
	}
	else if (servoPin == 1){
		RCServo1.write(angle);
	}
	else if (servoPin == 2){
		RCServo2.write(angle);
	}
}


void TestReposition(){
	while (true){
		motor.speed(RIGHT_MOTOR_PIN, 100);
		motor.speed(LEFT_MOTOR_PIN, 100);

		if (analogRead(PET_DISTANCE_SENSOR_PIN) > NOISE_THRESHOLD){
			RepositionByDistance();
		}
	}
}

void RepositionByDistance(){

	for (int index = 0; index < MAX_REPOSITION_ATTEMPTS; index++){

		int petDistance = analogRead(PET_DISTANCE_SENSOR_PIN);

		//pet is in correct Range
		if ((petDistance > MIN_PET_DISTANCE) && (petDistance < MAX_PET_DISTANCE)){
			return;
		}

		if (petDistance < MIN_PET_DISTANCE){
			MoveCloser();
		}
		else if (petDistance > MAX_PET_DISTANCE){
			MoveFurther();
		}


	}
}

void MoveCloser(){
	motor.speed(LEFT_MOTOR_PIN, 100);
	motor.speed(RIGHT_MOTOR_PIN, 0);
	delay(400);
	motor.speed(LEFT_MOTOR_PIN, 100);
	motor.speed(RIGHT_MOTOR_PIN, 100);
	delay(400);
	motor.speed(LEFT_MOTOR_PIN, 0);
	motor.speed(RIGHT_MOTOR_PIN, 100);
	delay(400);

	motor.speed(LEFT_MOTOR_PIN, -100);
	motor.speed(RIGHT_MOTOR_PIN, -100);

	for (int index = 0; index < 2000; index++){

		int petDistance = analogRead(PET_DISTANCE_SENSOR_PIN);

		if (petDistance > NOISE_THRESHOLD){
			motor.stop_all();
			return;
		}

	}

}

void MoveFurther(){
	motor.speed(LEFT_MOTOR_PIN, 0);
	motor.speed(RIGHT_MOTOR_PIN, 100);
	delay(400);
	motor.speed(LEFT_MOTOR_PIN, 100);
	motor.speed(RIGHT_MOTOR_PIN, 100);
	delay(400);
	motor.speed(LEFT_MOTOR_PIN, 100);
	motor.speed(RIGHT_MOTOR_PIN, 0);
	delay(400);

	motor.speed(LEFT_MOTOR_PIN, -100);
	motor.speed(RIGHT_MOTOR_PIN, -100);

	for (int index = 0; index < 2000; index++){

		int petDistance = analogRead(PET_DISTANCE_SENSOR_PIN);

		if (petDistance > NOISE_THRESHOLD){
			motor.stop_all();
			return;
		}

	}
}
