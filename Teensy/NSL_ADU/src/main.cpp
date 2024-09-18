/**
 * In trying to modify this code, most of the modifications that need to be made should be made in the MX_falling code. This code is where the falling edge of th e
 * PWM signal is captured and is where the necessary messages get published. 
*/
#include "uavcan.h"

/* Calibration Mode */
#define CALIBRATION	true

/* Define the pin numbers */
#define ALPHA_PIN 2
#define BETA_PIN  1

/* Interrupt functions and variables */
void alpha_rising(); void alpha_falling();
void beta_rising();  void beta_falling();

/* Vane angle PWM values and signal times */
volatile uint32_t alpha_PWM = 0, beta_PWM = 0;
uint32_t alpha_prev_time, beta_prev_time; // shouldn't this be volatile as well?

/* Set up CAN node */
static constexpr uint32_t NODE_ID = 2;
static constexpr uint8_t SW_VER = 1;
static constexpr uint8_t HW_VER = 1;
static const char* NODE_NAME = "ADU";
static const uint32_t NODE_MEM = 8192;  // size of node memory
uavcan::CanDriver<1> *can;
uavcan::Node<NODE_MEM> *node;
uavcan::CanIface<CAN1> *can1;
uavcan::Publisher<uavcan::equipment::air_data::AngleOfAttack> *alpha_pub;
uavcan::Publisher<uavcan::equipment::air_data::Sideslip> *beta_pub;
uavcan::equipment::air_data::AngleOfAttack alpha_msg;
uavcan::equipment::air_data::Sideslip beta_msg;

/**
 * @section Calibration Data
 * radians = m*PWM + b
 */
//

// Calibration for ADU-200 on 07/30/2024
/* float b_alpha = 3.0723432;
float b_beta  = -3.23126372;
float m_alpha = -0.0014945726;
float m_beta  = 0.00153231; */

// Calibration for ADU-201 on 07/30/2024
float b_alpha = 3.47091511;
float b_beta  = -3.25409781;
float m_alpha = -0.00155833;
float m_beta  = 0.00151306;
//
// Calibration for ADU-202 on 07/30/2024
/* float b_alpha = 3.08397362;
float b_beta  = -2.90423762;
float m_alpha = -0.00147747;
float m_beta  = 0.00148731; */

// Calibration for ADU-203 on 07/30/2024
/* float b_alpha = 3.19263333;
float b_beta  = -3.18490733;
float m_alpha = -0.00156451;
float m_beta  = 0.00151182; */

// Calibration for ADU-204 on 07/30/2024
/* float b_alpha = 3.47091511;
float b_beta  = -2.76039883;
float m_alpha = -0.00155833;
float m_beta  = 0.00157377; */

/**
 * @section Interrupt functions for alpha and beta vanes
 * @link https://www.benripley.com/diy/arduino/three-ways-to-read-a-pwm-signal-with-arduino/
 */
void alpha_rising() {
	attachInterrupt(ALPHA_PIN, alpha_falling, FALLING);
	alpha_prev_time = micros();
}
void beta_rising() {
	attachInterrupt(BETA_PIN, beta_falling, FALLING);
	beta_prev_time = micros();
}
void alpha_falling() {
	attachInterrupt(ALPHA_PIN, alpha_rising, RISING);
	alpha_PWM = micros()-alpha_prev_time;

	/* Assign value to the message */
	alpha_msg.aoa = m_alpha*alpha_PWM + b_alpha;

	if (alpha_pub->broadcast(alpha_msg) < 0) {
		Serial.println("WARNING: Issue publishing AngleOfAttack message");
	} else {
		// Debugging only
		/* Serial.print("a,");
		Serial.print(alpha_PWM);
		Serial.print(",");
		Serial.println(micros()); */
	}
}
void beta_falling() {
	attachInterrupt(BETA_PIN, beta_rising, RISING);
	beta_PWM = micros()-beta_prev_time;

	/* Assign value to the message */
	beta_msg.sideslip_angle = m_beta*beta_PWM + b_beta;

	if (beta_pub->broadcast(beta_msg) < 0) {
		Serial.println("WARNING: Issue publishing Sideslip message");
	} else {
		// Debugging only
		/* Serial.print("b,");
		Serial.print(beta_PWM);
		Serial.print(",");
		Serial.println(micros()); */
	}
}


void setup() {
	/* Begin serial interface for debugging and calibration */
	Serial.begin(115200);

	/* Initialize CAN interface */
	can1 = new uavcan::CanIface<CAN1>;
	can1->begin();
	can1->setBaudRate(1000000);

	/* Initialize CAN driver */
	can = new uavcan::CanDriver<1>({can1});

	/* Initialize CAN node */
	node = new uavcan::Node<NODE_MEM>(*can, uavcan::clock);
	uavcan::protocol::SoftwareVersion sw_ver;
	uavcan::protocol::HardwareVersion hw_ver;
	sw_ver.major = SW_VER;
	sw_ver.minor = 0;
	hw_ver.major = HW_VER;
	hw_ver.minor = 0;
	node->setNodeID(NODE_ID);
	node->setName(NODE_NAME);
	node->setSoftwareVersion(sw_ver);
	node->setHardwareVersion(hw_ver);
	if (node->start() < 0) {
		Serial.println("ERROR starting node");
		while (1) {}
	}
	Serial.println("Node initialized");

	/* Initialize publishers */
	alpha_pub = new uavcan::Publisher<uavcan::equipment::air_data::AngleOfAttack>(*node);
	if (alpha_pub->init() < 0) {
		Serial.println("ERROR initializing AngleOfAttack publisher");
		while (1) {}
	}
	beta_pub = new uavcan::Publisher<uavcan::equipment::air_data::Sideslip>(*node);
	if (beta_pub->init() < 0) {
		Serial.println("ERROR initializing Sideslip publisher");
		while (1) {}
	}
	Serial.println("Publishers initialized");

	/* CAN acceptance filters */
	uavcan::configureCanAcceptanceFilters(*node);
	
	/* Set the PWM interrupt pins and attach the interrupts */
	pinMode(ALPHA_PIN, INPUT);
	pinMode(BETA_PIN, INPUT);
	attachInterrupt(ALPHA_PIN, alpha_rising, RISING);
	attachInterrupt(BETA_PIN, beta_rising, RISING);
	
	/* Set CAN node mode to operational */
	node->setModeOperational();
	Serial.println("Setup complete");
	digitalWrite(LED_BUILTIN, 1);
	Serial.println(micros());

	/* If calibration mode, get samples of vane data and spit it out */
	#if CALIBRATION
		while(true) {
			delay(1000);
			int n = 100;
			long alphaPWM_sum = 0;
			long betaPWM_sum = 0;
			for (int i=0; i<n; i++) {
				delay(1);
				alphaPWM_sum = alphaPWM_sum + alpha_PWM;
				betaPWM_sum = betaPWM_sum + beta_PWM;
			}
			int alphaPWM_avg = alphaPWM_sum/n;
			int betaPWM_avg = betaPWM_sum/n;
			Serial.println("###############");
			Serial.print("alpha = ");
			Serial.println(alphaPWM_avg);
			Serial.print("beta = ");
			Serial.println(betaPWM_avg);
			Serial.println("###############");
			delay(1000);
		}
	#endif
	
}

void loop() {
	/* delay for stability */
	delay(2); // Is there a better way to do this?
}

