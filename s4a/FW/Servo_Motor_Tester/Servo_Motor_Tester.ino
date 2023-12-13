

#define DEG_STEP 10

/////
// LCD stuff.

#include <LiquidCrystal.h>

#define LCD_BL 10

LiquidCrystal lcd(
	8, // RS
	9, // EN
	4, // D4
	5, // D5
	6, // D6
	7  // D7
);

enum pb_t {
	PB_RIGHT,
	PB_UP,
	PB_DOWN,
	PB_LEFT,
	PB_SELECT,
	PB_NONE
};

pb_t read_LCD_Shield_PBs() {
	int x;
	x = analogRead(0);
	if(x < 60){
		return PB_RIGHT;
	}else if(x < 200){
		return PB_UP;
	}else if(x < 400){
		return PB_DOWN;
	}else if(x < 600){
		return PB_LEFT;
	}else if(x < 800){
		return PB_SELECT;
	}else{
		return PB_NONE;
	}
}

void test_LCD_Shield_PBs() {
	switch(read_LCD_Shield_PBs()){
	case PB_RIGHT:
		Serial.println("PB_RIGHT");
		break;
	case PB_UP:
		Serial.println("PB_UP");
		break;
	case PB_DOWN:
		Serial.println("PB_DOWN");
		break;
	case PB_LEFT:
		Serial.println("PB_LEFT");
		break;
	case PB_SELECT:
		Serial.println("PB_SELECT");
		break;
	default:
		Serial.println("PB_NONE");
		break;
	}
}

/////


#include <Servo.h>

Servo s;
int8_t deg;
void update_servo() {
	s.write(deg+90);
}


void setup() {
	Serial.begin(115200);
	s.attach(A1);
	deg = 0;
	update_servo();
	
	lcd.begin(16, 2);
}

void loop() {
	static pb_t prev_pb = PB_NONE;
	pb_t curr_pb = read_LCD_Shield_PBs();
	if(prev_pb == PB_NONE){
		switch(curr_pb){
			case PB_RIGHT:
				if(deg < 90){
					deg += DEG_STEP;
				}
				break;
			case PB_LEFT:
				if(deg > -90){
					deg -= DEG_STEP;
				}
				break;
			case PB_UP:
				deg = +90;
				break;
			case PB_DOWN:
				deg = -90;
				break;
			case PB_SELECT:
				deg = 0;
				break;
			default:
				break;
		}
	}
	prev_pb = curr_pb;
	update_servo();

	static int c = 0;
	if(c == 10-1){
		c = 0;
		lcd.clear();
		lcd.setCursor(0, 0);
		lcd.print(deg);
	}else{
		c++;
	}
	delay(10);
}
