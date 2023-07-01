#include <Servo.h>

#define TRIG_PIN_FRONT 26
#define ECHO_PIN_FRONT 28
#define TRIG_PIN_RIGHT 34
#define ECHO_PIN_RIGHT 36
#define TRIG_PIN_LEFT 30
#define ECHO_PIN_LEFT 32
#define TRIG_PIN_HEAD 22
#define ECHO_PIN_HEAD 24

#define redPin 23
#define bluePin 25
#define greenPin 27

#define S0 2
#define S1 3
#define S2 4
#define S3 5
#define Out 6

int redFrequency = 0;
int greenFrequency = 0;
int blueFrequency = 0;

#define BUTTON_PIN 50
#define LED_PIN 13

#define IN1 9
#define IN2 12
#define IN3 11
#define IN4 10



#define MIN_DISTANCE 15
#define MIN_DISTANCE_2 10
#define MOVE_TIME 200
#define STOP_TIME 50
#define SOUND_SPEED 0.034

Servo myservo_1;
Servo myservo_2;
int angle = 0;
int count = 0;

bool running = false;

void setup() {

  Serial.begin(9600);

  myservo_1.attach(7);  //上面的舵机
  myservo_2.attach(8);  //下面的舵机

  delay(2500);

  pinMode(BUTTON_PIN, INPUT);

  pinMode(TRIG_PIN_FRONT, OUTPUT);
  pinMode(ECHO_PIN_FRONT, INPUT);
  pinMode(TRIG_PIN_LEFT, OUTPUT);
  pinMode(ECHO_PIN_LEFT, INPUT);
  pinMode(TRIG_PIN_RIGHT, OUTPUT);
  pinMode(ECHO_PIN_RIGHT, INPUT);
  pinMode(TRIG_PIN_HEAD, OUTPUT);
  pinMode(ECHO_PIN_HEAD, INPUT);
  pinMode(LED_PIN, OUTPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(Out, INPUT);

  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
}

void loop() {

  int buttonState = digitalRead(BUTTON_PIN);

  if (buttonState == LOW) {
    Serial.println(running);
    running = !running;
    delay(500);
  }

  if (running) {
    float distance_front = get_distance_front();
    float distance_left = get_distance_left();
    float distance_right = get_distance_right();
    float distance_head = get_distance_head();

    int fr_cango = (distance_front > MIN_DISTANCE);
    int ri_cango = (distance_right > MIN_DISTANCE_2);
    int le_cango = (distance_left > MIN_DISTANCE_2);

    if (distance_head < 12) {
      digitalWrite(LED_PIN, HIGH);
      find_color();
      delay(8000);

      turnBack();
      delay(MOVE_TIME);
      stop();
      delay(STOP_TIME);

      servo_turn();
      delay(2000);
      digitalWrite(LED_PIN, LOW);
      delay(2000);

      turn_180();
    } else {
      action_choose(fr_cango, ri_cango, le_cango, distance_right, distance_left);
    }
  }
}

void action_choose(int fr_cango, int ri_cango, int le_cango, float distance_right, float distance_left) {

  if (fr_cango && ri_cango && le_cango) {
    forward();
    delay(MOVE_TIME);
    stop();
    delay(STOP_TIME);
  } else {
    if (!ri_cango && le_cango) {
      if (!fr_cango) {
        turnBack();
        delay(MOVE_TIME);
        stop();
        delay(STOP_TIME);
      }
      turnLeft();
      delay(MOVE_TIME);
      stop();
      delay(STOP_TIME);
    }

    if (ri_cango && !le_cango) {
      if (!fr_cango) {
        turnBack();
        delay(MOVE_TIME);
        stop();
        delay(STOP_TIME);
      }
      turnRight();
      delay(MOVE_TIME);
      stop();
      delay(STOP_TIME);
    }

    if (!ri_cango && !le_cango) {
      if (fr_cango) {
        turnBack();
        delay(MOVE_TIME);
        stop();
        delay(STOP_TIME);
      }
      turnBack();
      delay(MOVE_TIME * 1.5);
      stop();
      delay(STOP_TIME);

      if (count == 2) {
        turnRight();
        delay(MOVE_TIME * 4);
        count = 0;
      } else {
        turnLeft();
        delay(MOVE_TIME * 2);
        count++;
      }
      stop();
      delay(STOP_TIME);
    }

    if (ri_cango && le_cango && !fr_cango) {
      turnBack();
      delay(MOVE_TIME * 1.5);
      stop();
      delay(STOP_TIME);

      if (distance_left < distance_right) {
        turnRight();
        delay(MOVE_TIME);
        stop();
        delay(STOP_TIME);
      } else {
        turnLeft();
        delay(MOVE_TIME);
        stop();
        delay(STOP_TIME);
      }
    }
  }
}

float get_distance_front() {

  digitalWrite(TRIG_PIN_FRONT, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN_FRONT, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN_FRONT, LOW);

  long pulse_duration = pulseIn(ECHO_PIN_FRONT, HIGH);

  float distance = (pulse_duration * SOUND_SPEED) / 2;
  Serial.print("Distance_front: ");
  Serial.print(distance);
  Serial.println(" cm");

  return distance;
}

float get_distance_left() {

  digitalWrite(TRIG_PIN_LEFT, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN_LEFT, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN_LEFT, LOW);

  long pulse_duration = pulseIn(ECHO_PIN_LEFT, HIGH);

  float distance = (pulse_duration * SOUND_SPEED) / 2;
  Serial.print("Distance_left: ");
  Serial.print(distance);
  Serial.println(" cm");

  return distance;
}

float get_distance_right() {

  digitalWrite(TRIG_PIN_RIGHT, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN_RIGHT, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN_RIGHT, LOW);

  long pulse_duration = pulseIn(ECHO_PIN_RIGHT, HIGH);

  float distance = (pulse_duration * SOUND_SPEED) / 2;
  Serial.print("Distance_right: ");
  Serial.print(distance);
  Serial.println(" cm");

  return distance;
}

float get_distance_head() {
  digitalWrite(TRIG_PIN_HEAD, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN_HEAD, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN_HEAD, LOW);

  long pulse_duration = pulseIn(ECHO_PIN_HEAD, HIGH);

  float distance = (pulse_duration * SOUND_SPEED) / 2;
  Serial.print("Distance_head: ");
  Serial.print(distance);
  Serial.println(" cm");

  return distance;
}


void stop() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
void forward() {
  analogWrite(IN1, 0);
  analogWrite(IN2, 110);
  analogWrite(IN3, 0);
  analogWrite(IN4, 110);
}
void turnLeft() {
  analogWrite(IN1, 115);
  analogWrite(IN2, 0);
  analogWrite(IN3, 0);
  analogWrite(IN4, 115);
}
void turnRight() {
  analogWrite(IN1, 0);
  analogWrite(IN2, 115);
  analogWrite(IN3, 115);
  analogWrite(IN4, 0);
}
void turnBack() {
  analogWrite(IN1, 110);
  analogWrite(IN2, 0);
  analogWrite(IN3, 110);
  analogWrite(IN4, 0);
}
void turn_180() {
  turnRight();
  delay(MOVE_TIME * 4);
  stop();
  delay(STOP_TIME);
  turnBack();
  delay(MOVE_TIME);
  turnRight();
  delay(MOVE_TIME * 4);
  stop();
  delay(STOP_TIME);
}

void color(unsigned char red, unsigned char green, unsigned char blue) {
  int min = 9999;
  int min_color;
  if (red < min) {
    min = red;
    min_color = 1;
  }
  if (green < min) {
    min = green;
    min_color = 0;
  }
  if (blue < min) {
    min = blue;
    min_color = 2;
  }

  switch (min_color) {
    case 1:
      analogWrite(redPin, 255);
      analogWrite(bluePin, 0);
      analogWrite(greenPin, 0);
      break;
    case 2:
      analogWrite(redPin, 0);
      analogWrite(bluePin, 255);
      analogWrite(greenPin, 0);
      break;
    case 0:
      analogWrite(redPin, 0);
      analogWrite(bluePin, 0);
      analogWrite(greenPin, 255);
      break;
  }
}

void find_color() {
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);

  redFrequency = pulseIn(Out, LOW);
  delay(100);

  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  greenFrequency = pulseIn(Out, LOW);
  delay(100);

  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  blueFrequency = pulseIn(Out, LOW);
  delay(100);

  Serial.print("Red: ");
  Serial.print(redFrequency);
  Serial.print(" Green: ");
  Serial.print(greenFrequency);
  Serial.print(" Blue: ");
  Serial.println(blueFrequency);

  color(255 - redFrequency,255 - greenFrequency,255 - blueFrequency);
  delay(1000);
}

void smooth_servo_turn(Servo &servo, int start_angle, int end_angle) {
  int step = start_angle < end_angle ? 1 : -1;

  for (angle = start_angle; angle != end_angle; angle += step) {
    servo.write(angle);
    delay(30);
  }
}

void servo_turn() {
  smooth_servo_turn(myservo_2, 90, 135);
  delay(1000);
  smooth_servo_turn(myservo_1, 90, 45);
  smooth_servo_turn(myservo_1, 45, 90);

  smooth_servo_turn(myservo_2, 135, 90);
  delay(1000);
  smooth_servo_turn(myservo_1, 90, 45);
  smooth_servo_turn(myservo_1, 45, 90);

  smooth_servo_turn(myservo_2, 90, 45);
  delay(1000);
  smooth_servo_turn(myservo_1, 90, 45);
  smooth_servo_turn(myservo_1, 45, 90);

  smooth_servo_turn(myservo_2, 45, 90);
  delay(3000);
}