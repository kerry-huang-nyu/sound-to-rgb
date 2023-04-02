const int PIN_RED   = 1;
const int PIN_GREEN = 22;
const int PIN_BLUE  = 23;

pinMode(PIN_RED,   OUTPUT);
pinMode(PIN_GREEN, OUTPUT);
pinMode(PIN_BLUE,  OUTPUT);

void set_rgb(int r, int g, int b) {
    analogWrite(PIN_RED,   r);
    analogWrite(PIN_GREEN, g);
    analogWrite(PIN_BLUE,  b);
}

void led_off() {
    analogWrite(PIN_RED,   0);
    analogWrite(PIN_GREEN, 0);
    analogWrite(PIN_BLUE,  0);
}

