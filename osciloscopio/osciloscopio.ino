// --- Osciloscopio básico con ESP32 ---
// Lee una señal analógica y la grafica por el Serial Plotter

const int pinADC = 34;          // Pin analógico (ADC1_CH6)
const int sampleRate = 20000;   // 20 kHz de muestreo -> sirve hasta 2 kHz de señal
unsigned long samplePeriodUs;

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);     // 12 bits -> 0..4095
  samplePeriodUs = 1000000UL / sampleRate;
  delay(500);
}

void loop() {
  unsigned long t0 = micros();
  int value = analogRead(pinADC);    // Lectura ADC
  Serial.println(value);             // Envío al Serial Plotter

  // Espera para mantener la frecuencia de muestreo constante
  while (micros() - t0 < samplePeriodUs);
}
