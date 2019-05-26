#ifndef HCSR04ard_H
#define HCSR04ard_H

#define HCSR_01_ECHO	23
#define HCSR_01_TRIG	25

#define HCSR_02_ECHO	33
#define HCSR_02_TRIG	31

#define HCSR_03_ECHO	37
#define HCSR_03_TRIG	39

#define HCSR_04_ECHO	47
#define HCSR_04_TRIG	45

#define HCSR_05_ECHO	51
#define HCSR_05_TRIG	53

#define TimeOut			5000*4

void Ultrasonic_init(void);
void Ultrasonic_get_distance(float* distance);

#endif