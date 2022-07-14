/*
 * File: MG90S_servo.c
 * Author: Bhuml Depani
 * Brief: This file is a motor driver module for the MG90S Servo Motor. It
 * contains multiple functions which can be used to control the rotation of the
 * servo motor.
 * Created on 06/22/2022 04:28 pm
*/

/*******************************************************************************
 * #INCLUDES                                                                   *
 ******************************************************************************/

#include "MG90S_servo.h"                // Header file for MG90S_servo.c
#include "hardware/pwm.h"               //Pico PWM Library
#include <stdio.h>                      //C standard Input Output Library

/*******************************************************************************
 * PRIVATE #DEFINES                                                            *
 ******************************************************************************/

#define MOTOR_PIN 0                 // GPIO pin number where Motor is connected
#define MIN_MICROSECOND 440         /* As per datasheet 2% of duty cycle
* should be needed to move servo motor at -90 degree angle. For 50 Hz Servo
* motor, PWM period is 20,000 microsecond. So, 2% of 20,000 is 400. So, our
* minimium PWM pulse width could be around 400. From experiment, we found pulse
* width is 440 microsecond (2.20% duty cycle) */
#define MAX_MICROSECOND 2560        /* As per datasheet 12.5% of duty cycle
* should be needed to move servo motor at 90 degree angle. For 50 Hz Servo
* motor, PWM period is 20,000 microsecond. So, 12.5% of 20,000 is 2500. So, our
* maximum PWM pulse width could be around 2500. From experiment, we found
* pulse width is 2560 microsecond (12.80% duty cycle)*/
#define CENTER_MICROSECOND 1500      /* As most servo motors, MG90S servo motor
* also has center angle at 1500 microsecond*/ 
#define PWM_PERIOD_HZ 50             /* PWM period required by motor in Hertz,
* can find this value from datasheet of particular motor*/

#define ANGLE_RANGE 90          // MG90S Servo motor angle range is +90 to -90
#define FREQUENCY_DIVISOR 40    /* This would be remain static for 50 Hz servo
* motor. To know how we got 40 checkout Driver Development document.*/

/*******************************************************************************
 * PRIVATE VARIABLES                                                            
 ******************************************************************************/

int16_t servo_angle;              // global variable for servo angle
uint16_t pwm_wrap_count;        // global variable for PWM Wrap Count;

/*******************************************************************************
 * PRIVATE FUNCTIONS PROTOTYPES                                                 
 ******************************************************************************/

/*******************************************************************************
 * PUBLIC FUNCTION IMPLEMENTATIONS                                             *
 ******************************************************************************/

/*
 * @Function void MG90S_servo_init(void)
 * @param None
 * @return None
 * @brief Initializes PWM with the required PWM Period for motor at pin
 * MOTOR_PIN
 * @author Bhumil Depani
 */
void MG90S_servo_init(void)
{
    stdio_init_all();                           // initializes Pico I/O pins
    gpio_set_function(MOTOR_PIN, GPIO_FUNC_PWM);// to set MOTOR_PIN as PWM pin

    uint slice_num = pwm_gpio_to_slice_num(MOTOR_PIN);     /* to get a slice
    * number associated with a MOTOR_PIN */

    uint pwm_clock_freq;        /* pwm_clock_freq refers to the PWM frequency 
    * we will get after dividing Pico System Clock frequency */

    pwm_clock_freq = SYSTEM_CLOCK_FREQ / FREQUENCY_DIVISOR;
    pwm_wrap_count = pwm_clock_freq / PWM_PERIOD_HZ;
    
    pwm_config config = pwm_get_default_config();   /* to get default PWM 
    * configuration */

    pwm_config_set_clkdiv_int(&config, FREQUENCY_DIVISOR);
    pwm_config_set_wrap(&config, pwm_wrap_count);   // set PWM_wrap_count

    pwm_init(slice_num, &config, true);          /* initialize pwm pin/slice
    * number with updated configuration, start PWM signal to the MOTOR_PIN */
}

/*
 * @Function void MG90S_servo_set_angle(int16_t angle_degree)
 * @param angle_degree, an angle, in degree, where we want servo to rotate, for
 * MG90S Servo Motor angle should be between -9000 and 9000 centidegree
 * @return None
 * @brief takes an angle in degree, finds PWM duty cycle required, set PWM duty
 * cycle to the MOTOR_PIN 
 * @author Bhumil Depani
 */
void MG90S_servo_set_angle(int16_t angle_degree)
{   
    servo_angle = angle_degree;     /* save angle_degree to further use in 
    * MG90S_servo_get_angle function */
    uint base_microsecond = CENTER_MICROSECOND;

    uint range_microsecond;

    if(angle_degree >= 0)
        range_microsecond = MAX_MICROSECOND - CENTER_MICROSECOND;
    else
        range_microsecond = CENTER_MICROSECOND - MIN_MICROSECOND;
    
    float per_angle_microsecond = range_microsecond / ANGLE_RANGE;

    float required_microsecond = base_microsecond + (per_angle_microsecond * 
    (angle_degree)/100.0); /* here, converting an input centidegree into 
    * degree.And degree into width of high pulse in PWM signal in microsecond */
    
    uint16_t pwm_count = microsecond_to_pwm_count(required_microsecond);

    pwm_set_gpio_level(MOTOR_PIN, pwm_count);       /* to set PWM duty cycle at 
    * the MOTOR_PIN */
}

/*
 * @Function uint16_t microsecond_to_pwm_count(float microsecond)
 * @param microsecond, microsecond will be converted into appropriate PWM Count
 * @return PWM Count associated with output microsecond
 * @brief takes microsecond as an input and according to values of 
 * PWM_PERIOD_HZ and pwm_wrap_count, the function will calculate the pwm count*
 * @author Bhumil Depani
 */
uint16_t microsecond_to_pwm_count(float microsecond)
{
    float pwm_period_microsecond = (1.f / PWM_PERIOD_HZ) * MHz;
    uint16_t pwm_count = (microsecond * pwm_wrap_count) / 
    (pwm_period_microsecond); /* As pwm_period_microsecond is directly 
    * proportional to pwm_wrap_count, microsecond is directly proportional to
    * pwm_count*/
    return pwm_count;
}

/*
 * @Function int16_t MG90S_servo_get_angle(void)
 * @param None
 * @return angle of the servo motor in centidegree
 * @brief function can be called to know at which angle servo is remained
 * @author Bhumil Depani
 */
int16_t MG90S_servo_get_angle(void)
{
    return servo_angle;
}

#ifdef MG90SSERVO_TESTING

void main()
{
    MG90S_servo_init();
    while (1)
    {
        MG90S_servo_set_angle(0);
        printf("\nAngle at this time: %10d", MG90S_servo_get_angle());
        sleep_ms(5000);
        MG90S_servo_set_angle(-9000);
        printf("\nAngle at this time: %10d", MG90S_servo_get_angle());
        sleep_ms(5000);
        MG90S_servo_set_angle(9000);
        printf("\nAngle at this time: %10d", MG90S_servo_get_angle());
        sleep_ms(5000);
        MG90S_servo_set_angle(-4500);
        printf("\nAngle at this time: %10d", MG90S_servo_get_angle());
        sleep_ms(5000);
        MG90S_servo_set_angle(0);
        printf("\nAngle at this time: %10d", MG90S_servo_get_angle());
        sleep_ms(5000);

        for(int count = -9000; count <= 9000; count = count + 100)
        {
            MG90S_servo_set_angle(count);
            printf("\nAngle at this time: %10d", MG90S_servo_get_angle());
            sleep_ms(5000);
        }
    }
}

#endif  //MG90SSERVO_TESTING