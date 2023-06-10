/*
 *  Example of AP_Motors library.
 *  Code by Randy Mackay. DIYDrones.com
 */

/* on Linux run with
    ./waf configure --board linux
    ./waf --targets examples/AP_Motors_test
    ./build/linux/examples/AP_Motors_test
*/

// Libraries
#include <AP_HAL/AP_HAL.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_Motors/AP_Motors.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_ESC_Telem/AP_ESC_Telem.h>
#include <stdio.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// declare functions
void setup();
void loop();
void motor_order_test();
void stability_test();
void update_motors();

// Instantiate a few classes that will be needed so that the singletons can be called from the motors lib
#if HAL_WITH_ESC_TELEM
AP_ESC_Telem esc_telem;
#endif
SRV_Channels srvs;
AP_BattMonitor _battmonitor{0, nullptr, nullptr};

AP_Motors *motors;
AP_MotorsMatrix *motors_matrix;

bool thrust_boost = false;

uint8_t num_outputs;

// setup
void setup()
{
    hal.console->printf("AP_Motors library test ver 1.0\n");

    // default to quad frame class, frame class can be changed by argument in parser below
    AP_Motors::motor_frame_class frame_class = AP_Motors::MOTOR_FRAME_QUAD;

    // Parse the command line arguments
    uint8_t argc;
    char * const *argv;
    hal.util->commandline_arguments(argc, argv);
    if (argc > 1) {
        for (uint8_t i = 2; i < argc; i++) {
            const char *arg = argv[i];
            const char *eq = strchr(arg, '=');

            if (eq == NULL) {
                ::printf("Expected argument with \"=\"\n");
                exit(1);
            }

            char cmd[20] {};
            strncpy(cmd, arg, eq-arg);
            const float value = atof(eq+1);
            if (strcmp(cmd,"yaw_headroom") == 0) {
                if (motors_matrix != nullptr) {
                    motors_matrix->set_yaw_headroom(value);
                } else {
                    ::printf("frame_class = %d does not accept yaw_headroom commands\n", frame_class);
                }

            } else if (strcmp(cmd,"throttle_avg_max") == 0) {
                if (motors_matrix != nullptr) {
                    motors_matrix->set_throttle_avg_max(value);
                } else {
                    ::printf("frame_class = %d does not accept throttle_avg_max commands\n", frame_class);
                }

            } else if (strcmp(cmd,"thrust_boost") == 0) {
                thrust_boost = value > 0.0;

            } else if (strcmp(cmd,"frame_class") == 0) {
                // We must have the frame_class argument 2nd as resulting class is used to determine if
                // we have access to certain functions in the multicopter motors child class
                if (i != 2) {
                    ::printf("frame_class must be second argument\n");
                    exit(1);
                }

                // Setup the correct motors object for the frame class to test
                frame_class = (AP_Motors::motor_frame_class)value;

                switch (frame_class) {

                    case AP_Motors::MOTOR_FRAME_QUAD:
                    case AP_Motors::MOTOR_FRAME_HEXA:
                    case AP_Motors::MOTOR_FRAME_OCTA:
                        motors_matrix = new AP_MotorsMatrix(400);
                        motors = motors_matrix;
                        num_outputs = __builtin_popcount(motors->get_motor_mask());
                        break;

                    case AP_Motors::MOTOR_FRAME_HELI:
                    case AP_Motors::MOTOR_FRAME_HELI_DUAL:
                    case AP_Motors::MOTOR_FRAME_HELI_QUAD:
                        motors = new AP_MotorsHeli_Single(400);
                        num_outputs = 8;
                        break;

                    default:
                        ::printf("ERROR: frame_class=%d not implemented\n", frame_class);
                        exit(1);
                }

            // Infer number out of outputs

            motors->init(frame_class, AP_Motors::MOTOR_FRAME_TYPE_X);

            } else {
                ::printf("Expected \"frame_class\", \"yaw_headroom\" or \"throttle_avg_max\"\n");
                exit(1);
            }
        }

        // if we haven't been given a frame class by argument we just assume a quad by default
        // so that the single first argument s or t still works
        if (motors == nullptr) {
            motors_matrix = new AP_MotorsMatrix(400);
            motors = motors_matrix;
            motors->init(AP_Motors::MOTOR_FRAME_QUAD, AP_Motors::MOTOR_FRAME_TYPE_X);
            num_outputs = 4;
        }

        // Start a test type based on the input argument
        if (strcmp(argv[1],"t") == 0) {
            motor_order_test();

        } else if (strcmp(argv[1],"s") == 0) {
            stability_test();

        } else {
            ::printf("Expected first argument, 't' or 's'\n");
        }

        hal.scheduler->delay(1000);
        exit(0);

    } else {
        // We haven't been given a frame class so we just assume a quad frame as default
        motors_matrix = new AP_MotorsMatrix(400);
        motors = motors_matrix;
        motors->init(AP_Motors::MOTOR_FRAME_QUAD, AP_Motors::MOTOR_FRAME_TYPE_X);
        num_outputs = 4;
    }

    // motor initialisation
    motors->set_dt(1.0/400.0);
    motors->set_update_rate(490);

    char frame_and_type_string[30];
    motors->get_frame_and_type_string(frame_and_type_string, ARRAY_SIZE(frame_and_type_string));
    hal.console->printf("%s\n", frame_and_type_string);

    if (motors_matrix != nullptr) {
        motors_matrix->update_throttle_range();
        motors_matrix->set_throttle_avg_max(0.5f);
    }

    motors->output_min();

    hal.scheduler->delay(1000);
}

// loop
void loop()
{
    int16_t value;

    // display help
    hal.console->printf("Press 't' to run motor orders test, 's' to run stability patch test.  Be careful the motors will spin!\n");

    // wait for user to enter something
    while( !hal.console->available() ) {
        hal.scheduler->delay(20);
    }

    // get character from user
    value = hal.console->read();

    // test motors
    if (value == 't' || value == 'T') {
        motor_order_test();
        hal.console->printf("finished test.\n");
    }
    if (value == 's' || value == 'S') {
        stability_test();
        hal.console->printf("finished test.\n");
    }
}

// stability_test
void motor_order_test()
{
    hal.console->printf("testing motor order\n");
    motors->armed(true);
    for (int8_t i=1; i <= num_outputs; i++) {
        hal.console->printf("Motor %d\n",(int)i);
        motors->output_test_seq(i, 1150);
        hal.scheduler->delay(300);
        motors->output_test_seq(i, 1000);
        hal.scheduler->delay(2000);
    }
    motors->armed(false);

}

// stability_test
void stability_test()
{
    if (motors_matrix != nullptr) {
        hal.console->printf("Throttle average max: %0.4f\n",  motors_matrix->get_throttle_avg_max());
        hal.console->printf("Yaw headroom: %i\n", motors_matrix->get_yaw_headroom());
        hal.console->printf("Thrust boost: %s\n", thrust_boost?"True":"False");
    }

    const float throttle_tests[] = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
    const uint8_t throttle_tests_num = ARRAY_SIZE(throttle_tests);
    const float rpy_tests[] = {-1.0, -0.9, -0.8, -0.7, -0.6, -0.5, -0.4, -0.3, -0.2, -0.1, 0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0};
    const uint8_t rpy_tests_num = ARRAY_SIZE(rpy_tests);

    // arm motors
    motors->armed(true);
    motors->set_interlock(true);
    SRV_Channels::enable_aux_servos();

    hal.console->printf("Roll,Pitch,Yaw,Thr,");
    for (uint8_t i=0; i<num_outputs; i++) {
        hal.console->printf("Mot%i,",i+1);
    }

    if (motors_matrix != nullptr) {
        for (uint8_t i=0; i<num_outputs; i++) {
            hal.console->printf("Mot%i_norm,",i+1);
        }
    }

    hal.console->printf("LimR,LimP,LimY,LimThD,LimThU\n");

    // run stability test
    for (uint8_t y=0; y<rpy_tests_num; y++) {
        for (uint8_t p=0; p<rpy_tests_num; p++) {
            for (uint8_t r=0; r<rpy_tests_num; r++) {
                for (uint8_t t=0; t<throttle_tests_num; t++) {
                    const float roll_in = rpy_tests[r];
                    const float pitch_in = rpy_tests[p];
                    const float yaw_in = rpy_tests[y];
                    const float throttle_in = throttle_tests[t];
                    motors->set_roll(roll_in);
                    motors->set_pitch(pitch_in);
                    motors->set_yaw(yaw_in);
                    motors->set_throttle(throttle_in);
                    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);
                    update_motors();
                    SRV_Channels::calc_pwm();
                    // display input and output
                    hal.console->printf("%0.2f,%0.2f,%0.2f,%0.2f,", roll_in, pitch_in, yaw_in, throttle_in);
                    for (uint8_t i=0; i<num_outputs; i++) {
                        hal.console->printf("%d,",(int)hal.rcout->read(i));
                    }

                    if (motors_matrix != nullptr) {
                        for (uint8_t i=0; i<num_outputs; i++) {
                            hal.console->printf("%0.4f,", motors_matrix->get_thrust_rpyt_out(i));
                        }
                    }

                    hal.console->printf("%d,%d,%d,%d,%d\n",
                            (int)motors->limit.roll,
                            (int)motors->limit.pitch,
                            (int)motors->limit.yaw,
                            (int)motors->limit.throttle_lower,
                            (int)motors->limit.throttle_upper);
                }
            }
        }
    }

    // set all inputs to motor library to zero and disarm motors
    motors->set_pitch(0);
    motors->set_roll(0);
    motors->set_yaw(0);
    motors->set_throttle(0);
    motors->armed(false);

}

void update_motors()
{
    // call update motors 1000 times to get any ramp limiting complete
    for (uint16_t i=0; i<1000; i++) {
        motors->set_thrust_boost(thrust_boost);
        motors->output();
    }
}

AP_HAL_MAIN();
