unsigned long now;
unsigned long time_stamp;
unsigned long time_start = 47640000;
unsigned int seconds;
unsigned int minutes;
unsigned int hours;
bool exit_cond = false;
char date_stamp[11] = "2019/9/16 "; //extra byte for trailing null value

// List of 'data parameters' being output from sensor
float data_A_f = 65.7;
float data_B_f = 23.5;
int data_C = 106;
int data_D = 75;
int data_E = 108;
int data_F = 77;

void setup ()
{
    Serial.begin (9600);
}


void loop()
{
while (exit_cond == false) {

    time_stamp = time_start + millis();

    // Break the time stamp down into HH:MM:SS
    hours = time_stamp/3600000;  // Automatically rounds down (ignores the decimal) as it is of type int
    minutes = time_stamp/60000 - hours*60;
    seconds = time_stamp/1000 - minutes*60 - hours*3600;

    // Increment data parameters.  Each one is done at a different gradient
    // to make it easier to see which one is which when plotting them out
    data_A_f = data_A_f + 0.1f;
    // reduce the number of dp printed in serial out
    int data_A_i_int = data_A_f;
    int data_A_i_dec = ((data_A_f-data_A_i_int)*10.0f);
    
    data_B_f = data_B_f + 0.2f;
    int data_B_i_int = data_B_f;
    int data_B_i_dec = ((data_B_f-data_B_i_int)*10.0f);

    data_C = data_C + 1;
    data_D = data_D + 2;
    data_E = data_E - 2;
    data_F = data_F - 2;


    // --- --- --- Send over serial --- --- ---
    // Build date-time stamp
    Serial.print(date_stamp);
    Serial.print(hours);
    Serial.print(":");
    if (minutes < 10) {
        // Prefix 0 to single digit minutes
        Serial.print("0");
    }
    Serial.print(minutes);
    Serial.print(":");
    if (seconds < 10) {
        // Prefix 0 to single digit seconds
        Serial.print("0");
    }
    Serial.print(seconds);

    // Data
    Serial.print(",");
    // Build and send data A to 1dp
    Serial.print(data_A_i_int);
    Serial.print(".");
    Serial.print(data_A_i_dec);

    // Build and send data B to 1dp
    Serial.print(",");
    Serial.print(data_B_i_int);
    Serial.print(".");
    Serial.print(data_B_i_dec);

    // Send remaining integer data
    Serial.print(",");
    Serial.print(data_C);
    Serial.print(",");
    Serial.print(data_D);
    Serial.print(",");
    delay(500);
    Serial.print(data_E);
    Serial.print(",");
    Serial.println(data_F);
    delay(500);

} //end while loop

}
