Setup Notes
===========

Each CAN node is connected to a thermistor.  The Thermistor is on the top of the voltage divider as illustrated below:

3.3V ----- Thermistor ----- ADC ----- 10 kOhm Resistor ----- Gnd

The ADC that is used is the Curr pin which is pin 6.

The voltage --> temperature relationship was derived through experimentation.  Using a 3D Printer, the probe was attached to the print bed.  The Set Bed temperature was taken as "truth" and the the ADC voltage on the node was recorded relative to this temperature.  The curve fit tool "temp_curve_fit.py" was used to derive the polynomial coefficients needed for AP.

See the Excel doc for all of the recorded data.

Note that using the coefficients for the sensor to derive the resistance to temperature relationship did not yield accurate results (approx 9 deg difference), hence the calibration method above was adopted.

AP Customizations
=================

The HWdef for MatekL431-Periph in this branch enables temperature sensors on periph.

The HWdef for CubeOrangePlus in this branch enables temperature sensors.  Plane was built and used for the test rig.

Hardware
========

One Thermistor was added to one periph node.  Three periph nodes were used.  Due to the difficulty in getting the calibration, I did not want to add another resistor to the only 3.3V output and mess with the resistance relationship, potentially invalidating the calibration that had already been done.  Hence why there is only one temp sensor per node.

Key Parameters
==============

Temp Prob 1
-----------
- CAN_NODE, 20  (On CAN Node)
- MSG_ID, 20  (On CAN Node)
- BATT_SERIAL_NUM, 30 (On Cube)
- TEMP1_MSG_ID, 20 (On Cube)
- TEMP1_SRC, 4 = Battery ID/Serial Number (On Cube)
- TEMP1_SRC_ID, 30 (On Cube)

Temp Prob 2
-----------
- CAN_NODE, 21  (On CAN Node)
- MSG_ID, 21  (On CAN Node)
- BAT2_SERIAL_NUM, 31 (On Cube)
- TEMP2_MSG_ID, 21 (On Cube)
- TEMP2_SRC, 4 = Battery ID/Serial Number (On Cube)
- TEMP2_SRC_ID, 31 (On Cube)

Temp Prob 3
-----------
- CAN_NODE, 22  (On CAN Node)
- MSG_ID, 22  (On CAN Node)
- BAT2_SERIAL_NUM, 32 (On Cube)
- TEMP3_MSG_ID, 22 (On Cube)
- TEMP3_SRC, 4 = Battery ID/Serial Number (On Cube)
- TEMP3_SRC_ID, 32 (On Cube)


Labeling
========

All temperature probes are labelled according to the battery monitor that they are being logged under.

