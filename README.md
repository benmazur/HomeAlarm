# HomeAlarm
Microcontroller Home Alarm Developed in MPLAB XIDE for PIC32 Microcontroller on Digilent Cerebot 32MX4. 
Utilizes microphone, keypad, 7 segment display. All peripheral modules were created by Digilient.

-Takes input from keypad and microphone
-uses change notice interrupts to receive input from keyboard
-uses on-board ADC from Digilent Cerebot 32MX4 to handle microphone input
-set numerical passcode using keypad
-uses FFT to process microphone signal, a frequency equal to the passcode in Hz (within 3% tolerance) will go to unlocked mode
-timer interrupt and key pad handling governs output to two 7-segment displays
-alarm is tripped due to: wrong passcode, loud noise, frequency outside of acceptable range.
