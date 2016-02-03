# HomeAlarm
Microcontroller Home Alarm Developed in MPLAB XIDE for PIC32 Microcontroller on Digilent Cerebot 32MX4. 
Utilizes microphone, keypad, 7 segment display. All peripheral modules were created by Digilient.

Takes input from keypad and microphone
\n \t uses change notice interrupts to receive input from keyboard
\n \t uses ADC on Digilent Cerebot 32MX4 to handle microphone input

Security Logic
\n \t set numerical passcode using keypad
\n \t uses FFT to process microphone signal, a frequency equal to the passcode in Hz (within 3% tolerance) will go to unlocked mode
\n \t timer interrupt and key pad handling governs output to two 7-segment displays
\n \t alarm is tripped due to: wrong passcode, loud noise, frequency outside of acceptable range.