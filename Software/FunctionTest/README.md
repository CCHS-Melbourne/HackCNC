This sketch is a simple HackCNC function test. 

It waits for serial connection (115200 8n1) and then draws a 10mm square when it receives an 's' or 'S'. Anyother input results in no action but updates the LCD.

It really should work with digitalWriteFast, but for some reason, it just isn't right now. Look in to it if you have time.

----------
This software is based on the work of others. Attribution and other information can be found as follows:
digitalWriteFast - http://code.google.com/p/digitalwritefast/
NewLiquidCrystal - https://github.com/marcmerlin/NewLiquidCrystal
