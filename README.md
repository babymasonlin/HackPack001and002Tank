# This is a hardware _and_ a software hack, so make sure you get the code and the robots so you can use this.    
This is also on [Discord](discord.com/channels/@me) [here](https://discord.com/channels/1229106258749948056/1361423311241875459)    
IR Turret: https://www.crunchlabs.com/products/ir-turret?srsltid=AfmBOoqU-YAXiucbCrwmlKRUluHFtphfJTySFhiZfJcTfiuNTRA7lTDR          
Domino Robot: https://www.crunchlabs.com/products/domino-robot?srsltid=AfmBOopcUY8UZAyyH0F-sZIc2p7RFCtc4osDoRD8OrSwpJsaKTUNULPe    
## Instructions for assembly:
1. Take off the line sensors, servo, and limit switch(the black one with a metal bar sticking out) from the domino robot.
2. Take off all of the legs of the IR Turret except one. Take off the orange rubber pad on that leg.
3. For that leg that's still on the turret, loosen the nut on top of the board holding it, then rotate counter-clockwise 90°. Tighten the nut again.
4. Put that leg inside the domino storage area.
5. Take off the Acrylic Gate(The transparent red part) and the black wood piece holding it. Screw the nuts back on(Leave the steel bars on the robot).
6. Loosen the black rod(on top of the ball roller) and add the 2 extra red washers to the long red screw, and tighten the black rod back on the board.
7. Remove the Breadboard, Arduino, and power bank from the domino robot
8. Using the red short screws from the domino robot, screw in the orange nuts across from each other.  When looking from the front of the domino robot, the leg that is still on the IR Turret should be on the right side of the robot.
9. When looking from the front of the domino robot, take off the bottom left screw and replace with a red washer, and put an orange screw through the square hole on the bottom board of the IR Turret and though the washer and the red rod under the rectangle board(that was used for holding the power bank). It's a little hard
10. Take the power bank holder(the transparent plastic) off of the turret, and put it between the breadboard and the yaw servo.Use tape if you want to
11. Put the 4 wires of the Servos of the turret into the breadboard as follows: Yellow wire -> D12; Green wire -> D11; Blue wire -> D10; Purple wire -> D9
Put the H-Bridge motor driver in the rest of the free spaces so that the four 3R3 blocks and C106 block are facing the back of the domino robot.
Upload the code(Included below) and you're done!

### IR Remote Controls:    
1 -> Nothing    
2 -> Drive Forward    
3 -> Nothing    
4 -> Drive Left    
5 -> Nothing    
6 -> Drive Right    
7 -> Nothing    
8 -> Drive Backward    
9 -> Nothing    
0 -> Nothing    
'*' -> Nothing    
'#' -> Fire all bullets    
↑ Button -> Move barrel up    
↓ Button -> Move barrel down    
← Button -> Move barrel left    
→ Button -> Move barrel right    
🆗 Button -> Fire one bullet    
