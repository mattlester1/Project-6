This is the last project from a mechatronics class. The task was to implement PID (proportional–integral–derivative) control. For this project the focus was on the PI aspects of the PID control method.

The goal was to control a motor so that the shaft of the motor would remain in the same orientation as the motor was rotated about its long axis.

The program as written, along with the motor, micro-controller, gyro, and several other electical components achomplished this task.

The micro-controller, when powered on, will keep the motor shaft in its current orientation by acuating the motor in the oppoisite direction of long axis rotation.

It takes into account the proportion of roation or the delta of the angle of rotaion relative to the desired orientation. It then uses this reading
in the proportional aspect of the control.

The micro-controller also takes in how long the given delta has been present for. The longer a given delta remains present the greater response from the
micro-controller/ motor.

***

I think of PI control like lifting weights (this is not a perfect example):

If you are required to lift anm unknown weight to a certain height, say 3 feet off the ground then you would respond by attempting to move that weight
to that point in the air. This is the proportional aspect of PID control. You see that you need to get the weight to a certain point so you apply force.

Since the weight is unknown, you apply as much force as you can. If the weight was extremly light then you may move the weight far beyond the 3ft mark. 
Now when you realize you have gone far beyound your mark you immidietly start applying force in the opposite direction to bring it back to the 3ft mark.
Again, you don't know how much the weight weighs so you apply all your force. This loop continues as you have no way to determine how much force is needed 
as you do not know the weight.

Thuis is where the integral aspect comes in. Instead of applying force all at once this aspect of the control takes into account time. For instance say you
attempted to lift the weight from the floor, but did so with not enough force to lift it. This occures over the course of several seconds and as time moves on
you realize you need to apply more force. You continue to apply more and more force until it moves. You get the weight to move enough so that it moves beyond
the 3ft mark. Now that it's there, because of the P aspect you know you must move in the oppoiste direction, but because you now have the I aspect you can apply
a more appropirate amount of force because you haven't been applying force that long on this side of the 3ft mark. So you will move back torwards the 3ft mark at 
a slower rate. You will probably move beyond the 3ft mark again, but you will get closer and closer with each loop since the delta is growing smaller and smaller
and remaining for shorter and shorter amounts of time.

A micro-controller will do this loop extreamly fast so that there is virtually no lag in the ability to control the system.
