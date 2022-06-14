# TMInterface custom scripts by SaiMoen
I will try to say 'python script' when referring to the scripts that interact with the API, and 'input script' when talking about the input scripts used by TMInterface.

## sd_railgun:
##### Quick setup:
At no point should you need to edit the code itself, if you find an error, just send it in the TM TAS discord and I will look at it.

1. Run the python script while TMInterface is open
2. It should disable bruteforce automatically (with the command 'set controller none')
3. In the TMInterface console, write down a timerange, followed by 'sd'; (60100-61000 sd, or: 1:00.1-1:01.0 sd, for example)
4. If you have a stable sd at the start time (all wheels have full contact), you can use 'sdh' in the console to enable a slightly more accurate evaluation mode.
5. If you want it to properly take over an s4d, you can use 's4d' in the console to enable a mode that automatically full steers until the main algorithm can safely continue with the sd itself.
6. The direction it will try to speedslide in will be based on the average steering value in that timerange.
7. Press validate on the replay in which you would like to improve your speedslide. The python script will take a while to go through all the ticks, you might find yourself waiting up to 20 seconds to do one in-game second.
8. Afterwards, all of the recommended steering inputs will have been printed to the python terminal and also saved to a file named sd_railgun.txt in the same directory as the python script. The inputs in the file do not have duplicates, unlike the terminal which also has a speedometer that blocks you from easily copying over the inputs.
9. (Optional) Deregister the python script and go over the inputs with a built-in or script bruteforcer to be sure that all of the map-specific quirks are ironed out. This will also sort your button presses again, if it finds an improvement at least.

##### Common problems:
1. Using keyboard presses inside the timerange will most likely cause the python script to malfunction in some way or another. Not because that would be hard to implement, but because I don't care. Converting to pad/analog manually or using the TMInterface discord bot is highly recommended.
2. Most variables in the code are pretty optimal for all scenarios, but at 400-480ish kph on road you may find that the angle it holds has an abysmal quality, because full steering to correct itself is too expensive in the short term, so in that case you might want it to understeer at the start a bit to drop it in. Otherwise it should quickly full steer to get to the correct angle.
3. Most obstacles and walls cannot be avoided unless you are barely far enough away in order for the python script to react in time by steering to a very low steer value. You will see this in the console when it starts steering around wildly.
4. This effect can also be seen when gearing on dirt or grass, but you might end up losing speed anyway as, for a split second, it's faster to full steer and avoid the gear. The gear comes through at some point though, and then it will quickly stop steering until there is no more penalty for doing so. It is therefore recommended to bruteforce gear changes on these surfaces as well, in case it was faster to just let the gear go through normally. You can also just not use the script when you gear up.

##### Explanation:
Instead of trying to optimize friction and angles like sd_perfect, it tries to find the angle that will give it the most speed in the near future.
Technically this is just a very short-term speed script using a lot of rewinds to test things in a deterministic way rather than purely bruteforcing.
An sd is never defined in the code. However, after just 10 ticks (100ms), there is already a good measurable difference in speed gain from speedslides depending on steering angle, and 120ms is optimal in general as long as you restrict the permissible steering range mostly to one half of the total range.

The s4d command simply checks on every tick if the sideways sliding speed is below 4 m/s in the direction you're going to speedslide in and fullsteers if this is not the case, otherwise it turns itself off and enables the main algorithm (it also temporarily turns up the delay for picking the best steer to allow for a smooth transition to the sd angle)

The sdh command enables a mode of evaluating the velocity which doesn't take into account the local vertical velocity. Local meaning from the car's perspective. This mode can potentially gain a slight edge over using the total velocity ('Real speed' in the TMInterface info window). This is mostly true when the car has full ground contact as otherwise measuring the speed in this mode can be a bit tricky. Test using triggers or the speedometer in the terminal to see if it's actually faster or not in your case.

The algorithm will be able to hold the most 'perfect' angle down that can be achieved from the simulation state at input time. It does the following:
###### New algorithm and why it's better than the old algorithm:
The script starts out by taking 17 steering value points from 0 to 65536 (multiplied by the sd direction; -1 for left and 1 for right), similarly to the old algorithm. 
Unlike the old algorithm however, it just keeps taking points around the fastest point of that stage, rather than trying to interpolate using the second best point. As it turns out, using the second best point is not very reliable. Let's say for instance that your best steering value so far is 32k, number 2 is 34k and number 3 is 30k. Oftentimes what would happen is that the predicted angle is higher than 32k whereas actually 31999 would be faster than 32001. This would often randomly cause the algorithm to find steering values that correspond to a multiple of the interval being used for the starting points. An example of this that could actually happen is it steering 32800, 32770, 32740, 32768, 32768, 32650; where 32768 is a multiple of 2048, the best interval for the old algorithm. My personal explanation for this is that at the sort of small steering differences, oversteering is faster, but if you zoom very far you will find that understeering slightly would be better, which is why it would take one of the starting points way too often if it interpolated the wrong way. Understanding that the second best steering value is never reliable, the new algorithm takes a range around the best point with decreasing interval for every 'stage'.

The stages are pretty simple:
After our initial 17 starting points with an interval of 4096, divide the interval by 8 and measure 8 points around either side of the best point from the previous stage. After going from 4096 -> 512 -> 64 -> 8 -> 1, we can be fairly certain that we have approximated the best steering value to near perfection.
This change of algorithm gives us a pretty big acceleration boost of 0.014 km/h per second of speeddrift on average.

###### Old algorithm (No longer in use in newer versions of the python script):
Firstly, 33 steering values in the given direction are tested, with an interval of 2048. This covers the entire half of the steering range that we will be focusing on. (that is 0 up to and including 65536 in the set direction)
After that there is an interpolation stage of 11 tests, where we take the best value up to this point and move it towards the second best value in decreasing steering differences. (2048 halved 11 times gives 1)
After we only changed by 1 unit, pick the best value and display it. (And save it unless it is the same as the tick before)

p.s.
The reason I named it railgun is because visualizing the code running through the inputs made me think of how a railgun gradually propels its projectile (in this case the stadium car), though I might just have too much imagination.
