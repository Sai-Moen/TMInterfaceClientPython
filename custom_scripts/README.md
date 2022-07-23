# TMInterface custom scripts by SaiMoen
I will try to say 'python script' when referring to the scripts that interact with the API, and 'input script' when talking about the input scripts used by TMInterface.

## sd_railgun:
Deterministic, velocity-based procedural speedslide/speeddrift script

##### Quick setup:
At no point should you need to edit the code itself, if you find an error, just send it in the TM TAS discord and I will look at it.

1. Run the python script while TMInterface is open, it should disable bruteforce automatically (with the command 'set controller none').
2. In the TMInterface console, write down a timerange, followed by 'sd' and the direction; (13370-69420 sd left, or: 1:23.45-1:34.56 sd right, for example).
3. Use the sdmode command to use s4d or wiggle modes. The sdcounter command will toggle countersteering (True by default).
4. Press validate on the replay in which you would like to improve your speedslide. The python script will take a while to go through all the ticks, you might find yourself waiting up to 20 seconds to do one in-game second.
5. Afterwards, all of the recommended steering inputs will have been printed to the python terminal and also saved to a file named sd_railgun.txt in the same directory as the python script. The inputs in the file do not have duplicates, unlike the terminal, which also has a speedometer that blocks you from easily copying over the inputs.
6. (Optional) Deregister the python script and go over the inputs with a built-in or script bruteforcer to be sure that all of the map-specific quirks are ironed out. This will also sort your button presses again, if it finds an improvement at least.

##### Common problems:
1. Using keyboard presses inside the timerange will most likely cause the python script to malfunction in some way or another. Not because that would be hard to implement, but because I don't see why you would use keyboard to sd in TAS and the inputs that you start out with in the timerange don't really matter anyway. Converting to pad/analog manually or using the TMInterface discord bot is highly recommended.
2. Most variables in the code are pretty optimal for most scenarios, but below 500-ish kph on road you may find that the angle it holds has an abysmal quality, because full steering to correct itself is too demanding in the short term, so in that case you might want it to understeer at the start a bit to drop it in. Otherwise it should quickly full steer to get to the correct angle. You should check the quality if there is any uncertainty about it.
3. Most obstacles and walls cannot be avoided unless you are barely far enough away in order for the python script to react in time by steering to a very low steer value. You will see this in the console when it starts steering around wildly. This effect can also be seen when gearing on dirt or grass, but you might end up losing speed anyway as, for a split second, it's faster to full steer and avoid the gear. The gear comes through at some point though, and then it will quickly stop steering until there is no more penalty for doing so. It is therefore recommended to bruteforce gear changes on these surfaces as well, in case it was faster to just let the gear go through normally. You can also just avoid using the script when you gear up.
4. Dirt... At some early point in the script's lifespan it worked fine, but when switching to a different algorithm to get faster road sd's it stopped working without outside interference. Grass does not have the same issue and it works fine there.

##### Explanation:
Instead of trying to optimize friction and angles like sd_perfect, it tries to find the angle that will give it the most speed in the near future.
Technically this is just a very short-term speed script using a lot of rewinds to test things in a deterministic way rather than purely bruteforcing.
An sd is never defined in the code. However, after just 10 ticks (100ms), there is already a good measurable difference in speed gain from speedslides depending on steering angle, and 120ms is optimal in general. At different speeds and sd qualities this number may be different but 120 performs very well overall.

The sdmode command allows the user to select different modes than 'normal'. These modes are s4d and wiggle. Wiggle mode does sd's in an alternating direction and switches the direction every .3 seconds, unless a gear is taking place. s4d mode full steers at the start of the script as long as a certain sideways speed goal has not been met. After that the main algorithm takes over.

The sdcounter command simply toggles a mode where countersteering is/isn't allowed. This could be useful on dirt if you want to increase the chance of starting an sd automatically. This is more a bandaid solution until a more permanent fix for dirt is found.

###### New algorithm and why it's better than the old algorithm:
The script starts out by taking 17 steering value points from 0 to 65536 (multiplied by the sd direction; -1 for left and 1 for right), similarly to the old algorithm. 
Unlike the old algorithm however, it just keeps taking points around the fastest point of that stage, rather than trying to interpolate using the second best point. As it turns out, using the second best point is not very reliable.

Let's say for instance that your best steering value so far is 32k, number 2 is 34k and number 3 is 30k. Oftentimes what would happen is that the predicted angle is higher than 32k whereas actually 31999 would be faster than 32001. This would often randomly cause the algorithm to find steering values that correspond to a multiple of the interval being used for the starting points.

An example of this that could actually happen is it steering 32800, 32770, 32740, 32768, 32768, 32650; where 32768 is a multiple of 2048, the best interval for the old algorithm. My personal explanation for this is that at the sort of small steering differences, oversteering is faster, but if you zoom very far you will find that understeering slightly would be better, which is why it would take one of the starting points way too often if it interpolated the wrong way. Understanding that the second best steering value is never reliable, the new algorithm takes a range around the best point with decreasing interval for every 'stage'.

The stages are pretty simple:
After our initial 17 starting points with an interval of 4096, divide the interval by 8 and measure 8 points around either side of the best point from the previous stage. After going from 4096 -> 512 -> 64 -> 8 -> 1, we can be fairly certain that we have approximated the best steering value to near perfection.
This change of algorithm gives us a pretty big acceleration boost of 0.014 km/h per second of speeddrift on average.

###### Old algorithm (No longer in use in newer versions of the python script):
Firstly, 33 steering values in the given direction are tested, with an interval of 2048. This covers the entire half of the steering range that we will be focusing on. (that is 0 up to and including 65536 in the set direction)
After that there is an interpolation stage of 11 tests, where we take the best value up to this point and move it towards the second best value in decreasing steering differences. (2048 halved 11 times gives 1)
After we only changed by 1 unit, pick the best value and display it. (And save it unless it is the same as the tick before)

p.s.
The reason I named it railgun is because visualizing the code running through the inputs made me think of how a railgun gradually propels its projectile (in this case the stadium car), though I might just have too much imagination.

## Wallhugger
A script that loves full steering, except when there is a wall.

##### Quick setup:
At no point should you need to edit the code itself, if you find an error, just send it in the TM TAS discord and I will look at it.

1. Run the python script while TMInterface is open, it should disable bruteforce automatically (with the command 'set controller none').
2. In the TMInterface console, write down a timerange, followed by 'wallhug' and the direction; (13370-69420 wallhug left, or: 1:23.45-1:34.56 wallhug right, for example).
3. Press validate on the replay in which you would like to improve your wallhug. The python script will take a while to go through all the ticks, you might find yourself doing only a few ticks per second if the script is really searching.
4. Afterwards, all of the recommended steering inputs will have been printed to the python terminal and also saved to a file named wallhugger.txt in the same directory as the python script. The inputs in the file do not have duplicates, unlike the terminal, which also has a speedometer that blocks you from easily copying over the inputs.
5. (Optional) Deregister the python script and go over the inputs with a built-in or script bruteforcer to be sure that all of the map-specific quirks are ironed out. This will also sort your button presses again, if it finds an improvement at least.

##### Common problems:
The script starts steering 0 if there is no way to dodge a wall from the given game state. Different starting times can drastically alter how it will behave, so try that for a bit. Another useful way to determine the starting time would be looking where you could start fullsteering into the wall you want to hug without crashing within 1-1.5 seconds, and then starting the script one or two ticks before that time. Starting the script very far away from the wall will probably just result in a bonk.

##### Explanation:
It's very similar to sd_railgun in that it slowly interpolates the steer to tune for a specific goal. In this case the goal is to turn the yaw as much as possible in the chosen direction, without slowing down too much. In this case it re-uses the old sd algorithm because touching the wall is quite noticable, and it can simply try adding a decreasing amount to the highest non-crashing steering value every iteration.

The seeking range is 600ms as of the first version, which makes the script considerably slower than the sd script, while taking 4 times as few steering tests. If a better way of figuring out the tightest steering value is found, this would also speed up the script most likely.
