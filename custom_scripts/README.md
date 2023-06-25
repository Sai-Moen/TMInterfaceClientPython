# TMInterface custom scripts by SaiMoen
I will try to say 'python script' when referring to the scripts that interact with the API, and 'input script' when talking about the input scripts used by TMInterface.

## Troubleshooting:
Script doesn't seem to open or "...object doesn't have attribute..." ->
This is a problem with the tminterface API version not matching what the script expects.
Update your tminterface api using `pip install tminterface --upgrade` and:
Make sure your installations of Python and pip are both on the PATH environment variable (python installer advanced option),
Make sure you're running the script from those versions, as pip will install a package for the version on PATH.

## sd_railgun:
Deterministic, velocity-based procedural speedslide/speeddrift script

#### Quick setup:
At no point should you need to edit the code itself, if you find an error, just send it in the TM TAS discord and I will look at it.

1. Run the python script while TMInterface is open, it should disable bruteforce automatically (with the command 'set controller none').
2. In the TMInterface console, write down a timerange, followed by 'sd' and the direction; (13370-69420 sd left, or: 1:23.45-1:34.56 sd right, for example).
3. (Optional) Use the seek command to change the ticks that are taken into account when determining the best steering value (default is 12 ticks)
4. Press validate on the replay in which you would like to improve your speedslide. The python script will take a while to go through all the ticks, you might find yourself waiting up to 20 seconds to do one in-game second.
5. Afterwards, all of the recommended steering inputs will have been printed to the python terminal and also saved to a file named sd_railgun.txt in the directory you launched the script from. The inputs in the file do not have duplicates, unlike the terminal, which also has a speedometer that blocks you from easily copying over the inputs.
6. (Optional) Deregister the python script and go over the inputs with a built-in or script bruteforcer to be sure that all of the map-specific quirks are ironed out. This will also sort your button presses again, if it finds an improvement at least.

##### Common problems:
1. Be wary when using it below 500 speed! At these speeds there is an alternative line that it can also latch on to. This can be seen in the first sd of the trabadia 23.68 A01 TAS. Personally, I call this the eco-sd, and it makes it so that you gain much more overlap than expected. This is faster in that TAS because they exit out of it very soon, which makes it so there's much less friction in an sd that doesn't get a lot more speed than usual anyway. If you exit an sd above about 450 speed then it's a bit dubious, and you should most likely be going for the less overlap sd instead (As seen in the second sd of that TAS).
2. Using keyboard presses inside the timerange will most likely cause the python script to malfunction in some way or another. Not because that would be hard to implement, but because I don't see why you would use keyboard to sd in TAS and the inputs that you start out with in the timerange don't really matter anyway. Converting to pad/analog manually or using the TMInterface discord bot is highly recommended.
3. Most obstacles and walls cannot be avoided unless you are barely far enough away in order for the python script to react in time by steering to a very low steer value. You will see this in the console when it starts steering around wildly. This effect can also be seen when gearing on dirt or grass, but you might end up losing speed anyway as, for a split second, it's faster to full steer and avoid the gear. The gear comes through at some point though, and then it will quickly stop steering until there is no more penalty for doing so. It is therefore recommended to bruteforce gear changes on these surfaces as well, in case it was faster to just let the gear go through normally. You can also just avoid using the script when you gear up.
4. Dirt... At some early point in the script's lifespan it worked fine without assistance, but when switching to a different algorithm to get faster road sd's it stopped working without outside interference. Grass does not have the same issue and it will automatically latch on to the given direction.

##### Explanation:
Instead of trying to optimize friction and angles like sd_perfect, it tries to find the angle that will give it the most speed in the near future.
Technically this is just a very short-term speed script using a lot of rewinds to test things in a deterministic way rather than purely bruteforcing.
An sd is never defined in the code. However, after just 10 ticks (100ms), there is already a good measurable difference in speed gain from speedslides depending on steering angle, and 120ms is optimal in general. At different speeds and sd qualities this number may be different but 120 performs very well overall.

The seek command controls a variable that determines how many ticks the steering values should be held when testing the speeds they get. Lowering this value will result in a slightly more accurate but less stable evaluation and vice versa. The default is 12 ticks as it's a nice balance for a great amount of common speeddrift situations, but for sd's that are already very good or happen at higher speeds you can experiment with lowering the value. With sd's at the edge of 400 speed you could for instance try 13 or 14 ticks instead to get the sd to latch on properly.

###### New algorithm and why it's better than the old algorithm:
The script starts out by taking 4 steering value points from 4096 to 61440 (multiplied by the sd direction; -1 for left and 1 for right), similarly to the old algorithm, but with more efficient steps of 16384. 
Unlike the old algorithm however, it just keeps taking points around the fastest point of the previous pass, rather than trying to interpolate using the second best point. As it turns out, using the second best point is not very reliable.

Let's say for instance that your best steering value so far is 32k, number 2 is 34k and number 3 is 30k. Oftentimes what would happen is that the predicted angle is higher than 32k whereas actually 31999 would be faster than 32001. This would often randomly cause the algorithm to find steering values that correspond to a multiple of the interval being used for the starting points.

An example of this that could actually happen is it steering 32800, 32770, 32740, 32768, 32768, 32650; where 32768 is a multiple of 2048, the best interval for the old algorithm. My personal explanation for this is that at the sort of small steering differences, oversteering is faster, but if you zoom very far you will find that understeering slightly would be better, which is why it would take one of the starting points way too often if it interpolated the wrong way. Understanding that the second best steering value is never reliable, the new algorithm takes a range around the best point with decreasing interval for every 'stage'.

So by instead doing many passes with points around the best steering value we can slowly decrease the width of the search window, while still allowing for the window itself to move around as our guesses get closer and closer.

###### Old algorithm (No longer in use in newer versions of the python script):
Firstly, 33 steering values in the given direction are tested, with an interval of 2048. This covers the entire half of the steering range that we will be focusing on. (that is 0 up to and including 65536 in the set direction)
After that there is an interpolation stage of 11 tests, where we take the best value up to this point and move it towards the second best value in decreasing steering differences. (2048 halved 11 times gives 1)
After we only changed by 1 unit, pick the best value and display it. (And save it unless it is the same as the tick before)

##### p.s.
The reason I named it railgun is because visualizing the code running through the inputs made me think of how a railgun gradually propels its projectile (in this case the stadium car), though I might just have too much imagination.

## Wallhugger
A script that loves full steering, except when there is a wall.

#### Quick setup:
At no point should you need to edit the code itself, if you find an error, just send it in the TM TAS discord and I will look at it.

1. Run the python script while TMInterface is open, it should disable bruteforce automatically (with the command 'set controller none').
2. In the TMInterface console, write down a timerange, followed by 'wh' and the direction; (13370-69420 wh left, or: 1:23.45-1:34.56 wh right, for example).
3. Press validate on the replay in which you would like to improve your wallhug. The python script will take a while to go through all the ticks, you might find yourself doing only a few ticks per second if the script is really searching.
4. Afterwards, all of the recommended steering inputs will have been printed to the python terminal and also saved to a file named wallhugger.txt in the same directory as the python script. The inputs in the file do not have duplicates, unlike the terminal, which also has a speedometer that blocks you from easily copying over the inputs.
5. (Optional) Deregister the python script and go over the inputs with a built-in or script bruteforcer to be sure that all of the map-specific quirks are ironed out. This will also sort your button presses again, if it finds an improvement at least.

##### Explanation:
This script works largely the same as sd_railgun, but instead it tries to maximize the steering as much as possible, as long as the steering value doesn't make it crash or lose too much speed. The seeking timespan is 400ms instead of 120ms, because it's a bit tricky to get right without seeing further ahead if there are any walls, whereas with sd's you don't ever expect to collide.
The entry to a wallhug still has to be done manually, and starting the script on different ticks can have very different results.
