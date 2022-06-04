# TMInterface custom scripts by SaiMoen

I will try to say 'script' when referring to the scripts that interact with the API, and 'TAS script' when talking about the input scripts used by TMInterface.

## sd_railgun:

If I start rambling about countersteering... that really means no steering in this script.

##### Quick setup:
1. Run the script while TMInterface is open
2. It should disable bruteforce automatically (with the command 'set controller none')
3. Write down a timerange, followed by 'sd'; (100-1000 sd, or: 0.1-1.0 sd, for example)
4. The direction it will try to speedslide will be based on the average steering value in that timerange (just so you know)
5. Press validate on the replay in which you would like to improve your speedslide, within the given timerange.
6. The script will take a while to go through all the ticks, depending on your system you might find yourself waiting several seconds to do one in-game second.
7. Afterwards, all of the recommended steering inputs will have been printed to console and also saved to a file named sd_railgun.txt in the same directory as the script.
8. If you happen to have any button presses in the timerange, be aware that they have been executed in the simulation, but not saved. One way to deal with this is to simply paste the inputs below the existing TAS script and then go from there, since they don't necessarily have to be in ascending order for TMInterface to load the TAS script.
9. (Optional) Deregister the script and go over the inputs with a built-in or script bruteforcer to be sure that all of the map-specific quirks are ironed out. This will also sort your button presses again, if it finds an improvement at least.

##### Common problems:
1. Using keyboard presses inside the timerange will most likely cause the script to malfunction in some way or another. Converting to pad/analog manually or using the TMInterface discord bot is highly recommended.
2. If countersteering takes place at something like an s4d, you could try to raise the seeking distance, but it's already pretty optimized for most situations so probably try some different/later timeranges instead. The first tick it finds should be slighly understeered since then it will decrease to the best angle, whereas it can't always find the correct angle if it tries to increase the steering by itself. However, this happens mostly at 400-500 speed on road and even then it might be able to full steer to correct itself. Otherwise it doesn't really happen, but do pay attention to that.
3. Most obstacles and walls cannot be avoided unless you are barely far enough away in order for the script to react in time by steering to a very low steer value. You will see this in the console when it starts steering around wildly.
4. This effect can also be seen when gearing on dirt or grass, but you might end up losing speed anyway as, for a split second, it's faster to full steer and avoid the gear. The gear comes through at some point though, and then it will quickly stop steering until there is no more penalty for doing so. It is therefore recommended to bruteforce gear changes on these surfaces as well, in case it was faster to just let the gear go through normally.

##### Explanation:
Instead of trying to optimize friction and angles like sd_perfect, it tries to find the angle that will give it the most speed in the near future.
Technically this is just a very short-term speed script using a lot of rewinds to test things in a deterministic way instead of bruteforce.
An sd is never defined in the code. However, after just 10 ticks (100ms), there is already a good measurable difference in speed gain from speedslides depending on steering angle. But 12 ticks seems to be the fastest and most consistent delay.
The algorithm will be able to hold the most 'perfect' angle down that can be achieved from the simulation state at input time. It does the following:

Firstly, 33 steering values in the given direction are tested, with an interval of 2048. This covers the entire half of the steering range that we will be focusing on. (that is 0 up to and including 65536 in the set direction)
After that there is an interpolation stage of 11 tests, where we take the best value up to this point and move it towards the second best value in decreasing steering differences. (1024 halved 10 times gives 1)
After we only changed by 1 unit, pick the best value and display (and eventually save) it. (unless it is the same as last time)
The tests themselves are quite simple; take the vector norm of the velocity vector (3d speed basically) after a certain amount of time, and add it to a list along with its steering value.

I experimented with the time we should take as well as the amount of starting tests. I got to 120ms for the amount of time since it gives good speed everywhere and otherwise the script doesn't start a grass sd as soon as it should. Going lower or higher does not often result in more speed and can cause consistency issues, especially if you're barely above 'sd viable' speeds. On higher timespans the steering value becomes more and more disconnected with what would be fastest at the time of inputting. Going lower will often reward countersteering too much which would lose speed or even the slide in general. As for the starting tests, the difference between 4096 and 2048 intervals is still measurable, but 2048 and 1024 often cause sd's within thousandths of a km/h, even after several seconds, and 2048 often seems the most robust.

The reason I named it railgun is because visualizing the code running through the inputs made me think of how a railgun gradually propels its projectile (in this case the stadium car), though I might just have too much imagination.
