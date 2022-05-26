# sd_railgun by SaiMoen:

Instead of trying to optimize friction and angles like sd_perfect, it tries to find the angle that will give it the most speed in the near future.
Technically this is just a very short-term speed script with multiple iterations.
An sd is never defined in the code. However, after just 10 ticks (100ms), there is already a good measurable difference in speed gain from speedslides depending on steering angle. (Though a little bit of buffer is used for consistency)
The algorithm will be able to hold the 'perfect' angle down with a precision of a couple hundred steering units, worst case a couple thousand if there is a lot of suspension action going on.
That is assuming you're on a level surface, although it should keep working when going up a ramp.

My personal recommendation would be to get a decent sd going, then use this script to get it near-perfect, and then bruteforce it with the built-in velocity bruteforcer or longer term speed scripts just in case.
But do be wary that this script will not try to get out of the way of obstacles until it is too late, also if your intial sd is oversteered you might find yourself bonking into walls you didn't use to bonk into.
But that's the kind of tweaking that we TASers needed to do anyway, so good luck!

The reason I named it railgun is because visualizing the code running through the inputs made me think of how a railgun gradually propels its projectile (in this case the stadium car), though I might just have too much imagination.
