

A function called isDeadEnd is implemented, it returns a boolean and an int indicating the depth.
Right now it is disabled, because I don't know how it should work with the rest of the cost/heuristic function to get the best result.
It can be turned on by uncommenting the 3 lines in aStarSearch, then the pacman will not go into any deadend even if there is a food in there. Try it with deadEndCapture, make the blue team baselineTeam.
There is still the problem that this function does not take into consideration of the dead ends that turns to a different direction... I am still trying to fix that.
