# myTeam.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to UC Berkeley, including a link to http://ai.berkeley.edu.
# 
# Attribution Information: The Pacman AI projects were developed at UC Berkeley.
# The core projects and autograders were primarily created by John DeNero
# (denero@cs.berkeley.edu) and Dan Klein (klein@cs.berkeley.edu).
# Student side autograding was added by Brad Miller, Nick Hay, and
# Pieter Abbeel (pabbeel@cs.berkeley.edu).


from captureAgents import CaptureAgent
from util import PriorityQueue
import random, time, util
from game import Directions
import game

#################
# Team creation #
#################

def createTeam(firstIndex, secondIndex, isRed,
               first = 'SwitchAgent', second = 'SwitchAgent'):
  """
  This function should return a list of two agents that will form the
  team, initialized using firstIndex and secondIndex as their agent
  index numbers.  isRed is True if the red team is being created, and
  will be False if the blue team is being created.
  As a potentially helpful development aid, this function can take
  additional string-valued keyword arguments ("first" and "second" are
  such arguments in the case of this function), which will come from
  the --redOpts and --blueOpts command-line arguments to capture.py.
  For the nightly contest, however, your team will be created without
  any extra arguments, so you should make sure that the default
  behavior is what you want for the nightly contest.
  """
  if isRed:
  	return [eval(first)(firstIndex), eval(second)(secondIndex)]
  else:
  	return [eval('ThiefAgent')(firstIndex), eval('DefenseAgent')(secondIndex)]

##########
# Agents #
##########

class DummyAgent(CaptureAgent):
  """
  A Dummy agent to serve as an example of the necessary agent structure.
  You should look at baselineTeam.py for more details about how to
  create an agent as this is the bare minimum.
  """

  def registerInitialState(self, gameState):
    """
    This method handles the initial setup of the
    agent to populate useful fields (such as what team
    we're on).
    A distanceCalculator instance caches the maze distances
    between each pair of positions, so your agents can use:
    self.distancer.getDistance(p1, p2)
    IMPORTANT: This method may run for at most 15 seconds.
    """

    '''
    Make sure you do not delete the following line. If you would like to
    use Manhattan distances instead of maze distances in order to save
    on initialization time, please take a look at
    CaptureAgent.registerInitialState in captureAgents.py.
    '''
    CaptureAgent.registerInitialState(self, gameState)

    '''
    Your initialization code goes here, if you need any.
    '''


  def chooseAction(self, gameState):
    """
    Picks among actions randomly.
    """
    actions = gameState.getLegalActions(self.index)

    '''
    You should change this in your own agent.
    '''

    return random.choice(actions)

#Base class that implements some useful functions
class SmartAgent(CaptureAgent):
	actions = {"Stop": (0,0), "North": (0,1), "East": (1,0), "South": (0,-1), "West": (-1,0)}		#All the actions a Pacman can take and how it moves the Pacman

	#This function initializes some variables and calculates some values that will be useful later. The specifics of each variable are commented below.
	def registerInitialState(self, gameState):
		CaptureAgent.registerInitialState(self, gameState)

		self.teamIndices = CaptureAgent.getTeam(self, gameState)	#Array representing the indices of every ally Agent
		self.opponentIndices = CaptureAgent.getOpponents(self, gameState)	#Array representing the indices of every oponent Agent

		if self.index == self.teamIndices[0]:	#Determine which index information about this Agent can be found. That includes who it's going after (if it's on defense) and where it is
			self.teamOrder = 0
		elif self.index == self.teamIndices[1]:
			self.teamOrder = 1

		position = gameState.getAgentPosition(self.index)		#This figures out where the boundary between either side is
		gameMap = gameState.getWalls()
		boundary = gameMap.width / 2
		if position[0] < boundary:
			self.leftEdge = 0	#The farthest left the Agent can go and still be on it's home side
			self.rightEdge = boundary	#The farthest left the Agent can go and be on the opponent side
			self.boundary = boundary - 1	#The place on the home side closest to the far side
		else:
			self.leftEdge = boundary	#All these variables are the same as above.
			self.rightEdge = gameMap.width
			self.boundary = boundary

		

	#Gets information that is useful for determining an action
	def actionPrep(self, gameState):
		self.enemyPos = (gameState.getAgentPosition(self.opponentIndices[0]), gameState.getAgentPosition(self.opponentIndices[1]))		#Get the position of each enemy
		self.enemyDist = [util.manhattanDistance(gameState.getAgentPosition(self.index), self.enemyPos[i]) for i in range(2)]		#Get the manhattan distance to each enemy

	#Standard A* search. Literally copied from project 1
	def aStarSearch(self, gameState, isGoal, heuristic, costFunction):
	    """Search the node that has the lowest combined cost and heuristic first."""
	    visited = []    #Set up the initial state for A*
	    path = {}
	    queue = PriorityQueue()
	    position = gameState.getAgentPosition(self.index)
	    queue.push([position, 0], 0)
	    foodList = self.getFood(gameState).asList()
	    walls = gameState.getWalls().asList()
	    
	    while not queue.isEmpty():  #Continue looking until we have found the goal or tried looking everywhere
	        curPosition = queue.pop()
	        if isGoal(curPosition[0], gameState): #If the current state is the goal state, we can exit and move on to generating the path
	            break
	            
	        if curPosition[0] not in visited:
	            visited.append(curPosition[0])    #If we haven't visited it, mark it as visited
	            
	            successors = self.getSuccessors(curPosition[0], walls, costFunction, gameState)
	            for node in successors:
	                if node[0] not in visited:
	                    if (node[0] not in path) or (path[node[0]][2] > (curPosition[1] + node[2])):
	                        path[node[0]] = [curPosition[0], node[1], curPosition[1] + node[2]] #Mark down which state we were in before reaching this one and which action was taken to reach it
	                    queue.push([node[0], curPosition[1] + node[2]], curPosition[1] + node[2] + heuristic(node[0], gameState, position))
	            
	    ans = []
	    curPosition = curPosition[0]
	    goal = curPosition
	    cost = 0
	    if curPosition != position:
			cost = path[curPosition][2]

	    while curPosition != position:  #Until we reach the starting state, push actions into the beginning of the answer
	        ans.insert(0, path[curPosition][1])
	        curPosition = path[curPosition][0]

	    return ans, goal

	#Determine if the Pacman in on it's home side or on the enemies side
	def isHome(self, position):
		return position[0] >= self.leftEdge and position[0] < self.rightEdge

	#Find the distance to the boundary between sides
	def distFromHome(self, position):
		if self.leftEdge == 0:	
			boundary = self.rightEdge - 1
		else:
			boundary = self.leftEdge
		dist = position[0] - boundary

		return dist if dist > 0 else dist * -1


	#determine whether a location has a power pellet
	def isPowerPellet(self, position, gameState):
		return position in self.getCapsules(gameState)

	#Determines whether a pellet is on the given location
	def isPellet(self, position, gameState):
		return position in self.getFood(gameState).asList()

	#Determine whether or not the given position is a dead end
	def isDeadEnd(self, position, deadEndMap):
		return position in deadEndMap.asList()

	#Find all the pellets that aren't too deep we can't get them and leave before a defender cuts us off
	def getFeasiblePellets(self, gameState, distance):
		pellets = self.getFood(gameState).asList()
		copy = self.getFood(gameState).asList()	#Allows us to remove pellets without messing up the for loop
		for pellet in pellets:
			if self.findDepth(pellet, self.offenseCost, gameState) * 2 + 1 > self.closeMazeEnemy:	#If the pellet is too deep, remove it from the list
				copy.remove(pellet)

		return copy


	#Find the depth of a position if it is a dead end
	def findDepth(self, position, costFunction, gameState):
		curState = self.breadthFirstSearch(position, costFunction, gameState)
		
		return self.distancer.getDistance(position, curState)

	#Standard BFS. Used to find the depth of a dead end
	def breadthFirstSearch(self, position, costFunction, gameState):
	    #"""Search the shallowest nodes in the search tree first."""
	    
	    visited = []    #Set up the initial state for bfs
	    path = {}
	    queue = util.Queue()
	    state = position
	    queue.push(state)
	    visited.append(state)

	    walls = gameState.getWalls().asList()

	    
	    while not queue.isEmpty():  #Continue looking until we have found the goal or tried looking everywhere
	        curState = queue.pop()

	        x = curState[0]
	        y = curState[1]

	        if (curState != position and not self.deadEndMap[x][y]):
	         	break;
	        
	        successors = self.getSuccessors(curState, walls, costFunction, gameState)
	        for node in successors:
	            if node[0] not in visited:
	                path[node[0]] = [curState, node[1]] #Mark down which state we were in before reaching this one and which action was taken to reach it
	                queue.push(node[0])
	                visited.append(node[0])    #If we haven't visited it, mark it as visited


	    return curState

	#Creates a map of all the dead ends in the board
	def createDeadEndMap(self, gameState):
		walls = gameState.getWalls()
		deadEnds = walls.copy()

		done = False

		while (not done):	#Loop through until we've found all the dead ends
			done = True

			# iterate every location
			for x in range(walls.width):
				for y in range(walls.height):

					if (not deadEnds[x][y]):	#If the position isn't a dead end, check if it should be
						isDeadEnd1 = deadEnds[x-1][y]
						isDeadEnd2 = deadEnds[x][y-1]
						isDeadEnd3 = deadEnds[x+1][y]
						isDeadEnd4 = deadEnds[x][y+1]

						# if 3 sides are dead ends, make it a dead end
						if ((isDeadEnd1 and isDeadEnd2 and isDeadEnd3) or (isDeadEnd1 and isDeadEnd2 and isDeadEnd4) or (isDeadEnd1 and isDeadEnd3 and isDeadEnd4) or (isDeadEnd2 and isDeadEnd3 and isDeadEnd4)): 
							deadEnds[x][y] = True
							done = False

		return deadEnds

	#Gets the manhattan distance to the closest enemy
	def enemyDistance(self, position, gameState):
		return min(util.manhattanDistance(position, self.enemyPos[i]) for i in range(2))

	#Get all the legal positions the Pacman can be after one move and the action to get there
	def getSuccessors(self, position, walls, costFunction, gameState):
		nextStates = []
		for action in self.actions:		#Loop through all legal actions
			nextPos = (position[0] + self.actions[action][0], position[1] + self.actions[action][1])		#Find out where the Pacman would be
			if nextPos not in walls:		#Make sure the Pacman wouldn't move into a wall
				nextStates.append((nextPos, action, costFunction(position, gameState)))
		return nextStates

	
class SwitchAgent(SmartAgent):
	onOffense = [True, False]	#Array for determining who is on offense and who is on defense
	deadEndMap = None	#A map of all the dead ends in the map
	marked = [None, None]	#Who the defender with the corresponding teamOrder is going after

	#Calculate all the values that will be used later on
	def registerInitialState(self, gameState):
		SmartAgent.registerInitialState(self, gameState)
		self.hasPellet = False		#Boolean to indicate whether or not we're carrying a pellet
		self.start = gameState.getAgentPosition(self.index)		#The position we started in, used to determine if we were killed

		if SwitchAgent.deadEndMap == None:	#If the dead end map hasn't been created, create it
			SwitchAgent.deadEndMap = self.createDeadEndMap(gameState)
		self.goal = None	#The goal we're working towards. Can be used to avoid constantly readjusting path

	#Function for determining who should be on offense and who's on defense
	def setOffenseDefense(self, gameState):
		myState = gameState.getAgentState(self.index)	#The state of the agent
		pos1 = gameState.getAgentPosition(self.teamIndices[0])	#The position of the first agent in this team
		pos2 = gameState.getAgentPosition(self.teamIndices[1])	#The position of the second agent in this team
		enemyStates = [gameState.getAgentState(self.opponentIndices[i]) for i in range(2)]	#The states of both enemy teams

		if myState.scaredTimer > 0:		#If we're scared, there is no point playing defense
			SwitchAgent.onOffense = [True, True]	#So assign both Pacmans to offense
			SwitchAgent.marked = [None, None]	#Because they're on offense, noone is marked
		elif enemyStates[0].numCarrying > 5 or enemyStates[1].numCarrying > 5:	#If an enemy is carrying more than 5 pellets, they are a significant threat
			SwitchAgent.onOffense = [False, False]	#So make both agents target them
		elif SwitchAgent.onOffense[0] == SwitchAgent.onOffense[1]:	#If both agents are on either offense or defense, we need to evaluate whether or not it should stay that way
			if self.getScore(gameState) <= 0 or len(self.feasiblePellets) > 0:	#If we're not winning or there are feasible pellets to get, someone needs to go on offense while the other stays on defense
				if self.isHome(pos1) == self.isHome(pos2):	#If both agents are on the same side, determine who should be on offense based on the distance to the closest enemy
					mazeDists = [min(self.distancer.getDistance(pos1, self.enemyPos[i]) for i in range(2)), min(self.distancer.getDistance(pos2, self.enemyPos[i]) for i in range(2))]	#Get the distance to the closest enemy for each agent
					if mazeDists[0] > mazeDists[1]:	#If the first agent is closer to an enemy than the second,
						SwitchAgent.onOffense = [True, False]	#The first agent goes on offense
						SwitchAgent.marked[0] = None	#And it is no longer marking an enemy
					else:	#If the first agent isn't farther from an enemy, we do the same thing, but for the second agent
						SwitchAgent.onOffense = [False, True]
						SwitchAgent.marked[1] = None
				elif self.isHome(pos1):	#If one agent is on the far side while the other is home, that agent goes on offense
					SwitchAgent.onOffense = [False, True]
					SwitchAgent.marked[1] = None
				elif self.isHome(pos2):
					SwitchAgent.onOffense = [True, False]
					SwitchAgent.marked[0] = None
			else:	#If we're winning and there are no feasible pellets, just play defense
				SwitchAgent.onOffense = [False, False]
		elif len(self.feasiblePellets) == 0 and SwitchAgent.onOffense[self.teamOrder] and self.getScore(gameState) > 0:	#Same as above, but make sure the agent on offense if evaluating whether or not there are any feasible pellets
			SwitchAgent.onOffense = [False, False]

		#If none of the above criteria is met, keep the same offense defense arrangement from before.

	#Choose the action that the Agent should perform
	def chooseAction(self, gameState):
		self.actionPrep(gameState)	#Prepare the necessary values

		position = gameState.getAgentPosition(self.index)	#Get the current position of the agent
		if self.isHome(position):	#If it's home, it has returned all it's pellets, so it no longer has any
			self.hasPellet = False
		if position == self.goal:	#If we've reached a goal, we are free to choose a new one
			self.goal = None
		self.enemyMazeDists = [self.distancer.getDistance(position, self.enemyPos[i]) for i in range(2)]	#Get the real distance to each enemy
		self.closeMazeEnemy = min(self.enemyMazeDists)	#Get the distance to the closer of the two

		self.feasiblePellets = self.getFeasiblePellets(gameState, self.closeMazeEnemy)	#Find all the pellets that can be retrieved without going too deep into a dead end
		self.setOffenseDefense(gameState)	#Set the offense defense arrangement for this turn

		if self.onOffense[self.teamOrder]:	#If this agent is on offense, use the appropriate algorithms
			actions, goal = self.aStarSearch(gameState, self.offenseGoal, self.offenseHeuristic, self.offenseCost)
		else:	#Otherwise use defensive algorithms
			if SwitchAgent.onOffense == [False, False] and (self.isHome(self.enemyPos[0]) != self.isHome(self.enemyPos[1])):	#If both agents are on defense and exactly one enemy is home, try to cut off that enemy
				actions, goal = self.aStarSearch(gameState, self.cutoffGoal, self.defenseHeuristic, self.cutoffCost)
			else:	#Otherwise, use the normal defensive algorithms
				actions, goal = self.aStarSearch(gameState, self.defenseGoal, self.defenseHeuristic, self.defenseCost)

			if self.enemyPos[0] == goal:	#Determine which enemy this agent is targeting and mark it
				self.marked[self.teamOrder] = 0
			elif self.enemyPos[1] == goal:
				self.marked[self.teamOrder] = 1
			elif util.manhattanDistance(goal, self.enemyPos[0]) < util.manhattanDistance(goal, self.enemyPos[1]):
				self.marked[self.teamOrder] = 0
			else:
				self.marked[self.teamOrder] = 1

		if len(actions) == 0:	#If there are no actions to reach the goal state, we are already in the goal state
			return "Stop"	#So don't move

		action = actions[0]

		if self.isPellet((position[0] + self.actions[action][0], position[1] + self.actions[action][1]), gameState):	#If our next move takes us into a pellet, we are now carrying at least one
			self.hasPellet = True

		return action


	#Determines if an offensive agent should be working towards this position
	def offenseGoal(self, position, gameState):
		# closeEnemy = manhattan distance of the closer enemy 
		# if self has pellet 
		# if self is home and enemy distance 2: is goal
		# if self is pellet and enemy distance 2: is goal
		# if self no pellet
		# goal state is a pellet 

		"""if self.hasPellet:
			self.goal = None
			return self.isHome(position) or position in self.feasiblePellets
		elif len(self.feasiblePellets) > 0:
			self.goal = None
			return position in self.feasiblePellets
		else:
			start = gameState.getAgentPosition(self.index)
			if self.isHome(start):
				if self.goal == None:
					gameMap = gameState.getWalls()
					option1 = gameMap.height * 2 / 3
					option2 = gameMap.height / 3
					gameMap = gameMap.asList()
					dist = 0
					while True:
						if (self.boundary, option1 + dist) not in gameMap:
							option1 = (self.boundary, option1 + dist)
							break
						elif (self.boundary, option1 - dist) not in gameMap:
							option1 = (self.boundary, option1 - dist)
							break
						else:
							dist += 1

					while True:
						if (self.boundary, option2 + dist) not in gameMap:
							option2 = (self.boundary, option2 + dist)
							break
						elif (self.boundary, option2 - dist) not in gameMap:
							option2 = (self.boundary, option2 - dist)
							break
						else:
							dist += 1

					if util.manhattanDistance(start, option1) > util.manhattanDistance(start, option2):
						self.goal = option1
					else:
						self.goal = option2

				return position == self.goal
			else:
				self.goal = None
				return self.isHome(position)"""
		"""isDeadEnd = self.isDeadEnd(position, self.deadEndMap)	#Find out if the position is a 

		depth = 0
		
		if (isDeadEnd) and self.isPellet(position, gameState):
			#print(depth)
			depth = self.findDepth(position, self.offenseCost, gameState)

		# now it considers the power pellet as a food pellet 
		# but we shouldn't be going home if the only thing we have is a power pellet. 
		# how do we take this into consideration??????

		isPellet = self.isPellet(position, gameState) or self.isPowerPellet(position, gameState)
		getThisPellet = True

		if (isDeadEnd and depth*2 + 1 > self.closeMazeEnemy):
			getThisPellet = False

		enemyDists = [util.manhattanDistance(position, self.enemyPos[i]) for i in range(2)]
		closeEnemy = min(enemyDists)"""
		if self.hasPellet:	#If the agent has a pellet, it can either get another one or go home to drop off the ones it has
			return self.isHome(position) or position in self.feasiblePellets
		else:	#Otherwise it has to get one
			return position in self.feasiblePellets

	#Finds the cost of moving to a particular position
	def offenseCost(self, position, gameState):
		cost = 0

		#Try to avoid moving near a partner. This makes it harder for defensive agents to kill both Pacmans at the same time
		partnerDist = util.manhattanDistance(gameState.getAgentPosition(self.teamIndices[0]), gameState.getAgentPosition(self.teamIndices[1]))
		if partnerDist < 2:
			cost += 15

		enemyDist = [util.manhattanDistance(position, self.enemyPos[i]) for i in range(2)]	#Get the manhattan distance from each enemy
		if self.isHome(self.enemyPos[0]) == self.isHome(self.enemyPos[1]):	#If they are either both home or both not home
			if enemyDist[0] < enemyDist[1]:	#Perform calculations based on the closer of the two
				closeEnemy = self.enemyPos[0]
				dist = enemyDist[0]
			else:
				closeEnemy = self.enemyPos[1]
				dist = enemyDist[1]
		else:	#If one is home and the other isn't, perform calulcations based on the one that isn't
			if self.isHome(self.enemyPos[0]):
				closeEnemy = self.enemyPos[1]
				dist = enemyDist[1]
			else:
				closeEnemy = self.enemyPos[0]
				dist = enemyDist[0]

		if self.isHome(closeEnemy) and self.distFromHome(closeEnemy) > dist:
			enemyCosts = [-10, -5, -3, -1, 0]	#Costs for getting close to an enemy on the home side
		else:
			enemyCosts = [9999, 100, 10, 5, 0]	#Costs for getting close to an enemy on the far side

		if dist < 1:	#Add the cost based on the distance from the closest enemy.
			cost += enemyCosts[0]
		elif dist < 2:
			cost += enemyCosts[1]
		elif dist < 3:
			cost += enemyCosts[2]
		elif dist < 4:
			cost += enemyCosts[3]
		else:
			cost += enemyCosts[4]

		return cost + 1	#Add one to the cost for the actual movement

	#Heuristic for moving towards an offensive goal state. It currently just returns 0, because we haven't prioritized fixing it.
	def offenseHeuristic(self, position, gameState, initialPosition):
		cost = 0

		if self.enemyDist[0] < self.enemyDist[1]:
			closeEnemy = self.enemyPos[0]
			enemyDist = self.enemyDist[0]
		else:
			closeEnemy = self.enemyPos[1]
			enemyDist = self.enemyDist[1]

		enemyCosts = [30, 15, 0]

		if enemyDist < 2:	#Get the appropriate cost for moving close to an enemy based on whether or not we're home
			cost += enemyCosts[0]
		elif enemyDist < 4:
			cost += enemyCosts[1]
		else:
			cost += enemyCosts[2]

		return 0	
		return cost + 1	#Add one to the cost for the actual movement

	#Determines whether or not a defensive agent should be moving towards this position
	def defenseGoal(self, position, gameState):
		if self.marked[(self.teamOrder - 1) * -1] == 0:	#Don't consider enemies marked by a partner
			enemyPos = [self.enemyPos[1]]
		elif self.marked[(self.teamOrder - 1) * -1] == 1:
			enemyPos = [self.enemyPos[0]]
		else:
			enemyPos = self.enemyPos
		kill = False	#Used to determine if we are trying to kill an opponent
		for goal in enemyPos:
			if self.isHome(goal):	#If any of the enemies we are considering are home, we are trying to kill them
				kill = True
		if kill:	#If we're trying to kill them, the only goal state is where they are
			return position in enemyPos and self.isHome(position)
		else:		#Try to move to a position on the home side closest to the enemy
			walls = gameState.getWalls()	#Get a map of the board and its height
			height = walls.height
			walls = walls.asList()
			if self.leftEdge == 0:	#Find out the closest we can get to the far side without crossing
				boundary = self.rightEdge - 1
			else:
				boundary = self.leftEdge

			#Search for the best spot according to the above rule
			bestSpot = (boundary, 0)
			bestDist = 999999
			for i in range(height):		#Loop through all legal positions on the boundary
				if (boundary, i) in walls:	#Anywhere that is a wall isn't legal, so we continue
					continue
				closestEnemy = min(util.manhattanDistance((boundary, i), enemyPos[j]) for j in range(len(enemyPos)))	#Find the manhattan distance to the closest enemy from this spot
				if closestEnemy < bestDist:	#Compare that to the current closest distance
					bestSpot = (boundary, i)
					bestDist = closestEnemy
			return position == bestSpot	#Return true if this spot will put us as close as possible to an opponent without crossing to their side

	#Goal for trying to cut off opponents. This function will only be used if there is exactly one enemy on the home side, so we don't need to check that again.
	def cutoffGoal(self, position, gameState):
		#We ignore self.marked because the whole point of this function is for both agents to chase the same opponent.
		return position in self.enemyPos and self.isHome(position)

	def defenseCost(self, position, gameState):
		if not self.isHome(position):		#If we're not home, getting close to an enemy is very bad
			closestEnemy = min(util.manhattanDistance(position, self.enemyPos[i]) for i in range(2))
			if closestEnemy < 1:
				return 9999
			if closestEnemy < 2:		#All these values were arbitrarily chosen
				return 100
			elif closestEnemy < 3:
				return 20
			else:
				return 1		#If there is no nearby enemy, there is no issue going onto the opponents side
		else:
			return 1

	#Very similar to defenseCost, except it doesn't allow both agents to follow the same path
	def cutoffCost(self, position, gameState):
		cost = 1
		if not self.isHome(position):
			closestEnemy = min(util.manhattanDistance(position, self.enemyPos[i]) for i in range(2))
			if closestEnemy < 1:
				cost += 9999
			if closestEnemy < 2:		#All these values were arbitrarily chosen
				cost += 100
			elif closestEnemy < 3:
				cost += 20
			else:
				cost += 1
		positions = [gameState.getAgentPosition(self.teamIndices[i]) for i in range(2)]
		if position == positions[self.teamOrder * -1 + 1]:	#If the position is the same as a partner, the cost becomes prohibitively high
			cost += 9999
		return cost

	def defenseHeuristic(self, position, gameState, initialPosition):
		if self.isHome(self.enemyPos[0]) or self.isHome(self.enemyPos[1]):		#If either of the enemies is on your home side, return the manhattan distance to the closest of them
			return min(util.manhattanDistance(position, self.enemyPos[i]) for i in range(2) if self.isHome(self.enemyPos[i]))
		else:		#Otherwise, return the manhattan distance to the location on the home side closest to one of them
			estimate = 0		#The estimated distance
			if self.leftEdge == 0:		#If leftEdge is 0, rightEdge marks the boundary we don't want to cross
				if position[0] < self.rightEdge:		#Get the straight line distance to the border
					estimate += self.rightEdge - position[0]
				else:
					estimate += position[0] - self.rightEdge
			else:
				if position[0] > self.leftEdge:		#Same as above, but this time leftEdge is the boundary we don't want to cross
					estimate += position[0] - self.leftEdge
				else:
					estimate += self.leftEdge - position[0] + 10

			closeEnemy = max(self.enemyDist)
			if closeEnemy < 3:
				estimate += 9
			elif closeEnemy < 5:
				estimate += 4
			return estimate


#An implementation of SmartAgent that focuses solely on defense
class DefenseAgent(SmartAgent):
	def registerInitialState(self, gameState):
		SmartAgent.registerInitialState(self, gameState)

	#Method that figures out what the best action would be
	def chooseAction(self, gameState):
		self.actionPrep(gameState)
		actions, goal = self.aStarSearch(gameState, self.defenseGoal, self.defenseHeuristic, self.defenseCost)		#Use the A* search to find the best path, to a goal, given those functions
		if len(actions) == 0:		#If the length of the actions is 0, we are already at a goal state, so just stay there
			return "Stop"
		else:		#Otherwise, take teh first step in reaching the goal
			return actions[0]

	#A function that estimates the distance to a defensive goal state
	def defenseHeuristic(self, position, gameState, initialPosition):
		if self.isHome(self.enemyPos[0]) or self.isHome(self.enemyPos[1]):		#If either of the enemies is on your home side, return the manhattan distance to the closest of them
			return min(util.manhattanDistance(position, self.enemyPos[i]) for i in range(2) if self.isHome(self.enemyPos[i]))
		else:		#Otherwise, return the manhattan distance to the location on the home side closest to one of them
			estimate = 0		#The estimated distance
			if self.leftEdge == 0:		#If leftEdge is 0, rightEdge marks the boundary we don't want to cross
				if position[0] < self.rightEdge:		#Get the straight line distance to the border
					estimate += self.rightEdge - position[0]
				else:
					estimate += position[0] - self.rightEdge
			else:
				if position[0] > self.leftEdge:		#Same as above, but this time leftEdge is the boundary we don't want to cross
					estimate += position[0] - self.leftEdge
				else:
					estimate += self.leftEdge - position[0] + 10

			closeEnemy = max(self.enemyDist)
			if closeEnemy < 3:
				estimate += 9
			elif closeEnemy < 5:
				estimate += 4
			return estimate

	#Determines whether a position is a defensive goal state
	def defenseGoal(self, position, gameState):
		if self.isHome(self.enemyPos[0]) or self.isHome(self.enemyPos[1]):		#If either enemy is on the home side, they are the only goal states
			return position in self.enemyPos and self.isHome(position)
		else:		#Try to move to a position on the home side closest to the enemy
			walls = gameState.getWalls()
			height = walls.height
			walls = walls.asList()
			if self.leftEdge == 0:
				boundary = self.rightEdge - 1
			else:
				boundary = self.leftEdge

			#Search for the best spot according to the above rule
			bestSpot = (boundary, 0)
			bestDist = 999999
			for i in range(height):		#Loop through all legal positions on the boundary
				if (boundary, i) in walls:
					continue
				closestEnemy = min(util.manhattanDistance((boundary, i), self.enemyPos[j]) for j in range(2))
				if closestEnemy < bestDist:
					bestSpot = (boundary, i)
					bestDist = closestEnemy
			return position == bestSpot

	#Returns the cost of moving to a position
	def defenseCost(self, position, gameState):
		if not self.isHome(position):		#If we're not home, getting close to an enemy is very bad
			closestEnemy = min(self.enemyDist)
			if closestEnemy < 3:		#All these values were arbitrarily chosen
				return 9
			elif closestEnemy < 5:
				return 4
			else:
				return 1		#If there is no nearby enemy, there is no issue going onto the opponents side
		else:
			return 1


#Agent that crosses onto the other side to steal one pellet at a time
class ThiefAgent(SmartAgent):

	def registerInitialState(self, gameState):
		SmartAgent.registerInitialState(self, gameState)

		self.hasPellet = False
		self.start = gameState.getAgentPosition(self.index)

		self.deadEndMap = self.createDeadEndMap(gameState)
		#print(deadEndMap)

	#Gets the best action to take
	def chooseAction(self, gameState):
		self.actionPrep(gameState)
		position = gameState.getAgentPosition(self.index)
		self.enemyMazeDists = [self.distancer.getDistance(position, self.enemyPos[i]) for i in range(2)]
		self.closeMazeEnemy = min(self.enemyMazeDists)

		pellets = self.getFood(gameState).asList()
		copy = self.getFood(gameState).asList()
		for pellet in pellets:
			if self.findDepth(pellet, self.offenseCost, gameState) * 2 + 1 > self.closeMazeEnemy:
				copy.remove(pellet)
		if len(copy) == 0:
			self.hasPellet = True

		if position == self.start:		#If we're at the starting position, we were killed so we don't have a pellet
			self.hasPellet = False

		actions, goal = self.aStarSearch(gameState, self.isGoal, self.pelletHeuristic, self.offenseCost)
		if len(actions) == 0:
			return "Stop"
		action = actions[0]

		if self.isHome((position[0] + self.actions[action][0], position[1] + self.actions[action][1])):	
		#If we've reached a goal state, we either pick up or drop off a pellet. Either way, hasPellet is notted
			self.hasPellet = False
		if self.isPellet((position[0] + self.actions[action][0], position[1] + self.actions[action][1]), gameState):	
		#If we've reached a goal state, we either pick up or drop off a pellet. Either way, hasPellet is notted
			self.hasPellet = True

		

		return action

	#Determines whether or not a position is an offensive goal state
	def isGoal(self, position, gameState):

		# closeEnemy = manhattan distance of the closer enemy 
		# if self has pellet 
		# if self is home and enemy distance 2: is goal
		# if self is pellet and enemy distance 2: is goal
		# if self no pellet
		# goal state is a pellet 


		isDeadEnd = self.isDeadEnd(position, self.deadEndMap)

		depth = 0
		
		if (isDeadEnd) and self.isPellet(position, gameState):
			#print(depth)
			depth = self.findDepth(position, self.offenseCost, gameState)

		# now it considers the power pellet as a food pellet 
		# but we shouldn't be going home if the only thing we have is a power pellet. 
		# how do we take this into consideration??????

		isPellet = self.isPellet(position, gameState) or self.isPowerPellet(position, gameState)
		getThisPellet = True

		if (isDeadEnd and depth*2 + 1 > self.closeMazeEnemy):
			getThisPellet = False

		enemyDists = [util.manhattanDistance(position, self.enemyPos[i]) for i in range(2)]
		closeEnemy = min(enemyDists)
		if self.hasPellet:
			return (self.isHome(position) or (isPellet and getThisPellet)) and closeEnemy > 2 #how can enemy be > 2
		else:
			return (isPellet and getThisPellet)

	#Finds the cost of moving to a given position
	def offenseCost(self, position, gameState):
		cost = 0

		partnerDist = util.manhattanDistance(gameState.getAgentPosition(self.teamIndices[0]), gameState.getAgentPosition(self.teamIndices[1]))
		if partnerDist < 2:
			cost += 15

		enemyDist = [util.manhattanDistance(position, self.enemyPos[i]) for i in range(2)]	#Can be used later to move towards an enemy on the home side
		if self.isHome(self.enemyPos[0]) == self.isHome(self.enemyPos[1]):
			if enemyDist[0] < enemyDist[1]:
				closeEnemy = self.enemyPos[0]
				dist = enemyDist[0]
			else:
				closeEnemy = self.enemyPos[1]
				dist = enemyDist[1]
		else:
			if self.isHome(self.enemyPos[0]):
				closeEnemy = self.enemyPos[1]
				dist = enemyDist[1]
			else:
				closeEnemy = self.enemyPos[0]
				dist = enemyDist[0]

		if self.isHome(closeEnemy) and self.distFromHome(closeEnemy) > dist:
			enemyCosts = [-10, -3, -1, 0]
		else:
			enemyCosts = [50, 20, 10, 0]

		if dist < 2:
			cost += enemyCosts[0]
		elif dist < 3:	#Get the appropriate cost for moving close to an enemy based on whether or not we're home
			cost += enemyCosts[1]
		elif dist < 4:
			cost += enemyCosts[2]
		else:
			cost += enemyCosts[3]

		return cost + 1	#Add one to the cost for the actual movement
		
	#Estimate for cost to reach closest pellet
	def pelletHeuristic(self, position, gameState, initialPosition):


		cost = 0

		if self.enemyDist[0] < self.enemyDist[1]:
			closeEnemy = self.enemyPos[0]
			enemyDist = self.enemyDist[0]
		else:
			closeEnemy = self.enemyPos[1]
			enemyDist = self.enemyDist[1]

		enemyCosts = [30, 15, 0]

		if enemyDist < 2:	#Get the appropriate cost for moving close to an enemy based on whether or not we're home
			cost += enemyCosts[0]
		elif enemyDist < 4:
			cost += enemyCosts[1]
		else:
			cost += enemyCosts[2]

		return 0
		return cost + 1	#Add one to the cost for the actual movement