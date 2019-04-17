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

red = False

def createTeam(firstIndex, secondIndex, isRed,
               first = 'ThiefAgent', second = 'DefenseAgent'):
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

  # The following line is an example only; feel free to change it.
  red = isRed
  if isRed:
  	return [eval(first)(firstIndex), eval(second)(secondIndex)]
  else:
  	return [eval('ThiefAgent')(firstIndex), eval('ThiefAgent')(secondIndex)]

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



	#Initialization stuff. Creates some variables and figures out where the boundary between either side is.
	def registerInitialState(self, gameState):
		CaptureAgent.registerInitialState(self, gameState)

		self.teamIndices = CaptureAgent.getTeam(self, gameState)
		self.opponentIndices = CaptureAgent.getOpponents(self, gameState)
		self.enemyPos = [None, None]

		position = gameState.getAgentPosition(self.index)		#This figures out where the boundary between either side is
		gameMap = gameState.getWalls()
		boundary = gameMap.width / 2
		if position[0] < boundary:
			self.leftEdge = 0
			self.rightEdge = boundary
		else:
			self.leftEdge = boundary
			self.rightEdge = gameMap.width

		

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

	        	
	            #isDead, depth = self.isDeadEnd(curPosition[0], gameState)
	            #if (not isDead):
	            	#break

	            
	            
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
	    print curPosition
	    while curPosition != position:  #Until we reach the starting state, push actions into the beginning of the answer
	        ans.insert(0, path[curPosition][1])
	        curPosition = path[curPosition][0]

	    return ans

	#Determine if the Pacman in on it's home side or on the enemies side
	def isHome(self, position):
		if position is None:
			return False
		else:
			return position[0] >= self.leftEdge and position[0] < self.rightEdge

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


		
		#return (position in self.getFood(gameState).asList()) or (position in self.getCapsules(gameState))
		return position in self.getFood(gameState).asList()


	def isDeadEnd(self, position, deadEndMap):
		#print (position in deadEndMap.asList())
		return position in deadEndMap.asList()

	def findDepth(self, position, costFunction, gameState):
		curState = self.breadthFirstSearch(position, costFunction, gameState)

		#return 1
		#print (self.distancer.getDistance(position, curState))
		
		return self.distancer.getDistance(position, curState)

	def breadthFirstSearch(self, position, costFunction, gameState):
	    #"""Search the shallowest nodes in the search tree first."""
	    
	    visited = []    #Set up the initial state for bfs
	    path = {}
	    queue = util.Queue()
	    #state = problem.getStartState()
	    state = position
	    queue.push(state)
	    visited.append(state)

	    walls = gameState.getWalls().asList()

	    
	    while not queue.isEmpty():  #Continue looking until we have found the goal or tried looking everywhere
	        curState = queue.pop()
	        #if self.isGoalState(curState):   #If the current state is the goal state, we can exit and move on to generating the path
	            #break;

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

	    '''
	            
	    ans = []
	    while curState != position:  #Until we reach the starting state, push actions into the beginning of the answer
	        ans.insert(0, path[curState][1])
	        curState = path[curState][0]
	    '''


	    #print curState
	    return curState
	    #print ans
	    #return ans


	def createDeadEndMap(self, gameState):

		print("create deadEndmap called!!!!!")
		walls = gameState.getWalls()
		deadEnds = walls.copy()

		done = False

		# for each non walls in deadEnd, if 3 sides are walls, make it a wall
		# until there are no more locations like this 

		while (not done):

			done = True

			# iterate every location (actually it should be 1/2 of the locations)
			for x in range(walls.width):
				for y in range(walls.height):

					if (not deadEnds[x][y]): 
						isWall1 = deadEnds[x-1][y]
						isWall2 = deadEnds[x][y-1]
						isWall3 = deadEnds[x+1][y]
						isWall4 = deadEnds[x][y+1]

						# if 3 sides are walls, make it a wall 
						if ((isWall1 and isWall2 and isWall3) or (isWall1 and isWall2 and isWall4) or (isWall1 and isWall3 and isWall4) or (isWall2 and isWall3 and isWall4)): 
							deadEnds[x][y] = True
							done = False
						

		'''
		for x in range(walls.width):
			for y in range(walls.height):
				if (walls[x][y]): deadEnds[x][y] = False
		'''

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

	


#An implementation of SmartAgent that focuses solely on defense
class DefenseAgent(SmartAgent):

	

	def registerInitialState(self, gameState):
		SmartAgent.registerInitialState(self, gameState)

		

	#Method that figures out what the best action would be
	def chooseAction(self, gameState):
		self.actionPrep(gameState)
		actions = self.aStarSearch(gameState, self.defenseGoal, self.defenseHeuristic, self.defenseCost)		#Use the A* search to find the best path, to a goal, given those functions
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

		actions = self.aStarSearch(gameState, self.isGoal, self.pelletHeuristic, self.offenseCost)
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
		"""if self.hasPellet:	#If we have a pellet, we're trying to go home
			return self.isHome(position)
		else:	#Otherwise we're trying to get one
			return position in self.getFood(gameState).asList()"""

	#Finds the cost of moving to a given position
	def offenseCost(self, position, gameState):
		cost = 0

		


		partnerDist = util.manhattanDistance(gameState.getAgentPosition(self.teamIndices[0]), gameState.getAgentPosition(self.teamIndices[1]))
		if partnerDist < 3:
			cost += 10

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

		if self.isHome(closeEnemy) and self.distFromHome(closeEnemy) > dist and red:
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