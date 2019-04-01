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

class SmartAgent(CaptureAgent):
	actions = {"Stop": (0,0), "North": (0,1), "East": (1,0), "South": (0,-1), "West": (-1,0)}

	def registerInitialState(self, gameState):
		CaptureAgent.registerInitialState(self, gameState)

		self.teamIndices = CaptureAgent.getTeam(self, gameState)
		self.opponentIndices = CaptureAgent.getOpponents(self, gameState)
		self.enemyPos = [None, None]

		position = gameState.getAgentPosition(self.index)
		gameMap = gameState.getWalls()
		boundary = gameMap.width / 2
		if position[0] < boundary:
			self.leftEdge = 0
			self.rightEdge = boundary
		else:
			self.leftEdge = boundary
			self.rightEdge = gameMap.width

	def actionPrep(self, gameState):
		self.enemyPos = (gameState.getAgentPosition(self.opponentIndices[0]), gameState.getAgentPosition(self.opponentIndices[1]))
		self.enemyDist = [util.manhattanDistance(gameState.getAgentPosition(self.index), self.enemyPos[i]) for i in range(2)]

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
	            break;
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
	    while curPosition != position:  #Until we reach the starting state, push actions into the beginning of the answer
	        ans.insert(0, path[curPosition][1])
	        curPosition = path[curPosition][0]
	    return ans

	def isHome(self, position):
		if position is None:
			return False
		else:
			return position[0] >= self.leftEdge and position[0] < self.rightEdge

	def isPellet(self, position, gameState):
		return position in self.getFood(gameState).asList()

	def enemyDistance(self, position, gameState):
		return min(util.manhattanDistance(position, self.enemyPos[i]) for i in range(2))

	def getSuccessors(self, position, walls, costFunction, gameState):
		nextStates = []
		for action in self.actions:
			nextPos = (position[0] + self.actions[action][0], position[1] + self.actions[action][1])
			if nextPos not in walls:
				nextStates.append((nextPos, action, costFunction(position, gameState)))
		return nextStates

class DefenseAgent(SmartAgent):
	def registerInitialState(self, gameState):
		SmartAgent.registerInitialState(self, gameState)

	def chooseAction(self, gameState):
		self.actionPrep(gameState)
		actions = self.aStarSearch(gameState, self.defenseGoal, self.defenseHeuristic, self.defenseCost)
		if len(actions) == 0:
			return "Stop"
		else:
			return actions[0]

	def defenseHeuristic(self, position, gameState, initialPosition):
		if self.isHome(self.enemyPos[0]) or self.isHome(self.enemyPos[1]):
			return min(util.manhattanDistance(position, self.enemyPos[i]) for i in range(2))
		else:
			estimate = 0
			if self.leftEdge == 0:
				if position[0] < self.rightEdge:
					estimate += self.rightEdge - position[0]
				else:
					estimate += position[0] - self.rightEdge + 10
			else:
				if position[0] > self.leftEdge:
					estimate += position[0] - self.leftEdge
				else:
					estimate += self.leftEdge - position[0] + 10
			if util.manhattanDistance(position, self.enemyPos[0]) < util.manhattanDistance(position, self.enemyPos[1]):
				estimate += util.manhattanDistance(position, self.enemyPos[0])
			else:
				estimate += util.manhattanDistance(position, self.enemyPos[1])
			return estimate

	def defenseGoal(self, position, gameState):
		if self.isHome(self.enemyPos[0]) or self.isHome(self.enemyPos[1]):
			return position in self.enemyPos and self.isHome(position)
		else:
			walls = gameState.getWalls()
			height = walls.height
			walls = walls.asList()
			if self.leftEdge == 0:
				boundary = self.rightEdge - 1
			else:
				boundary = self.leftEdge

			bestSpot = (boundary, 0)
			bestDist = 999999
			for i in range(height):
				if (boundary, i) in walls:
					continue
				closestEnemy = min(util.manhattanDistance((boundary, i), self.enemyPos[j]) for j in range(2))
				if closestEnemy < bestDist:
					bestSpot = (boundary, i)
					bestDist = closestEnemy
			return position == bestSpot

	def defenseCost(self, position, gameState):
		if not self.isHome(position):
			closestEnemy = min(self.enemyDist)
			if closestEnemy < 3:
				return 9
			elif closestEnemy < 5:
				return 4
			else:
				return 1
		else:
			return 1


class ThiefAgent(SmartAgent):

	def registerInitialState(self, gameState):
		SmartAgent.registerInitialState(self, gameState)

		self.hasPellet = False
		self.start = gameState.getAgentPosition(self.index)

	def chooseAction(self, gameState):
		self.actionPrep(gameState)
		pellets = self.getFood(gameState).asList()
		position = gameState.getAgentPosition(self.index)
		if position== self.start:
			self.hasPellet = False

		if self.hasPellet:
			action = self.aStarSearch(gameState, self.isGoal, self.pelletHeuristic, self.offenseCost)[0]
		else:
			action = self.aStarSearch(gameState, self.isPellet, self.pelletHeuristic, self.offenseCost)[0]

		if self.isGoal((position[0] + self.actions[action][0], position[1] + self.actions[action][1]), gameState):
			self.hasPellet = not self.hasPellet

		return action

	def isGoal(self, position, gameState):
		if self.hasPellet:
			if position[0] >= self.leftEdge and position[0] < self.rightEdge:
				return True
			else:
				return False
		else:
			return position in self.getFood(gameState).asList()

	def offenseCost(self, position, gameState):
		if self.isHome(position):
			enemyCosts = [-15, -7, 0]
		else:
			enemyCosts = [30, 15, 0]

		partnerDist = util.manhattanDistance(gameState.getAgentPosition(self.teamIndices[0]), gameState.getAgentPosition(self.teamIndices[1]))
		if partnerDist < 5:
			cost = 10
		else:
			cost = 0

		closestEnemy = min(self.enemyDist)
		if closestEnemy < 3:
			cost += enemyCosts[0]
		elif closestEnemy < 6:
			cost += enemyCosts[1]
		else:
			cost += enemyCosts[2]

		return cost + 1

	def pelletHeuristic(self, position, gameState, initialPosition):
		minDistance = 99999
		time = self.getMazeDistance(initialPosition, position)
		pellets = self.getFood(gameState).asList()
		for pellet in pellets:
			if util.manhattanDistance(position, pellet) < minDistance:
				minDistance = util.manhattanDistance(position, pellet)

		partnerDist = util.manhattanDistance(gameState.getAgentPosition(self.teamIndices[0]), gameState.getAgentPosition(self.teamIndices[1]))
		if partnerDist < 5:
			minDistance += 10

		enemyDist = [util.manhattanDistance(position, self.enemyPos[i]) for i in range(2)]
		for i in range(2):
			if enemyDist[i] == 0:
				if time < 3:
					minDistance += 60
				else:
					minDistance += 20
			elif enemyDist[i] < 3:
				if time < 3:
					minDistance += 30
				else:
					minDistance += 10
			elif enemyDist[i] < 6:
				if time < 2:
					minDistance += 15
				else:
					minDistance += 5
		return minDistance