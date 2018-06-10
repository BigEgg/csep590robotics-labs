import sys

sys.path.insert(0, '../lab12')

from learningAgents import ReinforcementAgent
from featureExtractors import *

import random, util, math

class HumanInLoopReinforcementLearningAgent(ReinforcementAgent):
    """
      Instance variables you have access to
        - self.epsilon (exploration prob)
        - self.alpha (learning rate)
        - self.discount (discount rate)

      Functions you should use
        - self.getLegalActions(state)
          which returns legal actions for a state
    """
    def __init__(self, **args):
        "You can initialize Q-values here..."
        ReinforcementAgent.__init__(self, **args)

        "*** YOUR CODE HERE ***"
        self.q_values = {}

    def getQValue(self, state, action):
        """
          Returns Q(state,action)
          Should return 0.0 if we have never seen a state
          or the Q node value otherwise
        """
        "*** YOUR CODE HERE ***"
        if (state, action) in self.q_values:
            return self.q_values[(state, action)]
        else:
            return 0.0

    def computeValueFromQValues(self, state):
        """
          Returns max_action Q(state,action)
          where the max is over legal actions.  Note that if
          there are no legal actions, which is the case at the
          terminal state, you should return a value of 0.0.
        """
        "*** YOUR CODE HERE ***"
        (best_action, max_q) = self.computeValueAndActionFromQValues(state)
        return max_q

    def computeActionFromQValues(self, state):
        """
          Compute the best action to take in a state.  Note that if there
          are no legal actions, which is the case at the terminal state,
          you should return None.
        """
        "*** YOUR CODE HERE ***"
        (best_action, max_q) = self.computeValueAndActionFromQValues(state)
        return best_action

    def computeValueAndActionFromQValues(self, state):
        best_actions = []
        best_action = None
        max_q = None
        for action in self.getLegalActions(state):
            q = self.getQValue(state, action)
            if max_q is None or q > max_q:
                max_q = q
                best_actions = [action]
            elif q == max_q:
                best_actions.append(action)
        if max_q is None:
            max_q = 0.0
        if len(best_actions) == 0:
            best_action = None
        else:
            best_action = random.choice(best_actions)
        return (best_action, max_q)

    def getAction(self, state):
        """
          Compute the action to take in the current state.  With
          probability self.epsilon, we should take a random action and
          take the best policy action otherwise.  Note that if there are
          no legal actions, which is the case at the terminal state, you
          should choose None as the action.

          HINT: You might want to use util.flipCoin(prob)
          HINT: To pick randomly from a list, use random.choice(list)
        """
        # Pick Action
        return self.computeActionFromQValues(state)

    def update(self, state, action, nextState, reward):
        """
          The parent class calls this to observe a
          state = action => nextState and reward transition.
          You should do your Q-Value update here

          NOTE: You should never call this function,
          it will be called on your behalf
        """
        "*** YOUR CODE HERE ***"
        self.q_values[(state, action)] = self.getQValue(state, action) + self.alpha * (reward + self.discount * self.computeValueFromQValues(nextState) - self.getQValue(state, action))

    def getPolicy(self, state):
        return self.computeActionFromQValues(state)

    def getValue(self, state):
        return self.computeValueFromQValues(state)
