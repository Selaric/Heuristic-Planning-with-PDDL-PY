#Code written by: Felipe Meneguzzi,Mauricio Magnaguagno (PhD Student), Leonardo Rosa Amado (PhD Student) and Gabriel Paludo Licks (MSc Student)
import sys
import queue as queue
from pddl.action import Action
from pddl.heuristic import Heuristic
from pddl.pddl_parser import PDDLParser
from pddl.pddl_planner import PDDLPlanner
from nose.tools import assert_equal




# Objects example
# An action to move robot r1 from location l1 to location l2
#In this cell we provide two methods to aid you in your implementation. First, we provide a method to verify if an action is applicable, in a given a state (all positive preconditions are contained in the state, and no negative preconditions are contained in the state). Second, we provide a method to apply an action in a given state, returning the new resulting state.
def applicable(state, precondition):
    positive, negative = precondition 
    return positive.issubset(state) and not negative.intersection(state)

def apply(state, effects):
    positive, negative = effects
    return frozenset(state.difference(negative).union(positive))

#In this section, we provide a python implementation of how to build the Relaxed Planning Graph.
#  You can use this implementation to obtain the RPG based (FF) heuristic. Use wisely!
def build_rpg(actions, state, goals):
    i = 0
    factlevel = [state]
    actionlevel = []
    positive_goals = goals[0]
    while not positive_goals.issubset(factlevel[i]):
        # actionlevel[i]
        actionlevel.append([a for a in actions if a.positive_preconditions.issubset(factlevel[i])])
        # factlevel[i+1]
        factlevel.append(factlevel[i].union([pre for a in actionlevel[i] for pre in a.add_effects]))
        if factlevel[i+1] == factlevel[i]:
            return "G unreachable"
        i += 1
    return (factlevel, actionlevel)

#You will implement the Relaxed Plan (a.k.a. Fast Forward) heuristic. Return estimated distance between current state 
#and goal , a number between 0 (when s|=g) and infinity (when 'g' is unreachable).

#Notice that I've already implemented the code to build the best supporter for each predicate. The actual heuristic consists of computing a relaxed plan by backchaining from the goals via their best supporters and returning the size of the resulting relaxed plan.

#Your code must be contained in the h(self, actions, state, goals) function in the cell below. You can create additional functions (do not forget to comment the code intelligibly). H takes the following inputs:

#actions: list of ground actions
#state: current state
#goals: tuple with (positive predicates, negative predicates) of the goal

class FastForwardHeuristic(Heuristic):

    def build_bs_table(self, actions, initial_state, goal):
        self.empty_action = Action('nop', frozenset(), frozenset(), frozenset(), frozenset(), frozenset())
        self.bs_table = dict()
        self.update_bs_table(actions, initial_state, goal)

    def update_bs_table(self, actions, initial_state, goal):
        positive_g, negative_g = goal
        if not positive_g:
            return 0
        reachable = set(initial_state)
        missing_positive_g = set(positive_g)
        last_state = None
        t_add = {p: 0 for p in initial_state}  # Everything in the initial state costs 0
        add = 0
        while last_state != reachable:
            g_reached = missing_positive_g.intersection(reachable)
            if g_reached:
                add += sum(t_add[g] for g in g_reached)
                missing_positive_g -= g_reached
                if not missing_positive_g:
                    return add
            last_state = set(reachable)
            for a in actions:
                if a.positive_preconditions <= last_state:
                    new_reachable = a.add_effects - reachable
                    for eff in new_reachable:
                        if eff in t_add:
                            old_t_add = t_add[eff]
                            t_add[eff] = min(sum(t_add[pre] for pre in a.positive_preconditions)+1, t_add[eff])
                            if t_add[eff] != old_t_add:
                                self.bs_table[eff] = a  # best supporter changed
                        else:
                            t_add[eff] = sum(t_add[pre] for pre in a.positive_preconditions)+1
                            self.bs_table[eff] = a
                    reachable.update(new_reachable)
        return float("inf")

    def best_supporter(self, actions, initial_state, g):
        if g not in self.bs_table.keys():
            return self.empty_action
        return self.bs_table[g]

    def h(self, actions, initial_state, goal):
        # Build best supporter (I've done this for you)
        self.build_bs_table(actions, initial_state, goal)
        
        if self.stats:
            self.stats.h_calls += 1
        positive_goals, negative_goals = goal
        if positive_goals.issubset(initial_state):
            return 0
            
        r_plan = set()  # relaxed plan actions
        subgoals = set(positive_goals)
    
        while subgoals:
            g = subgoals.pop()
    
            # If already true in initial state, skip
            if g in initial_state:
                continue
            
            # Get best supporter action
            supporter = self.best_supporter(actions, initial_state, g)
    
            # If no supporter found, unreachable
            if supporter.name == 'nop':
                return float("inf")
    
            # Add action to relaxed plan
            if supporter not in r_plan:
                r_plan.add(supporter)
    
                # Add its positive preconditions as new subgoals
                for pre in supporter.positive_preconditions:
                    if pre not in initial_state:
                        subgoals.add(pre)
                        
        return len(r_plan)
    
class MaxHeuristic(Heuristic):
    #Positive goals and negative goals in a tuple
    def h(self, actions, state, goals):
        positive_goals, negative_goals = goals

        # If goal already satisfied
        if positive_goals.issubset(state):
            return 0

        rpg = build_rpg(actions, state, goals)

        # If unreachable
        if rpg == "G unreachable":
            return float("inf")

        factlevels, actionlevels = rpg

        max_level = 0

        for g in positive_goals:
            found = False
            for level, facts in enumerate(factlevels):
                if g in facts:
                    max_level = max(max_level, level)
                    found = True
                    break
            if not found:
                return float("inf")
        return max_level

a1 = Action(
    'move', #Action name
    ['r1', 'l1', 'l2'], #Parameters
    frozenset({('at', 'r1', 'l1'), ('adjacent', 'l1', 'l2')}), #Positive preconditions
    frozenset({('occupied', 'l2')}), #Negative preconditions
    frozenset({('at', 'r1', 'l2'), ('occupied', 'l2')}), #Add effects
    frozenset({('at', 'r1', 'l1'), ('occupied', 'l1')})  #Del effects
)

# Get each element from the action
print(a1.name)
print(a1.parameters)
print(a1.positive_preconditions)
print(a1.negative_preconditions)
print(a1.add_effects)
print(a1.del_effects)

print('---------------------------------------------')

# The list of actions contains all possible actions
actions = [
    a1,
    # ...
]

# Only positive literals are present in the initial state
initial_state = frozenset({
    ('on', 'ca', 'pallet'),
    ('at', 'r1', 'l1'),
    ('belong', 'k1', 'l1'),
    ('adjacent', 'l1', 'l2'), ('adjacent', 'l2', 'l1'), ('attached', 'q2', 'l2'),
    ('empty', 'k2'),
    ('attached', 'p1', 'l1'), ('occupied', 'l1'),
    ('empty', 'k1'),
    # ...
})

# Goal literals are split in two, positive and negative
positive_goal = frozenset({('in', 'cb', 'p1'), ('in', 'ca', 'p1')})
negative_goal = frozenset()

#Test if the action move (variable a1) is applicable in our initial state (initial_state)
applicable_action = applicable(initial_state, (a1.positive_preconditions, a1.negative_preconditions))
print('Is the action move applicable?', applicable_action)

print('---------------------------------------------')

#Apply the action move in the initial state
resulting_state = apply(initial_state, (a1.add_effects, a1.del_effects))
print('Resulting state:')
for predicate in resulting_state:
    print(predicate)

print('---------------------------------------------')

#Test if the goal was achieved  
goal_achieved = applicable(resulting_state, (positive_goal, negative_goal))
print('Was the goal achieved?', goal_achieved)

print('---------------------------------------------')

# The output plan from the planner is either a list of actions or failure (None)
# An empty plan is valid
plan = []
# Preconditions and effects are empty when obtained from a plan file, may be filled when obtained from the planner
plan = [
    Action('take', ['k1', 'cc', 'cb', 'p1', 'l1'], [], [], [], []), 
    Action('load', ['k1', 'r1', 'cc', 'l1'], [], [], [], []),
    Action('move', ['r1', 'l1', 'l2'], [], [], [], []),
    Action('unload', ['k2', 'r1', 'cc', 'l2'], [], [], [], [])
    # ...
]
# Failure
plan = None

# A valid plan is either true or false
valid_plan   = True
invalid_plan = False

#Test the heuristic functions
#We will test the Heuristics using 5 different domains, dinner, tsp, dwr, dompteur, and logistics. The state used is the initial state of each problem.

#At each execution we show the expected value for the heuristic.

# from pddl.heuristic import Heuristic



# The following should be visible to the students
# Load some domain and some problem
dwr = "examples/dwr/dwr.pddl"
pb1_dwr = "examples/dwr/pb1.pddl"
pb2_dwr = "examples/dwr/pb2.pddl"

tsp = "examples/tsp/tsp.pddl"
pb1_tsp = "examples/tsp/pb1.pddl"
pb2_tsp = "examples/tsp/pb2.pddl"

dinner = "examples/dinner/dinner.pddl"
pb1_dinner = "examples/dinner/pb1.pddl"

dompteur = "examples/dompteur/dompteur.pddl"
pb1_dompteur = "examples/dompteur/pb1.pddl"

logistics = "examples/logistics/logistics.pddl"
pb1_logistics = "examples/logistics/pb1.pddl"
pb2_logistics = "examples/logistics/pb2.pddl"

def parse_domain_problem(domain, problem):
    parser = PDDLParser()
    parser.parse_domain(domain)
    parser.parse_problem(problem)
    # Grounding process
    actions = []
    for action in parser.actions:
        for act in action.groundify(parser.objects):
            actions.append(act)
    return parser, actions

def test_heuristic(domain, problem, h, expected):
    parser, actions = parse_domain_problem(domain, problem)
    v = h.h(actions, parser.state, (parser.positive_goals, parser.negative_goals))
    print("Expected " + str(expected) + ", got:", str(v) + ('. Correct!' if v == expected else '. False!'))

# Apply Hmax to initial states of many problems from many domains
h = MaxHeuristic()
test_heuristic(dwr, pb1_dwr, h, 6)
test_heuristic(dwr, pb2_dwr, h, 0)
test_heuristic(tsp, pb1_tsp, h, 2)
test_heuristic(tsp, pb2_tsp, h, 2)
test_heuristic(dinner, pb1_dinner, h, 1)
test_heuristic(dompteur, pb1_dompteur, h, 2)
test_heuristic(logistics, pb1_logistics, h, 4)
test_heuristic(logistics, pb2_logistics, h, 4)

# h = AdditiveHeuristic()
# test_heuristic(dwr, pb1_dwr, h, 38)
# test_heuristic(dwr, pb2_dwr, h, 0)
# test_heuristic(tsp, pb1_tsp, h, 8)
# test_heuristic(tsp, pb2_tsp, h, 8)
# test_heuristic(dinner, pb1_dinner, h, 2)
# test_heuristic(dompteur, pb1_dompteur, h, 2)
# test_heuristic(logistics, pb1_logistics, h, 7)
# test_heuristic(logistics, pb2_logistics, h, 10)

h = FastForwardHeuristic()
test_heuristic(dwr, pb1_dwr, h, 16)
test_heuristic(dwr, pb2_dwr, h, 0)
test_heuristic(tsp, pb1_tsp, h, 5)
test_heuristic(tsp, pb2_tsp, h, 5)
test_heuristic(dinner, pb1_dinner, h, 2)
test_heuristic(dompteur, pb1_dompteur, h, 2)
test_heuristic(logistics, pb1_logistics, h, 5)
test_heuristic(logistics, pb2_logistics, h, 5)

dompteur = "examples/dompteur/dompteur.pddl"
pb1_dompteur = "examples/dompteur/pb1.pddl"

logistics = "examples/logistics/logistics.pddl"
pb1_logistics = "examples/logistics/pb1.pddl"
pb2_logistics = "examples/logistics/pb2.pddl"

def check_heuristic(domain, problem, h, expected):
    parser, actions = parse_domain_problem(domain, problem)
    v = h.h(actions, parser.state, (parser.positive_goals, parser.negative_goals))
    print("Expected " + str(expected) + ", got:", str(v) + ('. Correct!' if v == expected else '. False!'))
    assert_equal(expected, v)

#Planner
#Implement the planner solve function
#You will implement the Best-first search. This search must use the implemented Max-cost heuristic. The search receives a domain pddl file and a problem pddl file (both are already parsed for you). The search must always return an optimal plan, given that there is a solution for the given problem in the specified domain. Your code must be contained in the solve(self, actions, state, goals) function (in the following cell). Solve takes the following inputs:

#actions: list of grounded actions
#state: initial state of the problem file
#goals: tuple with (positive predicates, negative predicates) of the goal#
class HeuristicPlanner(PDDLPlanner):

    def __init__(self, heuristic=MaxHeuristic(),
                 search_type="astar",
                 verbose=False,
                 collect_stats=False):
        super().__init__(verbose, collect_stats)
        self.h = heuristic
        self.search_type = search_type

    def solve(self, actions, state, goals):

        if self.stats:
            self.h.stats = self.stats

        frontier = []
        visited = dict()

        g_cost = 0
        h_cost = self.h.h(actions, state, goals)

        if self.search_type == "bestfs":
            priority = h_cost
        else:
            priority = g_cost + h_cost

        frontier.append((priority, state, g_cost, []))

        while frontier:

            frontier.sort(key=lambda x: x[0])
            priority, current_state, g_cost, plan = frontier.pop(0)

            if self.stats:
                self.stats.nodes += 1

            # Goal test
            positive_goals, negative_goals = goals
            if positive_goals.issubset(current_state) and \
               negative_goals.isdisjoint(current_state):
                return plan

            # Skip worse paths
            if current_state in visited and visited[current_state] <= g_cost:
                continue

            visited[current_state] = g_cost

            for action in actions:

                if action.positive_preconditions.issubset(current_state) and \
                   action.negative_preconditions.isdisjoint(current_state):

                    new_state = apply(current_state,
                                      (action.add_effects,
                                       action.del_effects))

                    new_g = g_cost + 1

                    if new_state in visited and visited[new_state] <= new_g:
                        continue

                    new_h = self.h.h(actions, new_state, goals)

                    if self.search_type == "bestfs":
                        new_priority = new_h
                    else:
                        new_priority = new_g + new_h

                    frontier.append((new_priority,
                                     new_state,
                                     new_g,
                                     plan + [action]))

        return None


domain_folder = sys.argv[1]          # logistics
problem_file = sys.argv[2]           # pb1.pddl
algorithm = sys.argv[3]              # BFS or A*

domain_path = f"examples/{domain_folder}/{domain_folder}.pddl"
problem_path = f"examples/{domain_folder}/{problem_file}"

if algorithm == "BFS":
    planner = HeuristicPlanner(
        heuristic=FastForwardHeuristic(),
        search_type="bestfs",
        collect_stats=True
    )
elif algorithm == "A*":
    planner = HeuristicPlanner(
        heuristic=FastForwardHeuristic(),
        search_type="astar",
        collect_stats=True
    )
else:
    raise ValueError("Algorithm must be BFS or A*")

plan, _ = planner.solve_file(domain_path, problem_path)

print(f"Domain: {domain_folder}")
print(f"Problem: {problem_file}")
print("Plan:")

if plan:
    for action in plan:
        params = " ".join(action.parameters)
        print(f"({action.name} {params})")
else:
    print("No actions (goal already satisfied)")

print("Plan length:", len(plan))
print("Expanded nodes:", planner.stats.nodes)
print("Heuristic calls:", planner.stats.h_calls)
print("Execution time:", planner.stats.time)
