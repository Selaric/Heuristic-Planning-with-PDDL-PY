# FF Heuristic Planner

## Overview

This project implements a domain-independent heuristic planner for solving classical planning problems written in PDDL.

The planner provides:
- **Best-First Search (BFS)** using heuristic value only  
- **A*** search combining path cost and heuristic value  
- **Fast-Forward (FF) heuristic** based on a Relaxed Planning Graph (RPG)

Tested on the **logistics** domain, but works for any compatible PDDL domain/problem.

---

## How to Run

Run the planner from the directory containing `Execute_Plan.py`:
python Execute_Plan.py   

**Arguments:**

- `<domain_folder>`: Folder with domain/problem files (e.g., `logistics`)
- `<problem_file>`: The PDDL problem file (e.g., `pb1.pddl`)
- `<search_algorithm>`: Search algorithm to use:
    - `BFS` (Best-First Search)
    - `A*` (A-star Search)

**Example:**
python Execute_Plan.py logistics pb1.pddl BFS
or
python Execute_Plan.py logistics pb1.pddl A*

---

## Output Format

After running, the planner prints:

- Domain name
- Problem file name
- Plan (sequence of actions)
- Plan length
- Number of expanded nodes
- Number of heuristic calls
- Execution time

**Sample Output:**
Domain: logisticsProblem: pb1.pddlPlan:(drive a b)(drive b c)(load c)(drive c d)(unload d)Plan length: 5Expanded nodes: 6Heuristic calls: 8Execution time: 0.0012

---

## Implementation Details

**FF Heuristic:**  
- Based on a Relaxed Planning Graph (RPG): ignores delete effects, expands layers until all goals appear, extracts a relaxed plan, returns plan length as heuristic.

**Best-First Search:**  
- Uses only heuristic value for ranking:  
  `f(n) = h(n)`

**A\***  
- Combines path cost and heuristic:  
  `f(n) = g(n) + h(n)`  
  where `g(n)` is depth (cost so far), `h(n)` is FF heuristic value.

---

## Requirements

- Python 3.x


---

