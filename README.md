# Project: Greedy vs Divide & Conquer — Real-World Algorithmic Case Studies

## Overview

This project implements and benchmarks two classical algorithmic paradigms applied to realistic industrial scenarios:

1. **EV Truck Charging Planning (Greedy Algorithm)**
   Optimize the route of an electric freight truck to reach its destination with the *fewest possible charging stops* while ensuring the truck never runs out of battery.

2. **Closest Cell Towers (Divide & Conquer Algorithm)**
   Identify the closest pair of candidate 5G/6G tower sites on a 2D map to minimize interference and redundant deployment.

Both problems demonstrate how theoretical algorithms translate directly into real-world logistics and network-planning challenges.

---

## Problem 1: EV Truck Charging Planning

* **Goal:** Reach mile `D` using the minimum number of recharging stops.
* **Approach:** Greedy range-extension algorithm using a max-heap of passed stations.
* **Time Complexity:** `O(N log N)`
* **Validation:** Compared against a brute-force dynamic programming solver for small inputs.
* **Output:**

  * `ev_charging_times.csv` → average runtime (milliseconds) vs. number of stations (`n`)

---

## Problem 2: Closest Cell Towers

* **Goal:** Find the pair of towers with the minimum Euclidean distance.
* **Approach:** Classical Divide & Conquer algorithm using the strip method.
* **Time Complexity:** `O(N log N)`
* **Validation:** Compared against a brute-force `O(N²)` baseline for correctness.
* **Output:**

  * `celltower_closestpair_times.csv` → runtime comparison of brute force vs. divide-and-conquer

---

## How to Run

```bash
g++ -std=c++17 -O2 Project.cpp -o project
./project
```

Optional command-line arguments:

```
--seed <int>       Random seed (default 2025)
--min <n>          Minimum N
--max <n>          Maximum N
--step <n>         Increment step size
--trials <t>       Number of trials per N
--only ev          Run only the EV charging experiment
--only closest     Run only the tower experiment
```

---

## Generated Files

| File                              | Description                             |
| --------------------------------- | --------------------------------------- |
| `ev_charging_times.csv`           | Timing data for EV Greedy algorithm     |
| `celltower_closestpair_times.csv` | Timing data for Closest-Pair algorithms |

Use these CSVs for generating runtime plots in your report or Overleaf document.

---

## Key Insights

* **Greedy optimality:** Charging only when necessary and always choosing the best past station minimizes stops.
* **Divide & Conquer efficiency:** The strip method avoids checking all pairs, achieving near-linear scaling.
* **Empirical trends:** CSV timing data confirm theoretical complexities — quadratic vs. `n log n` growth.
