/*
Language: C++17

============================================================
PROBLEM 1 (GREEDY): EV TRUCK CHARGING PLANNING
============================================================

REAL-WORLD PROBLEM (logistics / sustainability):

You manage an electric freight truck driving along a highway from mile 0
to mile D. The truck starts with a full battery that can drive at most
R miles on a full charge.

Along the highway there are N public fast-charging stations at known
mile markers x_1, x_2, ..., x_N (strictly increasing). The final
destination at mile D also counts as "the last stop" (you do not need
to charge there, you just need to reach it).

Constraint:
 - The truck can *only* recharge at those stations.
 - When you stop, you fully recharge (go back to full range R).

Goal:
Pick charging stops so that the truck reaches mile D without ever
running out of battery AND using the *fewest possible charging stops*.
In practice, fewer stops = lower driver downtime and charging fees.

This is exactly what dispatch / logistics teams for EV delivery fleets
actually optimize.

ABSTRACTION:

We have an ordered set of stations S = {s_0, s_1, ..., s_k} where:
 - s_0 = 0 (starting position),
 - s_k = D (destination),
 - all s_i sorted increasingly,
 - distance between consecutive "usable" stops must never exceed R.

We want the *minimum number of refuels* to get from s_0 to s_k.

This is identical to the classic "minimum refueling stops" greedy model
on a line with a maximum travel radius.

GREEDY ALGORITHM (PRIORITY-BASED RANGE EXTENSION):

High-level intuition:
Drive as far as you can. Each time you are about to run out of battery
(before reaching the next station you *must* reach), retroactively
"commit" to having charged at the station among those you passed that
gives you the farthest reachable range extension.

Implementation variant used here (well-known optimal strategy):
1. Sort all station mile markers, include start (0) and destination (D).
2. Travel forward. Keep track of how far we can currently go on our charge.
3. Whenever we cannot reach the next station with current charge,
   we "must" have charged earlier.
   Which earlier station should we choose? The BEST one (the one that
   gave us the farthest forward reach). We greedily pick the farthest
   useful station each time to minimize how often we stop.
4. Use a max-heap (priority queue) of stations we've passed but not yet
   "used" for charging. When we need more range, pop the best one
   (longest reachable extension) and count a charge stop.

This greedy is known optimal: delaying each charge until absolutely
necessary while always choosing the best-possible past station ensures
we never "waste" a stop on a weak station if a better one existed.
Exchange-argument style proof: if a non-greedy solution ever chooses an
earlier/weaker station instead of a later/stronger one, we can swap them
and never increase the #stops.

TIME COMPLEXITY:

We process N stations once, pushing/popping at most N times from a
priority queue. Each push/pop is O(log N). So total is O(N log N).

EXPERIMENT VERIFICATION:

We compare the greedy result to a brute-force dynamic programming solver
for smaller N to ensure it really finds the minimum charges.
Then we time the greedy solution over increasing N and write CSV data:
   ev_charging_times.csv  (n, avg_ms)

Those timings empirically confirm near O(N log N).


============================================================
PROBLEM 2 (DIVIDE & CONQUER): CLOSEST CELL TOWERS
============================================================

REAL-WORLD PROBLEM (telecom / network planning):

You are planning a 5G/6G cellular network in a metro area.
You have candidate tower coordinates on a 2D map (latitude/longitude
converted into planar meters for one city-scale zone).

Two towers that are *too close* cause:
 - radio interference,
 - regulatory headaches,
 - wasted capital because you don't want to build redundant towers.

So network engineers routinely ask:
"Which two proposed tower sites are closest to each other?"

This is exactly the "closest pair of points" problem.

ABSTRACTION:

Given a set P of n points in 2D (tower sites),
find the pair with minimum Euclidean distance.

DIVIDE & CONQUER ALGORITHM (CLASSICAL STRIP METHOD):

1. Sort points by x-coordinate into Px, and also keep a copy Py sorted by y.
2. Split Px into left half and right half at median x.
3. Recursively compute best distances d_L and d_R. Let d = min(d_L, d_R).
4. Build the "strip": all points within horizontal distance d of the
   vertical split line. Sort that strip by y.
5. For each point in the strip, compare it to only the next few points in
   y-order (it can be proven you only need to check up to 7 neighbors).
6. Return the best overall distance.

TIME COMPLEXITY:

The recurrence is T(n) = 2T(n/2) + O(n),
which solves to T(n) = O(n log n).

CORRECTNESS ARGUMENT (GEOMETRIC STRIP LEMMA SKETCH):

Any optimal closest pair is either entirely in the left half,
entirely in the right half, or "straddles" the boundary.

Recursive calls handle the first two cases.

For straddling pairs, it can be shown that in a d-by-2d rectangle,
you cannot pack more than a constant number of points (at most 8-ish)
without creating a pair closer than d. That geometric packing fact
implies that for each point in the strip, you only need to check a
constant number of nearby points in sorted-by-y order.

Thus we can examine all cross-boundary candidates in O(n) extra time
at each level of recursion, and we never miss the global closest pair.

EXPERIMENT VERIFICATION:

For "ground truth", we also compute the closest pair by brute force
O(n^2) distance checks. For large n, that's slow, but for smaller n we
can compare the answers to prove correctness and also measure runtime.

We output:
   celltower_closestpair_times.csv (n, brute_ms, dc_ms)

As n grows, brute force becomes ~quadratic, while divide-and-conquer
grows ~n log n.
*/

#include <bits/stdc++.h>
using namespace std;

// -------- timing utilities --------
static inline uint64_t now_us() {
    using namespace std::chrono;
    return duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
}

// -------- RNG helper --------
struct RNG {
    std::mt19937_64 gen;
    RNG(uint64_t seed=std::random_device{}()) : gen(seed) {}
    int randint(int lo, int hi) {
        std::uniform_int_distribution<int> d(lo, hi);
        return d(gen);
    }
    double randreal(double lo, double hi) {
        std::uniform_real_distribution<double> d(lo, hi);
        return d(gen);
    }
};

// ============================================================
// GREEDY PROBLEM: EV truck charging planning
// ============================================================

struct ChargingInstance {
    // stations[i] is the mile marker of station i
    // sorted strictly increasing, not including start=0 nor dest=D yet
    vector<int> stations;
    int D; // destination mile marker
    int R; // max miles on full battery
};

// generate random synthetic scenario:
// highway length D miles, range R, N random station mile markers along the way
static ChargingInstance make_random_charging_instance(RNG &rng, int N,
                                                      int D = 2000,
                                                      int R = 300) {
    // We'll choose N stations in (0, D), sort them
    vector<int> st;
    st.reserve(N);
    for (int i=0;i<N;i++){
        st.push_back(rng.randint(1, D-1));
    }
    sort(st.begin(), st.end());
    st.erase(unique(st.begin(), st.end()), st.end()); // unique
    ChargingInstance inst;
    inst.stations = st;
    inst.D = D;
    inst.R = R;
    return inst;
}

/*
Greedy algorithm for minimum #charges to reach destination:

We transform:
- positions: [0 (=start)] + stations + [D (=dest)]
- current fuel range = R initially
- We'll "drive" through the list of stops in order.

We'll keep a max-heap of "stations we passed but haven't charged at yet",
which represent potential charging opportunities.

Process:
1. Sort all stops.
2. currentPos = 0, currentReach = R (we can reach up to currentPos+R).
3. Iterate over each next stop in order:
   - While next stop is beyond currentReach,
        we MUST "retroactively charge" at one of the stations we passed.
        Pick the station that was farthest ahead (max-heap),
        update currentReach to stationPosition + R,
        increment charges.
        If heap empty => impossible.
   - Push this stop into the heap of candidate refuels.
4. At the end, we ensure we can reach D.

NOTE: In this simplified version, we treat each chosen refuel as if it
resets our reach to (that station position + R). We don't simulate battery
depletion continuously, we just track "farthest reachable mile so far"
which is the standard viewpoint for this greedy.

We will also do DP brute force for small N to confirm optimum.
*/

static int min_charges_greedy(const ChargingInstance &inst, bool &reachable) {
    // Build sorted list of all "stops" including start(0) and dest(D)
    vector<int> stops;
    stops.reserve(inst.stations.size()+2);
    stops.push_back(0);
    for (int x : inst.stations) stops.push_back(x);
    stops.push_back(inst.D);
    sort(stops.begin(), stops.end());

    // quick feasibility: if any gap > R, it's impossible no matter what
    for (size_t i=1; i<stops.size(); i++) {
        if (stops[i] - stops[i-1] > inst.R) {
            reachable = false;
            return INT_MAX;
        }
    }

    // max-heap of candidate stations to "charge at"
    priority_queue<int> pq;

    int charges = 0;
    int currReach = inst.R; // we start at mile 0 with full R
    int i = 1; // index of next stop in 'stops' we want to reach
    int lastStopUsed = 0; // mile marker of last committed charge (or 0 at start)

    // We also push 0 at the start (you can consider start as "charged")
    pq.push(0);

    while (i < (int)stops.size()) {
        int nextStop = stops[i];

        // If we cannot reach nextStop with current reach, we must refuel
        while (nextStop > currReach) {
            if (pq.empty()) {
                reachable = false;
                return INT_MAX;
            }
            int bestStation = pq.top(); pq.pop();
            // commit a charge at bestStation:
            charges++;
            currReach = bestStation + inst.R;
            if (bestStation == lastStopUsed && currReach < nextStop) {
                // safety break if stuck (shouldn't normally happen if feasible)
                break;
            }
            lastStopUsed = bestStation;
            if (currReach >= inst.D) break;
        }

        // If still we can't reach, means impossible
        if (nextStop > currReach) {
            reachable = false;
            return INT_MAX;
        }

        // station i can now be used later as a charging option
        pq.push(nextStop);
        i++;
    }

    reachable = true;
    // We don't count a "charge" at the destination because we don't need to drive after D.
    // charges already counted any real charges on the way.
    return charges;
}

/*
Brute force (for validation on small N):

We will do DP over stations:
State = index i of the current station you are physically at with full battery.
From there, you can jump to any station j>i where stops[j] - stops[i] <= R.
Cost of moving i->j is:
 - if j is the destination index, cost 0 (no need to recharge after arrival)
 - else cost 1 (because you must have recharged at station i to leave it full)
We want the min total extra charges to reach destination.

We solve with simple recursion + memo or BFS.
This is O(N^2), which is fine for N up to maybe 200.
*/

static int min_charges_bruteforce_small(const ChargingInstance &inst, bool &reachable) {
    vector<int> stops;
    stops.reserve(inst.stations.size()+2);
    stops.push_back(0);
    for (int x : inst.stations) stops.push_back(x);
    stops.push_back(inst.D);
    sort(stops.begin(), stops.end());

    int n = (int)stops.size();
    // Quick feasibility
    for (int i=1;i<n;i++){
        if (stops[i]-stops[i-1] > inst.R) {
            reachable = false;
            return INT_MAX;
        }
    }

    const int INF = 1e9;
    vector<int> dp(n, INF);
    dp[0] = 0; // Starting at 0, no charges yet.

    for (int i=0;i<n;i++){
        if (dp[i] == INF) continue;
        for (int j=i+1;j<n;j++){
            if (stops[j]-stops[i] <= inst.R){
                // cost to go i->j
                int cost = dp[i] + (j == n-1 ? 0 : 1);
                dp[j] = min(dp[j], cost);
            }
        }
    }

    reachable = (dp[n-1] != INF);
    return dp[n-1];
}

// run timing experiment for EV charging greedy
// writes ev_charging_times.csv with (n,avg_ms)
static void experiment_ev_greedy(RNG &rng,
                                 int minN, int maxN, int step, int trials,
                                 const string &outCSV = "ev_charging_times.csv")
{
    ofstream out(outCSV);
    out << "n,avg_ms\n";

    // correctness sanity check on a smaller instance
    {
        int smallN = 40;
        auto inst = make_random_charging_instance(rng, smallN, 2000, 300);
        bool r1=false, r2=false;
        int g = min_charges_greedy(inst, r1);
        int b = min_charges_bruteforce_small(inst, r2);
        if (!(r1==r2 && (!r1 || g==b))) {
            cerr << "[WARN] Greedy mismatch brute force on EV charging smallN="
                 << smallN << " (g="<<g<<", b="<<b<<", r1="<<r1<<", r2="<<r2<<")\n";
        } else {
            cerr << "[OK] Greedy matches brute force for EV charging at N="<<smallN<<"\n";
        }
    }

    for (int n=minN; n<=maxN; n+=step) {
        double total_ms = 0.0;
        for (int t=0;t<trials;t++) {
            auto inst = make_random_charging_instance(rng, n, 5000, 400);
            uint64_t t0 = now_us();
            bool ok=false;
            (void)min_charges_greedy(inst, ok);
            uint64_t t1 = now_us();
            total_ms += (t1 - t0)/1000.0;
        }
        double avg_ms = total_ms / trials;
        out << n << "," << avg_ms << "\n";
        cerr << "[EV-Greedy] n="<<n<<" avg_ms="<<avg_ms<<"\n";
    }
    cerr << "[EV-Greedy] wrote " << outCSV << "\n";
}


// ============================================================
// DIVIDE & CONQUER PROBLEM: closest pair of cell towers
// ============================================================

struct Pt {
    double x, y;
    int id;
};

static inline double dist2(const Pt &a, const Pt &b) {
    double dx = a.x - b.x, dy = a.y - b.y;
    return dx*dx + dy*dy;
}

// random tower coordinates in a square region
static vector<Pt> make_random_towers(RNG &rng, int n, double lim=100000.0) {
    vector<Pt> P; P.reserve(n);
    for (int i=0;i<n;i++){
        P.push_back({ rng.randreal(-lim,lim), rng.randreal(-lim,lim), i });
    }
    return P;
}

// brute force for ground truth / validation
static pair<double, pair<Pt,Pt>> closest_pair_bruteforce(const vector<Pt> &P) {
    double best = numeric_limits<double>::infinity();
    pair<Pt,Pt> ans;
    int n=(int)P.size();
    for (int i=0;i<n;i++){
        for (int j=i+1;j<n;j++){
            double d2 = dist2(P[i],P[j]);
            if (d2 < best) {
                best = d2;
                ans = {P[i],P[j]};
            }
        }
    }
    return { sqrt(best), ans };
}

// recursive helper for divide & conquer
static pair<double,pair<Pt,Pt>> closest_pair_rec(vector<Pt> &Px, vector<Pt> &Py) {
    int n=(int)Px.size();
    if (n<=3) {
        return closest_pair_bruteforce(Px);
    }

    int mid=n/2;
    double midx=Px[mid].x;

    vector<Pt> Lx(Px.begin(), Px.begin()+mid);
    vector<Pt> Rx(Px.begin()+mid, Px.end());
    vector<Pt> Ly, Ry;
    Ly.reserve(Lx.size());
    Ry.reserve(Rx.size());
    for (auto &p: Py) {
        if (p.x <= midx) Ly.push_back(p);
        else            Ry.push_back(p);
    }

    auto left  = closest_pair_rec(Lx, Ly);
    auto right = closest_pair_rec(Rx, Ry);
    auto best  = (left.first < right.first)? left : right;
    double d   = best.first;

    // build strip of points within d of mid line
    vector<Pt> strip;
    strip.reserve(n);
    for (auto &p: Py) {
        if (fabs(p.x - midx) < d) strip.push_back(p);
    }

    // check up to ~7 neighbors per point in strip, sorted by y
    for (int i=0;i<(int)strip.size();i++){
        for (int j=i+1; j<(int)strip.size() && (strip[j].y - strip[i].y) < d; j++){
            double dd = sqrt(dist2(strip[i],strip[j]));
            if (dd < best.first) {
                best = { dd, {strip[i],strip[j]} };
                d = dd;
            }
        }
    }

    return best;
}

// wrapper
static pair<double,pair<Pt,Pt>> closest_pair_divide_conquer(const vector<Pt> &P) {
    vector<Pt> Px = P, Py = P;
    sort(Px.begin(),Px.end(),[](auto&a,auto&b){
        if (a.x!=b.x) return a.x<b.x;
        return a.y<b.y;
    });
    sort(Py.begin(),Py.end(),[](auto&a,auto&b){
        if (a.y!=b.y) return a.y<b.y;
        return a.x<b.x;
    });
    return closest_pair_rec(Px, Py);
}

// experiment: compare brute force vs divide & conquer
// writes celltower_closestpair_times.csv with (n, brute_ms, dc_ms)
static void experiment_closest_pair(RNG &rng,
                                    int minN, int maxN, int step, int trials,
                                    const string &outCSV = "celltower_closestpair_times.csv")
{
    ofstream out(outCSV);
    out << "n,brute_ms,dc_ms\n";

    // correctness sanity check
    {
        int smallN = 200;
        auto P = make_random_towers(rng, smallN);
        auto bf = closest_pair_bruteforce(P);
        auto dc = closest_pair_divide_conquer(P);
        if (fabs(bf.first - dc.first) > 1e-9) {
            cerr << "[WARN] D&C mismatch brute force at n="<<smallN
                 << " brute="<<bf.first<<" dc="<<dc.first<<"\n";
        } else {
            cerr << "[OK] D&C matches brute force for towers at n="<<smallN<<"\n";
        }
    }

    for (int n=minN; n<=maxN; n+=step) {
        double brute_sum=0.0, dc_sum=0.0;
        for (int t=0;t<trials;t++){
            auto P = make_random_towers(rng, n);

            uint64_t b0=now_us();
            auto bf = closest_pair_bruteforce(P);
            uint64_t b1=now_us();
            (void)bf;
            brute_sum += (b1-b0)/1000.0;

            uint64_t d0=now_us();
            auto dc = closest_pair_divide_conquer(P);
            uint64_t d1=now_us();
            (void)dc;
            dc_sum += (d1-d0)/1000.0;
        }
        double brute_avg = brute_sum/trials;
        double dc_avg    = dc_sum/trials;
        out << n << "," << brute_avg << "," << dc_avg << "\n";
        cerr << "[Tower-Closest] n="<<n
             <<" brute_ms="<<brute_avg<<" dc_ms="<<dc_avg<<"\n";
    }
    cerr << "[Tower-Closest] wrote " << outCSV << "\n";
}


// ============================================================
// main()
// ============================================================

int main(int argc, char** argv) {
    // defaults for experiments
    uint64_t seed = 2025;
    int minN = 100, maxN = 2000, step = 100, trials = 5;
    bool runEV = true, runClosest = true;

    // basic CLI parsing
    for (int i=1;i<argc;i++){
        string a = argv[i];
        auto getInt=[&](int &x){ if (i+1<argc) x=stoi(argv[++i]); };
        auto get64=[&](uint64_t &x){ if (i+1<argc) x=stoull(argv[++i]); };
        if (a=="--seed") get64(seed);
        else if (a=="--min") getInt(minN);
        else if (a=="--max") getInt(maxN);
        else if (a=="--step") getInt(step);
        else if (a=="--trials") getInt(trials);
        else if (a=="--only" && i+1<argc){
            string w=argv[++i];
            if (w=="ev"){ runEV=true; runClosest=false; }
            else if (w=="closest"){ runEV=false; runClosest=true; }
        }
    }

    RNG rng(seed);

    cout << "=== EV Truck Charging Planning (Greedy, O(n log n)) ===\n";
    cout << "Goal: reach destination with MINIMUM charging stops.\n";
    cout << "Greedy: delay charging until necessary, then pick best past station.\n\n";

    cout << "=== Cell Tower Closest-Pair (Divide & Conquer, O(n log n)) ===\n";
    cout << "Goal: detect dangerously close tower sites to avoid interference.\n";
    cout << "D&C: split map, solve halves, scan thin strip.\n\n";

    if (runEV)      experiment_ev_greedy(rng, minN, maxN, step, trials);
    if (runClosest) experiment_closest_pair(rng, minN, maxN, step, trials);

    cout << "Generated:\n";
    if (runEV)      cout << " - ev_charging_times.csv\n";
    if (runClosest) cout << " - celltower_closestpair_times.csv\n";
    cout << "Attach these CSVs in Overleaf for plots in the report.\n";
    return 0;
}
