"""
Reward-Biased RRT for the Orienteering Problem  (vectorised, fast version)
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.colors import Normalize
from matplotlib.cm import ScalarMappable
from dataclasses import dataclass, field
from typing import Optional, List, Tuple, FrozenSet
import random, time


# ══════════════════════════════════════════════════════════════════
# Data Structures
# ══════════════════════════════════════════════════════════════════

@dataclass
class POI:
    position: np.ndarray
    reward: float
    label: str = ""


class Node:
    __slots__ = ("position","cost","reward","parent","collected","_id")
    _counter = 0
    def __init__(self, position, cost=0.0, reward=0.0, parent=None, collected=None):
        self.position  = position
        self.cost      = cost
        self.reward    = reward
        self.parent    = parent
        self.collected = collected if collected is not None else frozenset()
        self._id       = Node._counter; Node._counter += 1
    def __hash__(self):  return self._id
    def __eq__(self, o): return self is o


# ══════════════════════════════════════════════════════════════════
# Planner
# ══════════════════════════════════════════════════════════════════

class RewardBiasedRRT:
    def __init__(
        self,
        bounds,           # (xmin,xmax,ymin,ymax)
        depot,
        pois: List[POI],
        budget: float,
        *,
        max_iter=5000,
        step_size=1.2,
        collect_radius=0.9,
        alpha=2.5,            # reward exponent for POI-biased sampling
        beta=1.0,             # reward weight in NN metric
        poi_sample_rate=0.45,
        rrt_star=True,
        rewire_radius=3.0,
        seed=42,
    ):
        self.xmin,self.xmax,self.ymin,self.ymax = bounds
        self.depot  = depot.copy()
        self.pois   = pois
        self.budget = budget
        self.max_iter = max_iter
        self.step_size = step_size
        self.collect_radius = collect_radius
        self.alpha  = alpha
        self.beta   = beta
        self.poi_sample_rate = poi_sample_rate
        self.rrt_star = rrt_star
        self.rewire_radius = rewire_radius

        np.random.seed(seed); random.seed(seed)

        N = len(pois)
        self._poi_pos  = np.array([p.position for p in pois]) if N else np.empty((0,2))
        self._poi_rew  = np.array([p.reward   for p in pois]) if N else np.empty(0)
        self._total_rew = float(self._poi_rew.sum()) if N else 1.0

        # ── Vectorised node arrays (grow dynamically) ──────────────
        INIT = 512
        self._pos_arr = np.zeros((INIT, 2))   # positions
        self._rew_arr = np.zeros(INIT)         # accumulated rewards
        self._cost_arr= np.zeros(INIT)         # accumulated costs
        self._sz = 0                           # current count

        root = Node(self.depot.copy())
        self.nodes: List[Node] = [root]
        self._add_to_arrays(root)

        self.best_reward = 0.0
        self.best_tour: List[Node] = []
        self.history: List[Tuple[int,float]] = []
        self.n_pruned = 0

    def _add_to_arrays(self, node: Node):
        if self._sz >= len(self._pos_arr):
            cap = len(self._pos_arr) * 2
            self._pos_arr  = np.resize(self._pos_arr,  (cap,2))
            self._rew_arr  = np.resize(self._rew_arr,  cap)
            self._cost_arr = np.resize(self._cost_arr, cap)
        self._pos_arr [self._sz] = node.position
        self._rew_arr [self._sz] = node.reward
        self._cost_arr[self._sz] = node.cost
        self._sz += 1

    # ── Sampling ──────────────────────────────────────────────────

    def _sample(self):
        if self._poi_rew.size > 0 and random.random() < self.poi_sample_rate:
            w = self._poi_rew ** self.alpha; w /= w.sum()
            idx = np.random.choice(len(self.pois), p=w)
            return self._poi_pos[idx] + np.random.randn(2) * 0.4
        return np.array([np.random.uniform(self.xmin, self.xmax),
                         np.random.uniform(self.ymin, self.ymax)])

    # ── Nearest Neighbour (vectorised) ────────────────────────────

    def _nearest_idx(self, x_rand):
        pos  = self._pos_arr[:self._sz]
        rew  = self._rew_arr[:self._sz]
        diff = pos - x_rand
        dists = np.sqrt((diff*diff).sum(axis=1))
        adj   = dists / (1.0 + self.beta * rew / self._total_rew)
        return int(np.argmin(adj))

    # ── Steer ─────────────────────────────────────────────────────

    def _steer(self, frm, to):
        d = to - frm; dist = np.linalg.norm(d)
        if dist < 1e-9: return frm.copy()
        return frm + d/dist * min(dist, self.step_size)

    # ── POI collection ────────────────────────────────────────────

    def _collect(self, pos, already: frozenset):
        if self._poi_pos.size == 0: return already, 0.0
        dists = np.linalg.norm(self._poi_pos - pos, axis=1)
        new_set = set(already); added = 0.0
        for i in np.where(dists <= self.collect_radius)[0]:
            if i not in already:
                new_set.add(int(i)); added += float(self._poi_rew[i])
        return frozenset(new_set), added

    # ── Budget ────────────────────────────────────────────────────

    def _ret(self, pos): return float(np.linalg.norm(pos - self.depot))

    def _feasible(self, parent: Node, new_pos):
        return parent.cost + np.linalg.norm(new_pos - parent.position) + self._ret(new_pos) <= self.budget

    # ── Near nodes (vectorised) ───────────────────────────────────

    def _near_indices(self, pos):
        diff  = self._pos_arr[:self._sz] - pos
        dists = np.sqrt((diff*diff).sum(axis=1))
        return np.where(dists <= self.rewire_radius)[0].tolist()

    # ── RRT* rewiring ─────────────────────────────────────────────

    def _rewire(self, new_node: Node):
        for idx in self._near_indices(new_node.position):
            nb = self.nodes[idx]
            if nb is new_node or nb is new_node.parent: continue
            e = float(np.linalg.norm(nb.position - new_node.position))
            c = new_node.cost + e
            if c < nb.cost and c + self._ret(nb.position) <= self.budget:
                col2, added2 = self._collect(nb.position, new_node.collected)
                nb.parent = new_node; nb.cost = c
                nb.reward = new_node.reward + added2; nb.collected = col2
                # sync arrays
                self._cost_arr[idx] = c
                self._rew_arr [idx] = nb.reward

    # ── Tour closure ──────────────────────────────────────────────

    def _try_close(self, node: Node, it: int):
        if node.cost + self._ret(node.position) <= self.budget and node.reward > self.best_reward:
            self.best_reward = node.reward
            path, cur = [], node
            while cur: path.append(cur); cur = cur.parent
            self.best_tour = list(reversed(path))
            self.history.append((it, self.best_reward))

    # ── Main loop ─────────────────────────────────────────────────

    def plan(self):
        for i in range(self.max_iter):
            x_rand = self._sample()
            ni     = self._nearest_idx(x_rand)
            x_near = self.nodes[ni]
            x_new_pos = self._steer(x_near.position, x_rand)

            if not self._feasible(x_near, x_new_pos):
                self.n_pruned += 1; continue

            e   = float(np.linalg.norm(x_new_pos - x_near.position))
            col, added = self._collect(x_new_pos, x_near.collected)
            new_node   = Node(x_new_pos,
                              cost=x_near.cost + e,
                              reward=x_near.reward + added,
                              parent=x_near,
                              collected=col)

            # RRT*: best parent
            if self.rrt_star:
                for idx2 in self._near_indices(x_new_pos):
                    nb = self.nodes[idx2]
                    e2 = float(np.linalg.norm(x_new_pos - nb.position))
                    c2 = nb.cost + e2
                    if c2 < new_node.cost and c2 + self._ret(x_new_pos) <= self.budget:
                        col2, added2 = self._collect(x_new_pos, nb.collected)
                        new_node.parent = nb; new_node.cost = c2
                        new_node.reward = nb.reward + added2; new_node.collected = col2

            self.nodes.append(new_node)
            self._add_to_arrays(new_node)

            if self.rrt_star:
                self._rewire(new_node)

            self._try_close(new_node, i)

        return self.best_tour


# ══════════════════════════════════════════════════════════════════
# Visualisation
# ══════════════════════════════════════════════════════════════════

def visualize(planner: RewardBiasedRRT, title="Reward-Biased RRT — Orienteering"):
    fig, axes = plt.subplots(1, 2, figsize=(17, 7))
    fig.patch.set_facecolor("#0f1117")

    ax = axes[0]; ax.set_facecolor("#0f1117")

    # Tree edges (subsample if huge)
    nodes = planner.nodes
    step = max(1, len(nodes)//4000)
    for node in nodes[::step]:
        if node.parent is not None:
            ax.plot([node.parent.position[0], node.position[0]],
                    [node.parent.position[1], node.position[1]],
                    color="#1a3a5c", lw=0.4, alpha=0.55, zorder=1)

    # POIs
    rew = np.array([p.reward for p in planner.pois])
    norm_r = Normalize(vmin=rew.min(), vmax=rew.max())
    cmap = plt.cm.plasma
    best_col = planner.best_tour[-1].collected if planner.best_tour else frozenset()

    for i, poi in enumerate(planner.pois):
        c = cmap(norm_r(poi.reward)); hit = i in best_col
        ax.scatter(*poi.position, color=c, s=240 if hit else 70,
                   marker="*" if hit else "o",
                   edgecolors="white" if hit else "#555", lw=1.3 if hit else 0.4, zorder=4)
        ax.text(poi.position[0]+0.2, poi.position[1]+0.2, f"{poi.reward:.0f}",
                color="white", fontsize=7, zorder=5)

    # Best tour
    if planner.best_tour:
        pts = [n.position for n in planner.best_tour] + [planner.depot]
        xs, ys = zip(*pts)
        ax.plot(xs, ys, color="#00e5ff", lw=2.5, zorder=5,
                label=f"Best tour  R = {planner.best_reward:.1f}")
        for k in range(len(pts)-1):
            ax.annotate("", xy=pts[k+1], xytext=pts[k],
                        arrowprops=dict(arrowstyle="->",color="#00e5ff",lw=1.8), zorder=6)

    # Depot
    ax.scatter(*planner.depot, color="#00ff88", s=300, marker="s", zorder=7)
    ax.text(planner.depot[0]+.25, planner.depot[1]+.25, "Depot",
            color="#00ff88", fontsize=9, fontweight="bold", zorder=8)

    sm = ScalarMappable(cmap=cmap, norm=norm_r); sm.set_array([])
    cb = plt.colorbar(sm, ax=ax, shrink=0.65, pad=0.02)
    cb.set_label("POI Reward", color="white")
    cb.ax.yaxis.set_tick_params(color="white")
    plt.setp(cb.ax.yaxis.get_ticklabels(), color="white")

    ax.set_xlim(planner.xmin, planner.xmax); ax.set_ylim(planner.ymin, planner.ymax)
    ax.set_title(title, color="white", fontsize=12, pad=10)
    ax.legend(loc="upper left", facecolor="#1a1a2e", labelcolor="white", fontsize=9)
    ax.tick_params(colors="white")
    for sp in ax.spines.values(): sp.set_edgecolor("#333")
    ax.set_aspect("equal"); ax.grid(True, color="#1a2a3a", lw=0.5)

    # Stats
    n_col = len(best_col); n_tot = len(planner.pois)
    max_r = sum(p.reward for p in planner.pois)
    if planner.best_tour:
        last = planner.best_tour[-1]
        used = last.cost + planner._ret(last.position)
        stats = (f"Tree nodes  : {len(planner.nodes)}\n"
                 f"Pruned      : {planner.n_pruned}\n"
                 f"POIs        : {n_col}/{n_tot}\n"
                 f"Reward      : {planner.best_reward:.1f}/{max_r:.1f}\n"
                 f"Budget used : {used:.2f}/{planner.budget:.1f}")
    else:
        stats = "No feasible tour found."
    ax.text(0.02, 0.02, stats, transform=ax.transAxes,
            color="white", fontsize=8, va="bottom",
            bbox=dict(facecolor="#1a1a2e", edgecolor="#334", alpha=0.85, pad=5))

    # Convergence plot
    ax2 = axes[1]; ax2.set_facecolor("#0f1117")
    if planner.history:
        its, rws = zip(*planner.history)
        ax2.step(its, rws, where="post", color="#00e5ff", lw=2)
        ax2.fill_between(its, rws, step="post", color="#00e5ff", alpha=0.15)
        max_r_tot = sum(p.reward for p in planner.pois)
        ax2.axhline(max_r_tot, color="#ff6b6b", lw=1.2, ls="--",
                    label=f"Max possible = {max_r_tot:.0f}")
        ax2.axhline(planner.best_reward, color="#00ff88", lw=1.0, ls=":",
                    label=f"Best found = {planner.best_reward:.1f}")
        ax2.legend(facecolor="#1a1a2e", labelcolor="white", fontsize=9)
    ax2.set_xlabel("Iteration", color="white")
    ax2.set_ylabel("Best Tour Reward", color="white")
    ax2.set_title("Convergence", color="white", fontsize=12, pad=10)
    ax2.tick_params(colors="white")
    for sp in ax2.spines.values(): sp.set_edgecolor("#333")
    ax2.grid(True, color="#1a2a3a", lw=0.5)

    plt.tight_layout(pad=1.5)
    return fig


# ══════════════════════════════════════════════════════════════════
# Scenarios
# ══════════════════════════════════════════════════════════════════

def make_random(n=25, seed=0):
    rng = np.random.default_rng(seed)
    pois = [POI(rng.uniform(0,20,2), float(rng.uniform(5,50))) for _ in range(n)]
    return dict(bounds=(0,20,0,20), depot=np.array([10.,10.]), pois=pois,
                budget=25., title=f"Random ({n} POIs)")

def make_clustered(seed=1):
    rng = np.random.default_rng(seed)
    clusters = [
        (np.array([ 3., 3.]),  [40,35,30,20]),
        (np.array([16., 3.]),  [10, 8, 7, 5]),
        (np.array([ 3.,17.]),  [15,12,10]),
        (np.array([16.,17.]),  [50,45,40,35,30]),
    ]
    pois = []
    for ctr, rews in clusters:
        for r in rews:
            pois.append(POI(ctr + rng.normal(0,.8,2), float(r)))
    return dict(bounds=(0,20,0,20), depot=np.array([10.,10.]), pois=pois,
                budget=30., title="Clustered (high reward top-right)")

def make_corridor(seed=2):
    rng = np.random.default_rng(seed)
    pois = []
    for x in np.linspace(1,19,12):
        r = 10 + 40*np.exp(-((x-15)**2)/8)
        pois.append(POI(np.array([x,10.])+rng.normal(0,.3,2), float(r)))
    for _ in range(8):
        pois.append(POI(rng.uniform(0,20,2), float(rng.uniform(3,8))))
    return dict(bounds=(0,20,0,20), depot=np.array([1.,10.]), pois=pois,
                budget=22., title="Corridor (reward peaks far right)")


# ══════════════════════════════════════════════════════════════════
# Main
# ══════════════════════════════════════════════════════════════════

def run(scenario, **kw):
    t0 = time.perf_counter()
    p = RewardBiasedRRT(scenario['bounds'], scenario['depot'],
                        scenario['pois'],   scenario['budget'], **kw)
    p.plan()
    elapsed = time.perf_counter()-t0
    last = p.best_tour[-1] if p.best_tour else None
    used = (last.cost + p._ret(last.position)) if last else 0
    max_r = sum(poi.reward for poi in scenario['pois'])
    print(f"  {scenario['title']:<38}  "
          f"reward={p.best_reward:6.1f}/{max_r:.0f}  "
          f"POIs={len(last.collected) if last else 0}/{len(scenario['pois'])}  "
          f"budget_used={used:.1f}/{scenario['budget']}  "
          f"nodes={len(p.nodes)}  t={elapsed:.2f}s")
    return p

if __name__ == "__main__":
    KW = dict(max_iter=5000, step_size=1.2, collect_radius=0.9,
              alpha=2.5, beta=1.0, poi_sample_rate=0.45,
              rrt_star=True, rewire_radius=3.0, seed=42)

    print("\n=== Scenario Results ===")
    scenarios = [make_random(), make_clustered(), make_corridor()]
    planners  = [run(s, **KW) for s in scenarios]

    print("\n=== Alpha Ablation (random scenario) ===")
    base = make_random()
    alphas, abl = [0.0, 1.0, 2.5, 5.0], {}
    for a in alphas:
        p = run({**base, 'title': f'alpha={a}'}, **{**KW,'alpha':a,'max_iter':3000})
        abl[a] = p.best_reward

    # ── Save figures ────────────────────────────────────────
    import matplotlib; matplotlib.use('Agg')
    for idx,(p,s) in enumerate(zip(planners,scenarios)):
        fig = visualize(p, title=s['title'])
        fig.savefig(f'./outputs/rrt_op_fig{idx}.png',
                    dpi=130, bbox_inches='tight', facecolor=fig.get_facecolor())
        plt.close(fig)

    fig_abl, ax = plt.subplots(figsize=(7,4))
    fig_abl.patch.set_facecolor('#0f1117'); ax.set_facecolor('#0f1117')
    cols = ['#1f4e79','#2e75b6','#00e5ff','#ff6b6b']
    bars = ax.bar([str(a) for a in alphas],[abl[a] for a in alphas],color=cols)
    ax.set_xlabel('alpha',color='white'); ax.set_ylabel('Best Reward',color='white')
    ax.set_title('Effect of Reward Bias (α)',color='white',fontsize=12)
    ax.tick_params(colors='white')
    for sp in ax.spines.values(): sp.set_edgecolor('#333')
    ax.grid(axis='y',color='#1a2a3a')
    for bar,a in zip(bars,alphas):
        ax.text(bar.get_x()+bar.get_width()/2, bar.get_height()+.5,
                f'{abl[a]:.1f}', ha='center', color='white', fontsize=9)
    plt.tight_layout()
    fig_abl.savefig('./outputs/rrt_op_fig3.png',
                    dpi=130, bbox_inches='tight', facecolor=fig_abl.get_facecolor())
    plt.close()
    print("\nFigures saved to /mnt/user-data/outputs/")
