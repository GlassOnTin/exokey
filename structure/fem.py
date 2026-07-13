"""A 3D FRAME SOLVER THAT FACTORISES ONCE AND SOLVES MANY LOAD CASES.

PyNite is correct -- it is checked to 0.000% against the closed-form cantilever and it stays the
reference -- but it rebuilds and re-factorises the stiffness matrix for every single solve. That
was affordable while there was ONE load case. There is not:

  * A WELL IS A FIVE-DIRECTION JOYSTICK. A digit can push it down, forward, back, left or right,
    and each is a different force on the structure. 15 of the 25 (digit, direction) pairs are
    wired to characters, and a typist presses ONE AT A TIME.
  * The structure had been grown against ALL FIVE DIGITS PRESSING SIMULTANEOUSLY -- a load case
    that never occurs. Re-solved one digit at a time, the thumb alone deflects 522 um against a
    500 um gate. ESO had optimised the case that does not happen and failed the case that does.

So the load set is ~15 cases, not one, and ESO needs the WORST of them at every step. With PyNite
that is 15 factorisations per step; here it is ONE factorisation and 15 back-substitutions, which
is the entire point of a direct sparse solver and is what makes the co-optimisation affordable at
all.

Euler-Bernoulli 3D frame, 6 DOF/node, circular section (so Iy = Iz and the element's roll about
its own axis does not matter -- no orientation vector to get wrong).

⚠ IT IS VALIDATED AGAINST PyNite ON THE REAL LATTICE, not just on a textbook beam. A solver that
agrees with the closed form on a cantilever and disagrees on the structure you actually care about
is a solver you have not tested.
"""
from __future__ import annotations

import numpy as np
from scipy.sparse import coo_matrix
from scipy.sparse.linalg import splu


def _element_k(L, E, G, A, I, J):
    """The 12x12 local stiffness of one Euler-Bernoulli frame element."""
    k = np.zeros((12, 12))
    k[0, 0] = k[6, 6] = E * A / L
    k[0, 6] = k[6, 0] = -E * A / L
    k[3, 3] = k[9, 9] = G * J / L
    k[3, 9] = k[9, 3] = -G * J / L

    a, b, c, d = 12 * E * I / L ** 3, 6 * E * I / L ** 2, 4 * E * I / L, 2 * E * I / L
    # bending in the local x-y plane: v (1, 7) and rz (5, 11)
    for (i, j, v) in ((1, 1, a), (1, 5, b), (1, 7, -a), (1, 11, b),
                      (5, 5, c), (5, 7, -b), (5, 11, d),
                      (7, 7, a), (7, 11, -b), (11, 11, c)):
        k[i, j] = k[j, i] = v
    # bending in the local x-z plane: w (2, 8) and ry (4, 10) -- signs flip
    for (i, j, v) in ((2, 2, a), (2, 4, -b), (2, 8, -a), (2, 10, -b),
                      (4, 4, c), (4, 8, b), (4, 10, d),
                      (8, 8, a), (8, 10, b), (10, 10, c)):
        k[i, j] = k[j, i] = v
    return k


def _cst_k(p0, p1, p2, E, nu, t):
    """A CONSTANT-STRAIN TRIANGLE: 9x9, in-plane (membrane) stiffness, in global coords.

    ⚠ MEMBRANE IS THE THING A BEAM CANNOT DO, AND IT IS WHY PLATES BEAT STRUTS.
    A lattice of rods carries load axially and in bending. A SHEET carries it in-plane, in two
    directions at once, and in-plane stiffness scales with THICKNESS while bending stiffness
    scales with thickness CUBED -- so for thin material the sheet wins on membrane by a mile. This
    project has already paid for that lesson once: modelling the palm arch as beams got its mass
    wrong by 25x, because beams cannot carry the membrane action a curved shell carries for free.

    So the ground structure offers BOTH, and ESO decides. This element supplies only the membrane
    part; the shell's BENDING is already there, carried by the two node sheets and the struts that
    brace between them (which is what a sandwich panel is).
    """
    e1 = p1 - p0
    e2 = p2 - p0
    n = np.cross(e1, e2)
    A = 0.5 * float(np.linalg.norm(n))
    if A < 1e-12:
        return None
    n /= np.linalg.norm(n)
    ex = e1 / np.linalg.norm(e1)
    ey = np.cross(n, ex)
    R = np.vstack([ex, ey])                       # (2,3): world -> the triangle's own plane

    x = np.array([[0.0, 0.0], R @ e1, R @ e2])    # local 2-D coords
    b = np.array([x[1, 1] - x[2, 1], x[2, 1] - x[0, 1], x[0, 1] - x[1, 1]])
    c = np.array([x[2, 0] - x[1, 0], x[0, 0] - x[2, 0], x[1, 0] - x[0, 0]])
    B = np.zeros((3, 6))
    B[0, 0::2] = b
    B[1, 1::2] = c
    B[2, 0::2] = c
    B[2, 1::2] = b
    B /= 2.0 * A

    D = (E / (1 - nu ** 2)) * np.array([[1, nu, 0], [nu, 1, 0], [0, 0, (1 - nu) / 2]])
    k2 = t * A * (B.T @ D @ B)                    # 6x6 in the plane

    T = np.zeros((6, 9))                          # plane dofs <- global translations
    for i in range(3):
        T[2 * i:2 * i + 2, 3 * i:3 * i + 3] = R
    return T.T @ k2 @ T, A, B, D, T


class Frame:
    """Assemble once, factorise once, then solve any number of load cases cheaply.

    Carries BARS (3-D frame elements) and, optionally, SHELLS (constant-strain membrane triangles)
    in the same stiffness matrix, so ESO can rank a strut against a plate on the same footing.
    """

    def __init__(self, nodes, bars, E, G, A, I, J, spring=None, fixed=(),
                 shells=(), shell_t=0.0006, nu=0.3):
        self.nodes = np.asarray(nodes, float)
        self.bars = [tuple(b) for b in bars]
        # the node map must cover SHELLS too, or a triangle whose corner touches no bar loses it
        used = sorted({i for b in self.bars for i in b} | {i for s_ in shells for i in s_})
        self.idx = {n: k for k, n in enumerate(used)}
        self.used = used
        n = len(used)
        self.ndof = 6 * n

        rows, cols, vals = [], [], []
        kloc, Ts, Ls, dofs_all = [], [], [], []
        for (i, j) in self.bars:
            pi, pj = self.nodes[i], self.nodes[j]
            v = pj - pi
            L = float(np.linalg.norm(v))
            ex = v / L
            ref = np.array([0.0, 0.0, 1.0])
            if abs(ex @ ref) > 0.99:
                ref = np.array([0.0, 1.0, 0.0])
            ez = np.cross(ex, ref)
            ez /= np.linalg.norm(ez)
            ey = np.cross(ez, ex)
            R = np.vstack([ex, ey, ez])                    # local axes as rows
            T = np.zeros((12, 12))
            for b in range(4):
                T[3 * b:3 * b + 3, 3 * b:3 * b + 3] = R
            kl = _element_k(L, E, G, A, I, J)
            kg = T.T @ kl @ T
            kloc.append(kl)
            Ts.append(T)
            Ls.append(L)

            dofs = np.array([6 * self.idx[i] + d for d in range(6)]
                            + [6 * self.idx[j] + d for d in range(6)])
            dofs_all.append(dofs)
            rr, cc = np.meshgrid(dofs, dofs, indexing="ij")
            rows.append(rr.ravel())
            cols.append(cc.ravel())
            vals.append(kg.ravel())

        self.kloc = np.array(kloc)          # (nbar, 12, 12)
        self.T = np.array(Ts)               # (nbar, 12, 12)
        self.L = np.array(Ls)               # (nbar,)
        self.dofs = np.array(dofs_all)      # (nbar, 12)

        # SHELLS: membrane triangles, sharing the same nodes as the bars.
        self.shells = [tuple(s_) for s_ in shells]
        self.shell_t = shell_t
        self.sk, self.sA, self.sdofs = [], [], []
        for (i, j, k) in self.shells:
            out = _cst_k(self.nodes[i], self.nodes[j], self.nodes[k], E, nu, shell_t)
            if out is None:
                self.sk.append(np.zeros((9, 9)))
                self.sA.append(0.0)
                self.sdofs.append(np.zeros(9, int))
                continue
            kg, Ael, _B, _D, _T = out
            # a CST has TRANSLATIONS only; its 3 nodes' rotational dofs are left to the bars
            dd = np.concatenate([[6 * self.idx[m] + d for d in range(3)] for m in (i, j, k)])
            self.sk.append(kg)
            self.sA.append(Ael)
            self.sdofs.append(dd)
            rr, cc = np.meshgrid(dd, dd, indexing="ij")
            rows.append(rr.ravel())
            cols.append(cc.ravel())
            vals.append(kg.ravel())
        self.sk = np.array(self.sk) if self.sk else np.zeros((0, 9, 9))
        self.sA = np.array(self.sA) if len(self.sA) else np.zeros(0)
        self.sdofs = np.array(self.sdofs) if len(self.sdofs) else np.zeros((0, 9), int)

        self.rows = np.concatenate(rows)
        self.cols = np.concatenate(cols)
        self.vals = np.concatenate(vals)
        self.fixed = set(fixed)
        self.lu = None
        self.factorise(spring or {})

    def factorise(self, spring: dict):
        """spring: node -> translational stiffness (N/m), added to the three translation DOFs."""
        r, c, v = [self.rows], [self.cols], [self.vals]
        sr, sv = [], []
        for i, k in spring.items():
            if i in self.idx:
                for d in range(3):
                    sr.append(6 * self.idx[i] + d)
                    sv.append(k)
        # ⚠ A SPRING RESTRAINS TRANSLATION ONLY. It does NOT stop a node ROTATING, and a
        # single-bar cantilever on a "stiff" 1e12 spring therefore PIVOTS: 19.6 METRES of tip
        # deflection where the closed form says 0.86 mm. In the lattice this never bites (every
        # node is held in rotation by its own bars, which is why it agrees with PyNite to 0.01%),
        # but a solver whose support model can be silently wrong by 7 orders of magnitude needs
        # the clamp to be a separate, explicit thing.
        for i in self.fixed:
            if i in self.idx:
                for d in range(6):
                    sr.append(6 * self.idx[i] + d)
                    sv.append(1e14)
        if sr:
            r.append(np.array(sr))
            c.append(np.array(sr))
            v.append(np.array(sv))
        K = coo_matrix((np.concatenate(v), (np.concatenate(r), np.concatenate(c))),
                       shape=(self.ndof, self.ndof)).tocsc()
        # ⚠ A LATTICE HAS SOFT MODES. A tiny diagonal keeps the factorisation from blowing up on
        # them; it is 1e-9 of the smallest real stiffness here, so it cannot carry load. It is a
        # numerical floor, NOT a support -- if a structure needs it, `solve` returns garbage that
        # the caller's own checks (deflection gate, connectivity) will catch.
        K = K + 1e-6 * coo_matrix((np.ones(self.ndof),
                                   (np.arange(self.ndof), np.arange(self.ndof))),
                                  shape=(self.ndof, self.ndof)).tocsc()
        # ⚠ ONE FACTORISATION AT A TIME, AND DROP THE OLD ONE FIRST.
        # An splu of a 14k-DOF frame is hundreds of MB of fill-in. Caching one per anchor
        # active-set -- 25 load cases, each potentially lifting a different part of the patch --
        # exhausted system memory and the run was OOM-killed. The factorisation is cheap (0.8 s);
        # KEEPING it is what is expensive. Recompute, never hoard.
        self.lu = None
        self.lu = splu(K)
        return self

    def solve(self, cases):
        """cases: list of {node: force vector}. Returns displacements, shape (ncase, nnode, 6)."""
        B = np.zeros((self.ndof, len(cases)))
        for c, load in enumerate(cases):
            for i, f in load.items():
                if i in self.idx:
                    B[6 * self.idx[i]:6 * self.idx[i] + 3, c] = f
        U = self.lu.solve(B)
        return U.T.reshape(len(cases), len(self.used), 6)

    def disp(self, U, node):
        """The translation of one node, per load case. U from solve()."""
        return U[:, self.idx[node], :3]

    def strain_energy(self, U):
        """Per-element strain energy density (per unit length), summed over load cases.

        The ESO criterion is energy per unit VOLUME, not energy: a long bar stores more at the
        same stress, so ranking on raw energy deletes the short highly-stressed struts first --
        exactly backwards.

        VECTORISED. As a Python loop this was 12x12 matmuls one at a time -- 14444 bars x 25 load
        cases = 361,000 of them -- and it, not the linear algebra, was the whole cost of a solve.
        """
        Uf = U.reshape(U.shape[0], -1)               # (ncase, ndof)
        ue = Uf[:, self.dofs]                        # (ncase, nbar, 12)
        ul = np.einsum("bij,cbj->cbi", self.T, ue)
        e = 0.5 * np.einsum("cbi,bij,cbj->b", ul, self.kloc, ul)
        return e / self.L

    def shell_energy(self, U):
        """Per-shell strain energy density, summed over load cases. Same currency as the bars'.

        THE ESO CRITERION MUST BE ENERGY PER UNIT VOLUME, or a strut and a plate cannot be ranked
        against each other at all -- they have different shapes and wildly different volumes. A
        bar's volume is A*L; a shell's is area*t. Rank on raw energy and every plate looks precious
        simply for being big.
        """
        if not len(self.shells):
            return np.zeros(0)
        Uf = U.reshape(U.shape[0], -1)
        ue = Uf[:, self.sdofs]                             # (ncase, nshell, 9)
        e = 0.5 * np.einsum("csi,sij,csj->s", ue, self.sk, ue)
        return e / np.maximum(self.sA * self.shell_t, 1e-12)

    def shell_mass(self, rho, live=None):
        A = self.sA if live is None else self.sA[np.asarray(live, int)]
        return float(rho * self.shell_t * A.sum())

    def stress(self, U, r):
        """Peak von-Mises-ish stress in each bar: axial + bending at the extreme fibre.

        sigma = |N|/A + |M|*r/I, taking the worse end and the worst load case. The old beam frame
        found yield never binds -- but that was an 8x2 mm aluminium strip, not a 1.8 mm rod, and
        an assumption that held for one structure is not evidence about another.
        """
        A = np.pi * r ** 2
        I = np.pi * r ** 4 / 4
        Uf = U.reshape(U.shape[0], -1)
        ue = Uf[:, self.dofs]
        ul = np.einsum("bij,cbj->cbi", self.T, ue)
        f = np.einsum("bij,cbj->cbi", self.kloc, ul)          # (ncase, nbar, 12) end forces
        N = np.abs(f[:, :, 0])
        M = np.maximum(np.hypot(f[:, :, 4], f[:, :, 5]), np.hypot(f[:, :, 10], f[:, :, 11]))
        return (N / A + M * r / I).max(axis=0)

    def axial(self, U):
        """Axial force in each bar, per load case. (ncase, nbar). Tension positive."""
        Uf = U.reshape(U.shape[0], -1)
        ul = np.einsum("bij,cbj->cbi", self.T, Uf[:, self.dofs])
        f = np.einsum("bij,cbj->cbi", self.kloc, ul)
        return -f[:, :, 0]                # local x at end i points i->j; sign so tension is +

    def mass(self, A, rho, live=None):
        L = self.L if live is None else self.L[np.asarray(live, int)]
        return float(A * rho * L.sum())
