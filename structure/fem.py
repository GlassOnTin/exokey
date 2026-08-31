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

# CHOLMOD, when available. The stiffness matrix is symmetric positive-definite by construction
# (the 1e-6 diagonal floor below guarantees it), and profiling one evaluate() put 74% of its
# 19.6 s in sparse-LU factorization -- 1,294 factorizations per design, re-done from scratch each
# ESO deletion x load case x hand. CHOLMOD's Cholesky factors the same matrices 4-4.8x faster
# (measured on the captured K's: 420 -> 87 ms for factor + 25-RHS solve over 8 of them).
# scipy stays as the fallback: CHOLMOD requires strict PD and will reject what splu limps through.
try:
    import warnings as _warnings

    from scipy.sparse import csc_array as _csc_array
    from sksparse.cholmod import (
        CholmodError as _CholmodError,
        CholmodWarning as _CholmodWarning,
        cho_factor as _cho_factor,
    )

    # K is nearly singular BY DESIGN (1e14 clamp springs against a 1e-6 diagonal floor gives
    # rcond ~1e-14), so CHOLMOD warns on every solve -- and each warning has a fresh rcond in
    # its text, defeating the once-per-site dedup and flooding run logs (434 lines in the first
    # 3 generations). The accuracy question is already answered against splu on those exact
    # matrices: F bit-identical, max dG ~1e-13. The warning carries no news; drop it.
    _warnings.filterwarnings("ignore", category=_CholmodWarning)
except ImportError:                                     # scikit-sparse not installed: pure scipy
    _cho_factor = None


def _element_k(L, E, G, A, Iy, Iz=None, J=None):
    """The 12x12 local stiffness of one Euler-Bernoulli frame element.

    ⚠ Iy AND Iz ARE SEPARATE, AND THAT IS THE WHOLE POINT OF A NON-CIRCULAR SECTION.

    THE USER: "I think the thickness of struts should be a spline too, with a major and minor radius,
    and principal orientation as a spline."

    A CIRCLE IS THE WORST POSSIBLE SECTION FOR A MEMBER THAT BENDS IN ONE PLANE. For an ellipse of
    semi-axes a, b:  A = pi*a*b, and I = pi*a*b^3/4 about one axis and pi*a^3*b/4 about the other.
    So AT CONSTANT AREA -- at constant MASS -- you can move stiffness out of the direction nothing is
    pushing and into the direction that is being bent. A 2:1 ellipse is 4x stiffer in its strong
    plane than a circle of the same mass. That is why I-beams exist, and it is why a long bone's
    cross-section is an ellipse whose principal axis lines up with the bending it actually sees.

    A round rod spends material providing stiffness in a direction nothing loads.

    Called with a single `Iy` and no `Iz`, this is the old circular element, exactly.
    """
    Iz = Iy if Iz is None else Iz
    J = 2.0 * Iy if J is None else J
    A_arr = np.asarray(A)
    if A_arr.ndim > 0:
        n = len(A_arr)
        L_arr = np.asarray(L)
        k = np.zeros((n, 12, 12))
        k[:, 0, 0] = k[:, 6, 6] = E * A_arr / L_arr
        k[:, 0, 6] = k[:, 6, 0] = -E * A_arr / L_arr
        k[:, 3, 3] = k[:, 9, 9] = G * J / L_arr
        k[:, 3, 9] = k[:, 9, 3] = -G * J / L_arr

        a = 12 * E * Iz / (L_arr ** 3)
        b = 6 * E * Iz / (L_arr ** 2)
        c = 4 * E * Iz / L_arr
        d = 2 * E * Iz / L_arr
        for (i, j, v) in ((1, 1, a), (1, 5, b), (1, 7, -a), (1, 11, b),
                          (5, 5, c), (5, 7, -b), (5, 11, d),
                          (7, 7, a), (7, 11, -b), (11, 11, c)):
            k[:, i, j] = k[:, j, i] = v

        a = 12 * E * Iy / (L_arr ** 3)
        b = 6 * E * Iy / (L_arr ** 2)
        c = 4 * E * Iy / L_arr
        d = 2 * E * Iy / L_arr
        for (i, j, v) in ((2, 2, a), (2, 4, -b), (2, 8, -a), (2, 10, -b),
                          (4, 4, c), (4, 8, b), (4, 10, d),
                          (8, 8, a), (8, 10, b), (10, 10, c)):
            k[:, i, j] = k[:, j, i] = v
        return k

    k = np.zeros((12, 12))
    k[0, 0] = k[6, 6] = E * A / L
    k[0, 6] = k[6, 0] = -E * A / L
    k[3, 3] = k[9, 9] = G * J / L
    k[3, 9] = k[9, 3] = -G * J / L

    # bending in the local x-y plane -- v (1, 7) and rz (5, 11) -- resisted by Iz
    a, b, c, d = (12 * E * Iz / L ** 3, 6 * E * Iz / L ** 2,
                  4 * E * Iz / L, 2 * E * Iz / L)
    for (i, j, v) in ((1, 1, a), (1, 5, b), (1, 7, -a), (1, 11, b),
                      (5, 5, c), (5, 7, -b), (5, 11, d),
                      (7, 7, a), (7, 11, -b), (11, 11, c)):
        k[i, j] = k[j, i] = v
    # bending in the local x-z plane -- w (2, 8) and ry (4, 10) -- resisted by Iy; signs flip
    a, b, c, d = (12 * E * Iy / L ** 3, 6 * E * Iy / L ** 2,
                  4 * E * Iy / L, 2 * E * Iy / L)
    for (i, j, v) in ((2, 2, a), (2, 4, -b), (2, 8, -a), (2, 10, -b),
                      (4, 4, c), (4, 8, b), (4, 10, d),
                      (8, 8, a), (8, 10, b), (10, 10, c)):
        k[i, j] = k[j, i] = v
    return k


def local_axes(v, roll=0.0):
    """The element's own axes (ex along it, ey and ez across it), ROLLED about its own axis.

    ⚠ THE ROLL USED TO BE ARBITRARY, AND THIS FILE'S OWN DOCSTRING SAID SO -- "its own axis does not
    matter, no orientation vector to get wrong". That is true of a CIRCLE and of nothing else. The
    moment an element has a major and a minor axis, the roll IS the design: it is which way the
    section is turned to meet the bending.
    """
    L = float(np.linalg.norm(v))
    ex = v / L
    ref = np.array([0.0, 0.0, 1.0])
    if abs(float(ex @ ref)) > 0.99:
        ref = np.array([0.0, 1.0, 0.0])
    ez0 = np.cross(ex, ref)
    ez0 /= np.linalg.norm(ez0)
    ey0 = np.cross(ez0, ex)
    c, s = np.cos(roll), np.sin(roll)
    ey = c * ey0 + s * ez0
    ez = -s * ey0 + c * ez0
    return ex, ey, ez, L


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
                 shells=(), shell_t=0.0006, nu=0.3, roll=0.0):
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
        for be, (i, j) in enumerate(self.bars):
            pi, pj = self.nodes[i], self.nodes[j]
            ex, ey, ez, L = local_axes(pj - pi, float(np.asarray(roll).flat[be])
                                       if np.size(roll) > 1 else float(roll))
            R = np.vstack([ex, ey, ez])                    # local axes as rows
            T = np.zeros((12, 12))
            for b in range(4):
                T[3 * b:3 * b + 3, 3 * b:3 * b + 3] = R
            A_e = float(A[be]) if hasattr(A, "__len__") else float(A)
            I_e = float(I[be]) if hasattr(I, "__len__") else float(I)
            J_e = float(J[be]) if (J is not None and hasattr(J, "__len__")) else (None if J is None else float(J))
            kl = _element_k(L, E, G, A_e, I_e, J=J_e)
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
        for s_idx, (i, j, k) in enumerate(self.shells):
            t_elem = shell_t[s_idx] if hasattr(shell_t, "__len__") else shell_t
            out = _cst_k(self.nodes[i], self.nodes[j], self.nodes[k], E, nu, t_elem)
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
        if _cho_factor is not None:
            try:
                self.lu = _cho_factor(_csc_array(K), lower=True)   # .solve(B), same shape contract
                return self
            except _CholmodError:
                pass                                    # not PD enough for Cholesky: general LU
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


# =============================================================================
# MODULAR 3D SPACE-FRAME FEM SOLVER FOR EXOKEY SPINE & OUTRIGGER ASSEMBLIES
# =============================================================================

class SpaceFrameFEM:
    """3D Space Frame Finite Element Solver."""
    
    def __init__(self):
        self.nodes = []
        self.elements = []
        self.fixed_dofs = set()
        
    def add_node(self, coord: np.ndarray) -> int:
        idx = len(self.nodes)
        self.nodes.append(np.asarray(coord, dtype=np.float64))
        return idx
        
    def add_element(self, n0_idx: int, n1_idx: int,
                    E: float = 140.0e9, G: float = 5.0e9,
                    r_od: float = 0.0022, r_id: float = 0.0013) -> int:
        idx = len(self.elements)
        self.elements.append({
            "n0": n0_idx,
            "n1": n1_idx,
            "E": float(E),
            "G": float(G),
            "r_od": float(r_od),
            "r_id": float(r_id)
        })
        return idx
        
    def fix_node(self, node_idx: int):
        for dof in range(6):
            self.fixed_dofs.add(node_idx * 6 + dof)
            
    def _element_stiffness(self, elem: dict) -> tuple[np.ndarray, np.ndarray, float]:
        p0 = self.nodes[elem["n0"]]
        p1 = self.nodes[elem["n1"]]
        v = p1 - p0
        L = float(np.linalg.norm(v))
        if L < 1e-6:
            raise ValueError(f"Element length {L} m is too short between node {elem['n0']} and {elem['n1']}")
            
        E = elem["E"]
        G = elem["G"]
        r_od = elem["r_od"]
        r_id = elem["r_id"]
        
        A = np.pi * (r_od**2 - r_id**2)
        Iy = np.pi * (r_od**4 - r_id**4) / 4.0
        Iz = Iy
        J = Iy + Iz
        
        k = np.zeros((12, 12))
        
        k[0, 0] = k[6, 6] = E * A / L
        k[0, 6] = k[6, 0] = -E * A / L
        
        k[3, 3] = k[9, 9] = G * J / L
        k[3, 9] = k[9, 3] = -G * J / L
        
        k[1, 1] = k[7, 7] = 12 * E * Iz / (L**3)
        k[1, 7] = k[7, 1] = -12 * E * Iz / (L**3)
        k[1, 5] = k[5, 1] = k[1, 11] = k[11, 1] = 6 * E * Iz / (L**2)
        k[5, 7] = k[7, 5] = k[7, 11] = k[11, 7] = -6 * E * Iz / (L**2)
        k[5, 5] = k[11, 11] = 4 * E * Iz / L
        k[5, 11] = k[11, 5] = 2 * E * Iz / L
        
        k[2, 2] = k[8, 8] = 12 * E * Iy / (L**3)
        k[2, 8] = k[8, 2] = -12 * E * Iy / (L**3)
        k[2, 4] = k[4, 2] = k[2, 10] = k[10, 2] = -6 * E * Iy / (L**2)
        k[4, 8] = k[8, 4] = k[8, 10] = k[10, 8] = 6 * E * Iy / (L**2)
        k[4, 4] = k[10, 10] = 4 * E * Iy / L
        k[4, 10] = k[10, 4] = 2 * E * Iy / L
        
        u_x = v / L
        ref = np.array([0.0, 0.0, 1.0]) if abs(u_x[2]) < 0.90 else np.array([0.0, 1.0, 0.0])
        u_y = np.cross(ref, u_x)
        u_y /= np.linalg.norm(u_y)
        u_z = np.cross(u_x, u_y)
        
        R = np.vstack([u_x, u_y, u_z])
        T = np.zeros((12, 12))
        for b in range(4):
            T[b*3:(b+1)*3, b*3:(b+1)*3] = R
            
        K_elem_global = T.T @ k @ T
        return K_elem_global, T, L
        
    def solve(self, nodal_forces: dict[int, np.ndarray]) -> dict:
        N = len(self.nodes)
        total_dofs = 6 * N
        K_global = np.zeros((total_dofs, total_dofs))
        
        elem_data = []
        for elem in self.elements:
            K_elem, T_elem, L = self._element_stiffness(elem)
            elem_data.append((K_elem, T_elem, L))
            
            n0 = elem["n0"]
            n1 = elem["n1"]
            dofs = np.array([
                n0*6+0, n0*6+1, n0*6+2, n0*6+3, n0*6+4, n0*6+5,
                n1*6+0, n1*6+1, n1*6+2, n1*6+3, n1*6+4, n1*6+5
            ])
            for i in range(12):
                for j in range(12):
                    K_global[dofs[i], dofs[j]] += K_elem[i, j]
                    
        F_global = np.zeros(total_dofs)
        for n_idx, f_vec in nodal_forces.items():
            F_global[n_idx*6 : (n_idx+1)*6] += f_vec
            
        all_dofs = np.arange(total_dofs)
        free_dofs = np.array([d for d in all_dofs if d not in self.fixed_dofs])
        
        if len(free_dofs) == 0:
            return {"displacements": np.zeros((N, 6)), "max_deflection_um": 0.0, "max_stress_MPa": 0.0}
            
        K_ff = K_global[np.ix_(free_dofs, free_dofs)]
        F_f = F_global[free_dofs]
        
        u_f = np.linalg.solve(K_ff, F_f)
        
        u_full = np.zeros(total_dofs)
        u_full[free_dofs] = u_f
        displacements = u_full.reshape((N, 6))
        
        elem_stresses = []
        for e_idx, elem in enumerate(self.elements):
            K_elem, T_elem, L = elem_data[e_idx]
            n0 = elem["n0"]
            n1 = elem["n1"]
            dofs = np.array([
                n0*6+0, n0*6+1, n0*6+2, n0*6+3, n0*6+4, n0*6+5,
                n1*6+0, n1*6+1, n1*6+2, n1*6+3, n1*6+4, n1*6+5
            ])
            u_elem_global = u_full[dofs]
            u_elem_local = T_elem @ u_elem_global
            
            E = elem["E"]
            r_od = elem["r_od"]
            r_id = elem["r_id"]
            A = np.pi * (r_od**2 - r_id**2)
            Iy = np.pi * (r_od**4 - r_id**4) / 4.0
            Iz = Iy
            
            delta_L = u_elem_local[6] - u_elem_local[0]
            N_force = E * A * delta_L / L
            
            theta_y0, theta_y1 = u_elem_local[4], u_elem_local[10]
            theta_z0, theta_z1 = u_elem_local[5], u_elem_local[11]
            w_y0, w_y1 = u_elem_local[1], u_elem_local[7]
            w_z0, w_z1 = u_elem_local[2], u_elem_local[8]
            
            M_z_max = max(abs(E * Iz * (4*theta_z0 + 2*theta_z1 - 6*(w_y1 - w_y0)/L) / L),
                          abs(E * Iz * (2*theta_z0 + 4*theta_z1 - 6*(w_y1 - w_y0)/L) / L))
            M_y_max = max(abs(E * Iy * (4*theta_y0 + 2*theta_y1 + 6*(w_z1 - w_z0)/L) / L),
                          abs(E * Iy * (2*theta_y0 + 4*theta_y1 + 6*(w_z1 - w_z0)/L) / L))
            M_bend_max = np.sqrt(M_y_max**2 + M_z_max**2)
            
            sigma_axial = abs(N_force) / max(A, 1e-12)
            sigma_bending = (M_bend_max * r_od) / max(Iy, 1e-12)
            sigma_total = sigma_axial + sigma_bending
            elem_stresses.append(sigma_total)
            
        trans_displacements = np.linalg.norm(displacements[:, :3], axis=1)
        max_deflection_um = float(np.max(trans_displacements)) * 1.0e6
        max_stress_MPa = float(np.max(elem_stresses)) / 1.0e6 if len(elem_stresses) else 0.0
        
        return {
            "displacements": displacements,
            "trans_displacements_m": trans_displacements,
            "elem_stresses_Pa": elem_stresses,
            "max_deflection_um": max_deflection_um,
            "max_stress_MPa": max_stress_MPa
        }


def run_exokey_fem_analysis(p_root: np.ndarray,
                            mcp_nodes: dict[str, np.ndarray],
                            digit_chains: dict[str, list[np.ndarray]],
                            typing_force_N: float = 0.196) -> dict:
    """Run full 3D Space Frame Finite Element Analysis with Thumb Strut attached to Index Knuckle."""
    fem = SpaceFrameFEM()
    node_map = {}
    
    # 1. Root Node (clamped at Metacarpal Saddle Hub)
    n_root = fem.add_node(p_root)
    node_map["root"] = n_root
    fem.fix_node(n_root)
    
    # 2. MCP Knuckle Nodes
    for f, p_mcp in mcp_nodes.items():
        node_map[f"mcp_{f}"] = fem.add_node(p_mcp)
        
    # 3. Primary Central Spine: Saddle Root -> Middle Knuckle (MCP3) - CF ⌀ 8.0mm OD / ⌀ 6.0mm ID (E = 180 GPa)
    fem.add_element(node_map["root"], node_map["mcp_middle"],
                    E=180e9, G=6e9, r_od=0.0040, r_id=0.0030)
                    
    # 4. Transverse Knuckle Arch across Fingers: Little <-> Ring <-> Middle <-> Index (CF ⌀ 6.0mm OD / ⌀ 4.4mm ID)
    arch_seq = ["little", "ring", "middle", "index"]
    for i in range(len(arch_seq) - 1):
        fA, fB = arch_seq[i], arch_seq[i+1]
        fem.add_element(node_map[f"mcp_{fA}"], node_map[f"mcp_{fB}"],
                        E=180e9, G=6e9, r_od=0.0030, r_id=0.0022)
                        
    # 5. DIRECT THUMB STRUT ATTACHED TO INDEX KNUCKLE JOINT! (Index MCP -> Web Arch -> Thumb MCP)
    p_web = 0.5 * (mcp_nodes["index"] + mcp_nodes["thumb"]) + 0.008 * np.array([0.0, 0.0, 1.0]) + 0.006 * np.array([0.0, 1.0, 0.0])
    n_web = fem.add_node(p_web)
    fem.add_element(node_map["mcp_index"], n_web,
                    E=180e9, G=6e9, r_od=0.0030, r_id=0.0022)
    fem.add_element(n_web, node_map["mcp_thumb"],
                    E=180e9, G=6e9, r_od=0.0030, r_id=0.0022)
                        
    # 6. Conformal Phalanx Chains - CF ⌀ 5.0mm OD / ⌀ 3.4mm ID
    pod_node_indices = {}
    for f, chain in digit_chains.items():
        prev_node = node_map[f"mcp_{f}"]
        for seg_idx, pt in enumerate(chain[1:]):
            n_curr = fem.add_node(pt)
            node_map[f"{f}_node_{seg_idx+1}"] = n_curr
            fem.add_element(prev_node, n_curr,
                            E=180e9, G=6e9, r_od=0.0025, r_id=0.0017)
            prev_node = n_curr
        pod_node_indices[f] = prev_node
        
    # 7. Evaluate Load Cases
    # Case A: Individual typing on each digit (0.196 N / 20 gf plunge)
    single_results = {}
    for f, pod_idx in pod_node_indices.items():
        forces = {pod_idx: np.array([0.0, 0.0, -typing_force_N, 0.0, 0.0, 0.0])}
        res = fem.solve(forces)
        tip_disp_um = float(res["trans_displacements_m"][pod_idx]) * 1.0e6
        single_results[f] = {
            "tip_deflection_um": tip_disp_um,
            "max_stress_MPa": res["max_stress_MPa"],
            "safety_factor": 1200.0 / max(res["max_stress_MPa"], 1e-3)
        }
        
    # Case B: Simultaneous 5-finger chord typing (1.0 N total load)
    chord_forces = {}
    for f, pod_idx in pod_node_indices.items():
        chord_forces[pod_idx] = np.array([0.0, 0.0, -typing_force_N, 0.0, 0.0, 0.0])
    res_chord = fem.solve(chord_forces)
    
    # Case C: 2.0 N accidental snag impact on little finger
    snag_forces = {pod_node_indices["little"]: np.array([0.0, 0.0, -2.0, 0.0, 0.0, 0.0])}
    res_snag = fem.solve(snag_forces)
    
    worst_tip_deflection_um = max(s["tip_deflection_um"] for s in single_results.values())
    worst_digit = max(single_results.keys(), key=lambda k: single_results[k]["tip_deflection_um"])
    
    return {
        "single_finger_results": single_results,
        "worst_single_deflection_um": worst_tip_deflection_um,
        "worst_single_digit": worst_digit,
        "chord_typing": {
            "max_deflection_um": res_chord["max_deflection_um"],
            "max_stress_MPa": res_chord["max_stress_MPa"],
            "safety_factor": 1200.0 / max(res_chord["max_stress_MPa"], 1e-3)
        },
        "snag_impact": {
            "max_deflection_um": res_snag["max_deflection_um"],
            "max_stress_MPa": res_snag["max_stress_MPa"],
            "safety_factor": 1200.0 / max(res_snag["max_stress_MPa"], 1e-3)
        },
        "passes_deflection_gate": worst_tip_deflection_um <= 150.0,
        "passes_stress_gate": res_chord["max_stress_MPa"] <= 120.0
    }
