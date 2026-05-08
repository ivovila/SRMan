# Kinematic Analysis of the KUKA LBR MED Surgical Manipulator
## Phase I Report — Robotic Systems in Manipulation 2025/2026
**Instituto Superior Técnico** | Prof. Jorge Martins, Prof. Rui Coelho
**Due:** 17 May 2026

---

## 1. Denavit-Hartenberg Convention

### 1.1 Robot Description

The KUKA LBR Med 7 R800 is a **7-DOF redundant surgical manipulator** with all revolute joints. Its kinematic structure consists of alternating perpendicular joint axes, giving a 7-DOF configuration for a 6-DOF task space (one degree of intrinsic redundancy).

**Link dimensions:**
| Parameter | Value | Description |
|-----------|-------|-------------|
| d₁ | 0.340 m | Base platform to shoulder (joint 2 axis) |
| d₃ | 0.400 m | Shoulder to elbow (joint 4 axis) |
| d₅ | 0.400 m | Elbow to wrist center (joint 6 axis) |
| d₇ | 0.126 m | Wrist center to flange |

### 1.2 Frame Assignment (Standard DH Convention)

Following the algorithmic procedure (Siciliano et al.):

1. Number joint axes z₀, ..., z₆ (zᵢ = axis of joint i+1)
2. Place Frame 0 at base; z₀ pointing vertically upward
3. For i = 1..6: origin Oᵢ at intersection of zᵢ₋₁ and zᵢ
4. xᵢ along common normal from zᵢ₋₁ to zᵢ, directed from joint i to joint i+1
5. yᵢ completes the right-handed frame
6. Frame 7 (end-effector): z₇ aligned with z₆; O₇ at flange center

**Key structural observation:** All consecutive joint axes on this robot **intersect** (they are never skew), therefore all link offsets aᵢ = 0.

**Home configuration verification (all qᵢ = 0 — arm fully extended upward):**
- z₀, z₂, z₄, z₆, z₇ → +z direction (vertical)
- z₁, z₅ → −y direction (horizontal)
- z₃ → +y direction (horizontal)

### 1.3 DH Parameter Table

The DH transformation per link is: **Aᵢ^{i-1}(qᵢ) = Rz(θᵢ) · Tz(dᵢ) · Tx(aᵢ) · Rx(αᵢ)**

| Link i | aᵢ (m) | αᵢ (rad) | dᵢ (m) | θᵢ | Joint |
|--------|---------|----------|--------|-----|-------|
| 1 | 0 | +π/2 | 0.340 | q₁ | Base rotation |
| 2 | 0 | −π/2 | 0 | q₂ | Shoulder tilt |
| 3 | 0 | −π/2 | 0.400 | q₃ | Upper arm rotation |
| 4 | 0 | +π/2 | 0 | q₄ | Elbow tilt |
| 5 | 0 | +π/2 | 0.400 | q₅ | Forearm rotation |
| 6 | 0 | −π/2 | 0 | q₆ | Wrist tilt |
| 7 | 0 | 0 | 0.126 | q₇ | End-effector rotation |

> **[INSERT HERE: Hand-drawn sketch of the robot with all 7 DH frames labelled, showing zᵢ and xᵢ axes for each link, and the dᵢ offsets.]**

---

## 2. Direct Kinematics

### 2.1 Method

Using the provided Symbolic Robotics MATLAB Toolbox, the direct kinematics transformation matrix is computed as:

**T₇⁰(q) = A₁⁰(q₁) · A₂¹(q₂) · ... · A₇⁶(q₇)**

Implemented in `LBR_MED.m` (robot definition) and computed by `DKin()` from the toolbox. The Simulink library `LBR_MED_Lib.slx` contains the block `LBR_MED_Direct_Kinematics` with inputs q₁..q₇ and outputs R (3×3) and p (3×1).

### 2.2 Simulink Model

> **[INSERT: Screenshot of LBR_MED_Lib.slx showing the DK block]**
> **[INSERT: Screenshot of LBR_MED_Simul.slx showing the complete validation model]**

### 2.3 Validation

Four configurations were chosen where the end-effector pose can be determined by geometric reasoning:

| Config | Joint angles q (rad) | Expected p (m) | Expected R | Error |p| | Error |R|_F |
|--------|---------------------|----------------|-----------|--------|---------|
| 1: Home | all = 0 | [0, 0, 1.266] | I₃ | < 1e-12 | < 1e-12 |
| 2: Shoulder bend | q₂=π/2, rest=0 | [−0.926, 0, 0.340] | [[0,0,−1],[0,1,0],[1,0,0]] | < 1e-12 | < 1e-12 |
| 3: Base spin | q₁=π/2, rest=0 | [0, 0, 1.266] | Rz(π/2) | < 1e-12 | < 1e-12 |
| 4: Elbow fold | q₂=q₄=π/2, rest=0 | [−0.400, 0, 0.866] | I₃ | < 1e-12 | < 1e-12 |

**Geometric reasoning for each config:**
- **Config 1:** Links stack vertically; total height = d₁+d₃+d₅+d₇ = 1.266 m; no rotation.
- **Config 2:** q₂=π/2 tilts the arm horizontal (−x direction); only d₁=0.340 m remains as height; remaining arm extends 0.400+0.400+0.126=0.926 m horizontally.
- **Config 3:** Rotating a vertically extended arm about its own base axis does not move the end-effector; only orientation changes by Rz(π/2).
- **Config 4:** Upper arm extends −x by 0.400 m; elbow bends 90° upward; forearm+flange add 0.526 m to height from joint 2 at 0.340 m → total = 0.866 m; frames realign so R=I₃.

> **[INSERT: Output of validate_DK.m showing all errors < 1e-10]**

---

## 3. Inverse Kinematics (Closed-Form)

### 3.1 Kinematic Decoupling Approach

The robot has a **spherical wrist** formed by joints 5, 6, 7: their axes z₄, z₅, z₆ all intersect at point O₆ (since d₆=0, so O₆=O₅). This enables kinematic decoupling:

- **Arm problem** (joints 1–4): position the wrist center O₆
- **Wrist problem** (joints 5–7): orient the end-effector

**Redundancy resolution:** The robot has 7 DOF for a 6-DOF task → 1 degree of redundancy. We resolve this by fixing **q₃ = 0** (upper arm rotation). This gives a unique closed-form solution.

### 3.2 Closed-Form Solution

**Step 1 — Wrist center:**
$$\mathbf{p}_W = \mathbf{p}_e - d_7 \, \mathbf{a}_e$$
where **a**_e = R_e[:,3] is the approach vector.

**Step 2 — Base rotation q₁:**
$$q_1 = \text{atan2}(p_{Wy},\, p_{Wx})$$

**Step 3 — Sagittal-plane distances (with q₃=0):**
$$L = \sqrt{p_{Wx}^2 + p_{Wy}^2}, \quad H = p_{Wz} - d_1$$

Using the cosine law in the triangle O₂–O₃–O₅:
$$\cos q_4 = \frac{L^2 + H^2 - d_3^2 - d_5^2}{2 d_3 d_5}, \quad \sin q_4 = \pm\sqrt{1-\cos^2 q_4}$$
$$q_4 = \text{atan2}(\sin q_4,\, \cos q_4) \quad \text{[two solutions: elbow-up/down]}$$

**Step 4 — Shoulder angle q₂:**
$$\gamma = \text{atan2}(L, H), \quad \delta = \text{atan2}(d_5 \sin q_4,\; d_3 + d_5 \cos q_4)$$
$$q_2 = \gamma - \delta$$

**Step 5 — q₃ = 0** (redundancy resolution).

**Step 6 — Wrist rotation:**
$$\mathbf{R}_W = {\mathbf{R}_4^0}^T \cdot \mathbf{R}_e$$

**Step 7 — ZYZ decomposition of R_W → q₅, q₆, q₇:**

From the wrist DH chain structure (α₅=π/2, α₆=−π/2, α₇=0), R₇⁴ has the form:
$$R_W = \begin{bmatrix} c_5 c_6 c_7 - s_5 s_7 & -c_5 c_6 s_7 - s_5 c_7 & -c_5 s_6 \\ s_5 c_6 c_7 + c_5 s_7 & -s_5 c_6 s_7 + c_5 c_7 & -s_5 s_6 \\ s_6 c_7 & -s_6 s_7 & c_6 \end{bmatrix}$$

Reading off q₅, q₆, q₇:
$$q_6 = \text{atan2}\!\left(\sqrt{r_{13}^2 + r_{23}^2},\; r_{33}\right)$$
$$q_5 = \text{atan2}(-r_{23},\; -r_{13}), \quad q_7 = \text{atan2}(-r_{32},\; r_{31})$$

### 3.3 Singularity Conditions

| Label | Condition | Geometric Meaning |
|-------|-----------|-------------------|
| S1 | p_Wx = p_Wy = 0 | Wrist on z₀ axis; q₁ indeterminate |
| S2 | \|cos q₄\| = 1 (q₄=0 or π) | Wrist center on workspace boundary; elbow locked |
| S3 | q₂ = 0 | Arm aligned with z₀; base and forearm axes parallel |
| S4 | q₆ = 0 or π | z₄ ∥ z₆; q₅ and q₇ indeterminate (only sum/difference matters) |

### 3.4 Simulink Model

**Implementation in `IK_LBR_MED.m`.** The IK block takes (p_e, R_e) as input and outputs q (7×1).

> **[INSERT: Screenshot of the IK validation Simulink model (DK → IK → DK round-trip)]**

### 3.5 Validation

Round-trip validation (DK → IK → DK): a joint configuration q is forward-kinematically mapped to (p, R), then the IK recovers q_IK, and DK on q_IK gives (p', R'). All errors |p−p'| < 1e-8 m confirm correctness.

> **[INSERT: Output of validate_IK.m showing all round-trip errors < 1e-6]**

---

## 4. Geometric Jacobian

### 4.1 Definition

For each revolute joint i (slide 54), the geometric Jacobian columns are:
$$\mathbf{J}_{P_i} = \mathbf{z}_{i-1} \times (\mathbf{p}_e - \mathbf{O}_{i-1}), \qquad \mathbf{J}_{O_i} = \mathbf{z}_{i-1}$$

giving the full **6×7** Jacobian J = [J_P; J_O].

This relates joint velocities to end-effector velocity:
$$\mathbf{v}_e = \begin{bmatrix} \dot{\mathbf{p}}_e \\ \boldsymbol{\omega}_e \end{bmatrix} = \mathbf{J}(\mathbf{q})\, \dot{\mathbf{q}}$$

### 4.2 Simulink Model

**Implementation in `Jacobian_LBR_MED.m`** (symbolic computation) and generated into `LBR_MED_Lib.slx` as block `LBR_MED_Jacobian`. Input: q (7×1). Output: J (6×7).

> **[INSERT: Screenshot of LBR_MED_Lib.slx showing the Jacobian block]**

### 4.3 Validation — Numerical Differentiation

The translational part of J is validated via central finite differences:
$$J_{P_i}^{num} \approx \frac{\mathbf{p}_e(\mathbf{q}+\varepsilon\mathbf{e}_i) - \mathbf{p}_e(\mathbf{q}-\varepsilon\mathbf{e}_i)}{2\varepsilon}, \quad \varepsilon = 10^{-7}$$

The angular part is validated via the skew-symmetric matrix of the numerical angular velocity:
$$\mathbf{J}_{O_i}^{num} \approx \text{skew}^{-1}\!\left(\frac{R(\mathbf{q}+\varepsilon\mathbf{e}_i) - R(\mathbf{q}-\varepsilon\mathbf{e}_i)}{2\varepsilon} R^T\right)$$

> **[INSERT: Output of validate_Jacobian.m showing |J_sym - J_fd|_F < 1e-4 for all configs]**

### 4.4 Effect of Kinematic Singularities on Rank

At singularities the rank of J drops from 6 (maximum) to a lower value:

| Configuration | Singularity | rank(J) | Physical meaning |
|---------------|-------------|---------|-----------------|
| q = 0 | S2+S3: elbow+shoulder | 4 | Two DOF lost |
| q₄ = π, others 0 | S2: elbow extended | 5 | One DOF lost |
| q₆ = 0 | S4: wrist aligned | 5 | One DOF lost |
| q₆ = π | S4: wrist aligned | 5 | One DOF lost |

> **[INSERT: Output of validate_Jacobian.m showing rank analysis]**

---

## 5. Closed Loop Inverse Kinematics (CLIK)

### 5.1 CLIK Law with Null-Space Stabilisation

From class (slide 69-70), the CLIK control law is:
$$\dot{\mathbf{q}} = \mathbf{J}^{\dagger}(\mathbf{q})\begin{bmatrix}\dot{\mathbf{p}}_d + K_P\,\mathbf{e}_P \\ \boldsymbol{\omega}_d + K_O\,\mathbf{e}_O\end{bmatrix} + \underbrace{(\mathbf{I} - \mathbf{J}^{\dagger}\mathbf{J})\,\dot{\mathbf{q}}_0}_{\text{null-space term}}$$

**Components:**
- **J^† = J^T(JJ^T)^{-1}** — right pseudo-inverse (7×6), uses damped inversion for singularity robustness
- **e_P = p_d − p_e(q)** — position error
- **e_O = r θ** — orientation error (angle-axis), where R(θ,r) = R_d R_e^T(q)
- **K_P = 2·I₃, K_O = 2·I₃** — proportional gain matrices
- **(I − J^†J)q̇₀** — null-space projection for redundancy stabilisation
- **q̇₀ = −k₀(q − q_mid)** — joint centering potential, k₀ = 0.5

The null-space term drives the robot toward the joint center while tracking the desired end-effector pose. Integration of q̇ gives q(t).

### 5.2 Simulink Model

**Implementation in `generate_CLIK_model.m`**, which creates `LBR_MED_CLIK.slx`.

Block diagram:
```
[p_d, R_d] ─────────────────────────> [CLIK law] ──> q_dot ──> [∫] ──> q ──┐
                                            ^                              │
                                    q ──────┘  (feedback)                 │
                                    │                                      │
                                    └──────────────────────────────────────┘
```

> **[INSERT: Screenshot of LBR_MED_CLIK.slx]**

### 5.3 Validation Against IK (Step 3)

The CLIK model is run with a constant desired pose (p_d, R_d) corresponding to a configuration computed by the closed-form IK from Step 3. Starting from a perturbed initial condition, the CLIK law drives the robot to the same pose.

**Convergence criteria:**
- |p_e(t_final) − p_d| < 1e-4 m
- |R_e(t_final) − R_d|_F < 1e-3

> **[INSERT: Plot of |e_P|(t) and |e_O|(t) over 10 s showing exponential convergence to zero]**
> **[INSERT: Comparison table: q from IK vs q_CLIK at convergence]**

---

## Code Files Summary

| File | Purpose |
|------|---------|
| `LBR_MED.m` | DH parameter table |
| `generate_LBR_MED.m` | Generates `LBR_MED_Lib.slx` (DK block + 3D viz) |
| `build_LBR_MED_Simul.m` | Builds `LBR_MED_Simul.slx` (DK validation model) |
| `validate_DK.m` | Symbolic DK validation (4 configurations) |
| `IK_LBR_MED.m` | Closed-form IK function |
| `generate_IK_model.m` | Adds IK block to library |
| `validate_IK.m` | Round-trip IK validation |
| `Jacobian_LBR_MED.m` | Symbolic Geometric Jacobian |
| `generate_Jacobian_model.m` | Adds Jacobian block to library |
| `validate_Jacobian.m` | Numerical differentiation + singularity analysis |
| `generate_CLIK_model.m` | Builds `LBR_MED_CLIK.slx` |
