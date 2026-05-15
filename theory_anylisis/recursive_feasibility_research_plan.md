# SRBD-MPC Recursive Feasibility Research Plan

This note gives a staged research path for studying recursive feasibility and
stability of the current Python SRBD-MPC/full-SRB setup. The final goal is to
move from a fixed-contact, fixed-footstep MPC problem to a multi-mode switched
system with variable contact timing and footstep decisions.

## 0. Current Problem Statement

The current closed loop is:

```text
HLIP/LQR footstep planner
  -> fixed contact preview over MPC horizon
  -> linearized SRBD-MPC QP
  -> full nonlinear SRB plant integration
```

The plant state is

```text
X = (R, p, omega, v)
```

and the MPC state is

```text
x = [rpy, p, omega, v, g].
```

The full SRB plant obeys

```text
m v_dot = sum_i c_i f_i - m g e_z

I_W(R) omega_dot + omega x I_W(R) omega
  = sum_i c_i ((p_i - p) x f_i + tau_i)
```

where

```text
I_W(R) = R I_B R^T.
```

The MPC uses a linearized SRBD model:

```text
rpy_dot = Rz(yaw_0) omega
p_dot   = v
v_dot   = (1 / m) sum_i c_i f_i - g e_z

omega_dot =
I_W(R_0)^(-1) sum_i c_i ((p_i - p_ref) x f_i + tau_i).
```

So the recursive-feasibility question is:

```text
If the QP is feasible at time k, and the first input is applied to the full SRB
plant, under what assumptions is the shifted QP feasible again at time k+1?
```

## 1. The Two Main Linearization Errors

The current MPC has two dominant approximation channels.

### Error A: Moment-Arm Bilinearity

The true centroidal moment contains

```text
(p_i - p) x f_i.
```

The MPC replaces `p` with `p_ref`:

```text
(p_i - p_ref) x f_i.
```

The residual is

```text
d_M,p = ((p_i - p) - (p_i - p_ref)) x f_i
      = (p_ref - p) x f_i.
```

For all contacts:

```text
d_M,p = sum_i c_i (p_ref - p) x f_i.
```

A useful bound is

```text
||d_M,p|| <= ||p - p_ref|| sum_i c_i ||f_i||.
```

If the contact wrench set gives

```text
||f_i|| <= f_i,max,
```

and the tube bound is

```text
||p - p_ref|| <= rho_p,
```

then

```text
||d_M,p|| <= rho_p sum_i c_i f_i,max.
```

This term is well suited for tube/robust MPC because it is a bounded additive
disturbance if COM tracking error and contact forces are bounded.

### Error B: Nonlinear Rotational Dynamics

The full SRB rotational equation is

```text
omega_dot =
I_W(R)^(-1) (M - omega x I_W(R) omega).
```

The MPC uses

```text
omega_dot_mpc = I_W(R_0)^(-1) M_mpc.
```

There are two residuals inside this channel:

```text
d_omega,I =
(I_W(R)^(-1) - I_W(R_0)^(-1)) M
```

and

```text
d_omega,gyro =
- I_W(R)^(-1) (omega x I_W(R) omega).
```

For small attitude error `delta theta`, a local Lipschitz bound can be written

```text
||I_W(R)^(-1) - I_W(R_0)^(-1)||
  <= L_I ||delta theta||
```

so

```text
||d_omega,I|| <= L_I rho_R ||M||.
```

The gyroscopic residual is quadratic:

```text
||d_omega,gyro||
  <= ||I_W^(-1)|| ||omega|| ||I_W omega||
  <= k_gyro ||omega||^2.
```

This can also be treated as a bounded disturbance in a local operating region:

```text
||delta theta|| <= rho_R
||omega|| <= rho_omega
||M|| <= M_max.
```

Then

```text
||d_omega,I + d_omega,gyro||
<= L_I rho_R M_max + k_gyro rho_omega^2.
```

This is tube-MPC friendly only if the planned motion keeps roll/pitch/yaw and
angular velocity inside a small region. For aggressive yaw or large torso
rotation, this approximation should be replaced by a nonlinear or
successive-linearization model.

## 2. Stage A: Fixed Footsteps, Fixed Contact Sequence, Fixed Timing

This is the cleanest recursive-feasibility problem.

Assume the following are known and fixed:

```text
p_L(j), p_R(j)
c_L(j), c_R(j)
t_switch(j)
```

The MPC is a time-varying constrained linear system:

```text
x_{k+1} = A_k x_k + B_k u_k + d_k
u_k in U_k
x_k in X_k
```

where `d_k` represents the full-SRB mismatch.

Research tasks:

1. Define compact state constraints `X_k`.

   Use physically meaningful bounds:

   ```text
   |roll| <= roll_max
   |pitch| <= pitch_max
   |yaw - yaw_ref| <= yaw_max
   |p - p_ref| <= p_max
   |v - v_ref| <= v_max
   |omega| <= omega_max
   ```

2. Define compact input constraints `U_k`.

   These are already the foot wrench cone and inactive-contact zero constraints.

3. Compute a disturbance set `D_k`.

   Start with box bounds from simulation logs:

   ```text
   d_k in D_k = {d: |d| <= d_max,k}.
   ```

   Then derive analytic conservative bounds from the two residual terms above.

4. Add tightened constraints:

   ```text
   z_k in Z_k
   x_k = z_k + e_k
   e_k in E_k
   Z_k = X_k minus E_k
   V_k = U_k minus K E_k
   ```

5. Use tube MPC:

   Nominal system:

   ```text
   z_{k+1} = A_k z_k + B_k v_k
   ```

   Feedback:

   ```text
   u_k = v_k + K_k (x_k - z_k).
   ```

   Error dynamics:

   ```text
   e_{k+1} = (A_k + B_k K_k) e_k + d_k.
   ```

6. Prove recursive feasibility by shifted sequence:

   If at time `k` there is a feasible nominal sequence

   ```text
   {v_{0|k}, ..., v_{N-1|k}},
   ```

   then at `k+1` use

   ```text
   {v_{1|k}, ..., v_{N-1|k}, v_terminal}
   ```

   and show tightened constraints hold under

   ```text
   e_k in E_k,
   d_k in D_k.
   ```

Minimum deliverable:

```text
Fixed footstep + fixed timing robust MPC remains feasible for N steps under
bounded full-SRB mismatch D_k.
```

## 3. Stage B: Terminal Set For Periodic Walking

For a periodic gait, contact mode alternates:

```text
L stance -> R stance -> L stance -> ...
```

With fixed timing and fixed nominal step length, build the Poincare map from
one touchdown to the next:

```text
x_{n+1} = P x_n + Q u_n + d_n.
```

Research tasks:

1. Define a step-to-step nominal orbit:

   ```text
   x_star,L, x_star,R
   u_star,L(t), u_star,R(t).
   ```

2. Linearize around the orbit:

   ```text
   delta x_{n+1} = A_P delta x_n + B_P delta u_n + d_n.
   ```

3. Compute a robust positively invariant set for the step-to-step error:

   ```text
   E_P = (A_P + B_P K_P) E_P plus D_P.
   ```

4. Use this set as the terminal condition:

   ```text
   x_N - x_star,mode in E_P.
   ```

This gives a stronger recursive-feasibility proof because the terminal set is
matched to walking instead of standing.

## 4. Stage C: Fixed Timing, Footsteps As Bounded Decision Variables

Next relax fixed touchdown position while keeping contact sequence and timing
fixed.

Let the next footstep be

```text
p_swing,next = p_stance + Rz(yaw) [l_x, l_y, 0]^T.
```

Add step variables:

```text
l_x in [l_x,min, l_x,max]
l_y in [l_y,min, l_y,max]
```

Current G1 lateral bounds:

```text
left stance:  l_y in [-0.60, -0.10]
right stance: l_y in [ 0.10,  0.60].
```

Challenge:

The moment arm now depends on the decision variable:

```text
(p_foot(l) - p_ref) x f
```

which is bilinear in step variable and force.

Options:

1. Two-layer approach.

   Keep footstep generated by HLIP/LQR. The MPC sees footsteps as parameters.
   Recursive feasibility is conditional on the footstep planner returning a
   point inside a robust feasible footstep set.

2. Alternating/SQP approach.

   Fix footstep, solve wrench MPC, update footstep, repeat.

3. Mixed-integer or nonlinear MPC.

   Use exact bilinear dynamics or mode-dependent contact variables. This is
   more faithful but much harder for formal recursive feasibility.

Recommended path:

Use option 1 first. Define a robust feasible footstep set:

```text
P_step(x) = {p_next: MPC is feasible under tightened constraints}.
```

Then prove the planner is filtered by projection:

```text
p_next = Proj_{P_step(x)}(p_HLIP).
```

## 5. Stage D: Timing Relaxation

Now allow touchdown time to vary:

```text
T_ss in [T_min, T_max].
```

This creates a switched/time-varying system:

```text
x_{k+1} = A_sigma(T) x_k + B_sigma(T) u_k + d_k.
```

Research tasks:

1. Treat timing uncertainty as a bounded mode uncertainty.

   If the planned switch time differs by

   ```text
   delta T in [-Delta_T, Delta_T],
   ```

   then derive an additional disturbance set:

   ```text
   d_T = x(T + delta T) - x(T).
   ```

2. Tighten constraints around switching surfaces.

   Near impact, ensure both the pre-switch and post-switch contact assumptions
   remain feasible for a guard interval:

   ```text
   t in [t_switch - Delta_T, t_switch + Delta_T].
   ```

3. Add dwell-time assumptions.

   Recursive feasibility for switched systems usually needs minimum dwell time:

   ```text
   T_ss >= T_min > 0.
   ```

Minimum deliverable:

```text
The MPC remains feasible if actual contact switching occurs within a bounded
time window and both adjacent modes have nonempty tightened feasible sets.
```

## 6. Stage E: Multi-Mode Switched System

The final formulation has modes

```text
sigma in {DS, LSS, RSS, flight/failure optional}
```

with mode-dependent constraints:

```text
U_DS  = both foot cones active
U_LSS = left cone active, right wrench zero
U_RSS = right cone active, left wrench zero
```

and mode-dependent dynamics:

```text
x_{k+1} = f_sigma(x_k, u_k, p_contact,k).
```

The hybrid system includes guards and resets:

```text
guard: swing foot reaches touchdown set
reset: p_swing becomes new contact point
```

Research tasks:

1. Define mode-specific robust feasible sets:

   ```text
   C_DS, C_LSS, C_RSS.
   ```

2. Define transition maps:

   ```text
   T_DS->LSS, T_LSS->RSS, T_RSS->LSS.
   ```

3. Prove set inclusion:

   ```text
   T_sigma->sigma_next(C_sigma) subset C_sigma_next.
   ```

4. Use multiple Lyapunov functions:

   ```text
   V_sigma(x)
   ```

   with mode transition decrease:

   ```text
   V_sigma_next(x^+) - V_sigma(x) <= -alpha ||x||^2 + disturbance_bound.
   ```

This is the natural formal endpoint of the current simulator.

## 7. Should We Use Tube MPC?

Tube MPC is appropriate for the current model if the goal is to analyze local
robust recursive feasibility.

It handles well:

```text
d_M,p = sum_i (p_ref - p) x f_i
```

because this can be bounded by COM tracking error and force limits.

It can also handle rotational mismatch locally:

```text
d_omega = d_omega,I + d_omega,gyro
```

if attitude and angular rate are kept inside a bounded tube.

But tube MPC becomes conservative when:

```text
roll/pitch/yaw are large,
omega is large,
contact timing uncertainty is large,
footstep variables are optimized inside the same QP.
```

Recommended tube formulation:

```text
x = z + e
u = v + K_sigma e
e^+ = (A_sigma + B_sigma K_sigma) e + d_sigma
```

with mode-dependent tubes:

```text
E_DS, E_LSS, E_RSS.
```

The tightened QP uses

```text
z in X_sigma minus E_sigma
v in U_sigma minus K_sigma E_sigma.
```

## 8. Alternative Modeling Options

If tube bounds are too conservative, consider these upgrades.

### Successive Linearization MPC

At each MPC solve, linearize around a predicted trajectory:

```text
x_{j+1} = A_j x_j + B_j u_j + c_j.
```

This captures:

```text
I_W(R_j),
(p_i - p_j) x f_i,
omega_j x I_W(R_j) omega_j.
```

Pros:

```text
less conservative than tube around a fixed point
still QP-based after linearization
```

Cons:

```text
recursive feasibility proof is harder
requires trust region or contraction argument
```

### Nonlinear MPC On SO(3)

Use full dynamics directly:

```text
R_{j+1} = exp(hat(dt omega_j)) R_j
omega_dot = I_W(R_j)^(-1)(M_j - omega_j x I_W(R_j) omega_j)
```

Pros:

```text
physically clean
less model mismatch
```

Cons:

```text
nonconvex
harder real-time solve
recursive feasibility proof needs nonlinear robust MPC tools
```

### Centroidal-Momentum State Model

Use angular momentum

```text
h = I_W omega
```

as the rotational state:

```text
h_dot = sum_i ((p_i - p) x f_i + tau_i).
```

This removes the explicit gyroscopic term from the momentum dynamics. The
attitude reconstruction still needs kinematics, but the input-to-momentum map
is cleaner.

Pros:

```text
better matched to centroidal dynamics
natural for ALIP/HLIP momentum states
```

Cons:

```text
attitude constraints still nonlinear
mapping h -> omega depends on R
```

This is likely the best next modeling upgrade before full nonlinear MPC.

## 9. Recommended Milestones

1. Fixed-footstep nominal feasibility.

   Disable plant mismatch and verify the shifted MPC sequence remains feasible.

2. Fixed-footstep robust feasibility.

   Add bounded disturbance from measured full-SRB residuals and implement
   tightened constraints.

3. Analytic tube bounds.

   Replace empirical disturbance bounds with bounds from moment-arm, inertia,
   and gyroscopic residuals.

4. Periodic terminal set.

   Compute a step-to-step terminal invariant set for alternating single support.

5. Footstep feasible-set filter.

   Keep HLIP/LQR footsteps, but project them into a robust feasible footstep
   set before passing them to MPC.

6. Timing uncertainty.

   Add bounded touchdown-time uncertainty and guard-interval feasibility.

7. Multi-mode hybrid proof.

   Build mode-specific feasible sets and transition inclusion checks.

8. Modeling upgrade.

   Compare three controllers:

   ```text
   SRBD + tube
   successive-linearized SRB
   centroidal-momentum SRB
   ```

   Evaluate conservatism, feasibility rate, and RPY stability.

## 10. Immediate Experiments In This Repo

Use the existing stable command as baseline:

```bash
conda run -n aligator python theory_anylisis/run_g1_srbd_full_srb_analysis.py \
  --duration 8 --no-plot --print-wrench-summary
```

Then run disturbance identification:

```text
For each time step:
1. Predict x_{k+1}^{mpc} with the linear SRBD model.
2. Integrate x_{k+1}^{full} with full_srb_sim.py.
3. Log d_k = x_{k+1}^{full} - x_{k+1}^{mpc}.
4. Group d_k by mode: DS, LSS, RSS.
5. Fit box or ellipsoidal disturbance sets D_sigma.
```

This gives the first practical tube-MPC disturbance model.
