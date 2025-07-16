# Autorotation S-Curve Trajectory Calculations Derivation

---

Starting with a raised cosine jerk profile $(J(t))$, integrate to derive the acceleration, velocity, and position, accounting for the constants obtained from initial conditions $a_{0}, \, v_{0}, \, p_{0}$:

$$
J(t)=\alpha\bigl(1-\cos(\beta t)\bigr),
\quad
\alpha=\frac{j_m}{2},\quad
\beta=\frac\pi{t_j},
$$

to get acceleration $a(t)$, velocity $v(t)$, and position $p(t)$, enforcing

$$
 a(0)=a_0,\quad v(0)=v_0,\quad p(0)=p_0.
$$

---

### 1. Acceleration

Integrate jerk:

$$
a(t)=\int_0^t J(\tau)\,d\tau + C_1
=\int_0^t \alpha\bigl(1-\cos(\beta\tau)\bigr)\,d\tau + C_1.
$$

Compute integral:

$$
\int_0^t \bigl(1-\cos(\beta\tau)\bigr)\,d\tau
= \left[\tau - \frac{1}{\beta}\sin(\beta\tau)\right]_0^t
= t - \frac{1}{\beta}\sin(\beta t).
$$

So

$$
a(t)=\alpha\Bigl(t - \frac{1}{\beta}\sin(\beta t)\Bigr)+C_1.
$$

Apply $a(0)=a_0$ gives $C_1=a_0$. Thus

$$
\boxed{
a(t)
= a_0
+\alpha\Bigl(t - \tfrac{1}{\beta}\sin(\beta t)\Bigr).
}
$$

---

### 2. Velocity

Integrate acceleration:

$$
v(t)=\int_0^t a(\tau)\,d\tau + C_2
=\int_0^t[a_0 + \alpha(\tau - \tfrac{1}{\beta}\sin(\beta\tau))]d\tau + C_2.
$$

Compute terms and apply $v(0)=v_0$ to get

$$
\boxed{
v(t)
= v_0
+ a_0\,t
+ \alpha\Bigl(\tfrac{t^2}{2} + \tfrac{\cos(\beta t)-1}{\beta^2}\Bigr).
}
$$

---

### 3. Position

Integrate velocity and apply $p(0)=p_0$:

$$
\boxed{
p(t)
= p_0
+ v_0\,t
+ a_0\,\tfrac{t^2}{2}
+ \alpha\Bigl(\tfrac{t^3}{6} - \tfrac{t}{\beta^2} + \tfrac{\sin(\beta t)}{\beta^3}\Bigr).
}
$$

---

**Summary**

$$
a(t)=a_0 +\alpha\Bigl(t - \tfrac{1}{\beta}\sin(\beta t)\Bigr),
$$
$$
v(t)=v_0 + a_0 t + \alpha\Bigl(\tfrac{t^2}{2} + \tfrac{\cos(\beta t)-1}{\beta^2}\Bigr),
$$
$$
p(t)=p_0 + v_0 t + a_0 \tfrac{t^2}{2} + \alpha\Bigl(\tfrac{t^3}{6} - \tfrac{t}{\beta^2} + \tfrac{\sin(\beta t)}{\beta^3}\Bigr).
$$

---

Assuming that the trajectory can be flown across two distinct phases:
- **Phase 1**: A positive jerk phase, in which  $0 <= t <= T_{1}$, where $T1 = 2 t_{j1}$, and $t_{j1}$ is the cosine time period of phase 1.
- **Phase 2**: A negative jerk phase, in which  $T_{1} <= t <= T_{1} + T_{2}$, where $T2 = 2 t_{j2}$, and $t_{j2}$ is the cosine time period of phase 2.

Prescribe a maximum jerk ($j_{m}$) that is the same in both phases.  Then calculate the time periods $t_{j1}$ and $t_{j2}$ thusly:

$$
T_{1}=2\,t_{j1},\quad T_{2}=2\,t_{j2}
$$

$$
a(0)=a_0,\quad v(0)=v_0,
$$

and using a symmetric “raised‑cosine” jerk profile of peak magnitude $j_m$:

$$
a(T_{1}+T_{2})=a_2,\quad v(T_{1}+T_{2})=v_2,
$$

while enforcing continuity at $t=T_{1}$.

---

### Phase 1

Jerk:

$$
J_1(t)=+\frac{j_m}{2}\bigl[1-\cos(\beta_1\,t)\bigr],\quad \beta_1=\frac{\pi}{t_{j1}}.
$$

Because $T_{1}=2\,t_{j1}$, the acceleration and velocity equations at $T_{1}$ simplify to become:

$$
a_1=a(T_{1})=a_0+j_m\,t_{j1},
\\
v_1=v(T_{1})=v_0+2\,a_0\,t_{j1}+j_m\,t_{j1}^2.
$$

---

### Phase 2

Reverse jerk:

$$
J_2(t)=-\frac{j_m}{2}\bigl[1-\cos(\beta_2\,(t-T_{1}))\bigr],\quad \beta_2=\frac{\pi}{t_{j2}}.
$$

Similarly, because $T_{2}=2\,t_{j2}$, the acceleration and velocity equations at $T_{2}$ simplify to become::

$$
a_2=a_1-j_m\,t_{j2},
\\
v_2=v_1+2\,a_1\,t_{j2}-j_m\,t_{j2}^2.
$$

---

### Enforcing end‑conditions

Ensure acceleration continuity:

$$
a_2=a_1-j_m\,t_{j2}\implies t_{j2}=\frac{a_1-a_2}{j_m}=t_{j1}+\frac{a_0-a_2}{j_m}.
$$

Ensure velocity continuity:

$$
v_2=v_1+2\,a_1\,t_{j2}-j_m\,t_{j2}^2
$$

Substitution of acceleration continuity into velocity continuity:

$$
a_1=a_0+j_m\,t_{j1},
v_1=v_0+2\,a_0\,t_{j1}+j_m\,t_{j1}^2,
t_{j2}=t_{j1}+\frac{a_0-a_2}{j_m}.
$$

Expand all terms, collect like powers of $t_{j1}$, cancel, define

$$
\Delta v=v_2-v_0,\quad \Delta a=a_0-a_2,
$$

Multiply through by $j_m$, note

$$
2a_0\Delta a - \Delta a^2 = a_0^2 - a_2^2,
$$

yielding the quadratic in $t_{j1}$

$$
2\,j_m^2\,t_{j1}^2 + 4\,a_0\,j_m\,t_{j1} + (a_0^2 - a_2^2 - j_m(v_2 - v_0)) = 0.
$$

Solve for the positive root to get:

$$
t_{j1} = \frac{-a_0 + \sqrt{\tfrac{a_0^2 + a_2^2 + j_m\,(v_2 - v_0)}{2}}}{j_m},
$$
$$
t_{j2} = \frac{-a_2 + \sqrt{\tfrac{a_0^2 + a_2^2 + j_m\,(v_2 - v_0)}{2}}}{j_m}.
$$

These roots yield the time required to meet the initial and final conditions in acceleration and velocity. The position equation can then be solved to obtain a look forward in final position if this trajectory were to be flown. If the final position is close to the desired exit position (within some acceptable tolerance) then the manoeuver can executed.

Due to the way this method accounts for the boundary conditions in position, it is not suitable for calculating the trajectory if the final position is already below the desired exit condition. In this case an alternative approach must be taken.

---















