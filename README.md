## Problem Description

The objective of this work is to estimate hydrodynamic parameters of an underwater vehicle—specifically linear and quadratic damping coefficients and effective mass (added and rigid body) terms in the body-frame $x, y, z$ directions—using experimental data collected during vehicle operation.

The available measurements consist of:

- Linear accelerations from an IMU (high rate, noisy)
- Body-frame linear velocities from a DVL (low rate, noisy)
- Commanded thruster forces (subject to modeling uncertainty and degradation with velocity)

The vehicle was operated in a stabilized mode with low angular rates, allowing rotational dynamics and Coriolis coupling to be neglected. Under these assumptions, the translational dynamics along each axis can be modeled independently using a reduced-order rigid-body + hydrodynamic model.

<!-- Although acceleration and velocity are measured at different rates and noise levels, all signals are synchronized and resampled prior to estimation. The MLE framework remains valid under this preprocessing, provided the resulting residuals are approximately Gaussian. -->

<!-- A major challenge in this problem is that the data are:

- Noisy and multi-rate (IMU at 200 Hz, DVL at 10 Hz)
- Correlated through filtering and interpolation
- Affected by unmodeled effects such as thrust degradation and added mass -->

## Why MLE Is an Appropriate Solution

The MLE framework was chosen because:

- It provides a principled probabilistic interpretation of parameter estimation
- MLE provides statistical tools (e.g., covariance of estimates, residual variance) to assess estimator quality.
- It allows extension to:
  - Weighted residuals using known measurement covariances
  - Regularization or priors (MAP estimation)
  - Joint estimation of multiple coupled parameters


## Dynamic Model

For a single translational degree of freedom, the continuous-time dynamics are modeled as:

$$
F(t) = (m + m_a) \, a(t) + d_l \, v(t) + d_q \, |v(t)| v(t) + b
$$

where:

- $F(t)$: net force from thrusters
- $m$: rigid-body mass
- $m_a$: added mass
- $a(t)$: linear acceleration
- $v(t)$: body-frame velocity
- $d_l$: linear damping coefficient
- $d_q$: quadratic damping coefficient
- $b$: constant force bias

This equation is linear in the unknown parameters and can be written in regression form:

$$
F_i = \phi_i^T \theta + \varepsilon_i
$$

with

$$
\phi_i = \begin{bmatrix}
 a_i & v_i & |v_i|v_i & 1
\end{bmatrix},
\quad
\theta = \begin{bmatrix}
 m + m_a \\ d_l \\ d_q \\ b
\end{bmatrix}
$$

and measurement noise $\varepsilon_i$.


## Maximum Likelihood Estimation (MLE)

### Definition

In maximum likelihood parameter learning, we seek the parameter vector $\theta$ that maximizes the likelihood of observing the measured data. Let $D = o_{1:m}$ denote the dataset, where each observation $o_i$ corresponds to a observation of applied force. The maximum likelihood estimate is defined as:

$$
\hat{\theta} = \arg\max_{\theta} P(D \,|\, \theta)
$$

where $P(D | \theta)$ is the likelihood that the probabilistic model assigns to the observed data given parameters $\theta$. 

---

### Likelihood Model

We assume:

1. The samples are independent and identically distributed (i.i.d.)
2. The measurement noise is zero-mean Gaussian with variance $\sigma^2$

$$
\varepsilon_i \sim \mathcal{N}(0, \sigma^2)
$$

Under these assumptions, the likelihood of a single observation is:

$$
P(o_i | \theta) = \mathcal{N}(F_i \,;\, \phi_i^T \theta, \sigma^2)
$$

and the likelihood of the full dataset is:

$$
P(D | \theta) = \prod_{i=1}^m P(o_i | \theta)
$$

---

### Log-Likelihood

Maximizing the likelihood is equivalent to maximizing the log-likelihood:

$$
\hat{\theta} = \arg\max_{\theta} \sum_{i=1}^m \log P(o_i | \theta)
$$

Substituting the Gaussian density and removing constants yields:

$$
\hat{\theta} = \arg\min_{\theta} \sum_{i=1}^m (F_i - \phi_i^T \theta)^2
$$

Thus, under i.i.d. Gaussian noise assumptions, maximum likelihood estimation reduces to ordinary least squares.

---

### Least Square Estimation

$$
y_i = F_{thrust,x}^b(t_i)
$$

and define the regression matrix

$$
\mathbf{X}_i =
\begin{bmatrix}
a_{imu,x}^b(t_i) & v_x^b(t_i) & |v_x^b(t_i)| v_x^b(t_i) &  1
\end{bmatrix}
$$

with parameter vector


$$
\theta =
\begin{bmatrix}
m_x  \\
D_{l,x} \\
D_{q,x} \\
b
\end{bmatrix}
$$

Stacking all samples gives the linear model

$$
\mathbf{y} = \mathbf{X}\theta + \epsilon
$$

Assuming independent and identically distributed Gaussian samples, the likelihood is

$$
P(\mathbf{y} \mid \theta, \sigma^2)
= \prod_i \mathcal{N}(y_i; \mathbf{X}_i\theta, \sigma^2)
$$

Maximizing the log-likelihood yields the maximum likelihood estimate

$$
\hat{\theta} = (\mathbf{X}^T \mathbf{X})^{-1}\mathbf{X}^T\mathbf{y}
$$

and the noise variance estimate

$$
\hat{\sigma}^2 = \frac{1}{N - p} \|\mathbf{y} - \mathbf{X}\hat{\theta}\|^2
$$

The covariance of the parameter estimate is

$$
\mathrm{Cov}(\hat{\theta}) = \hat{\sigma}^2 (\mathbf{X}^T \mathbf{X})^{-1}
$$

