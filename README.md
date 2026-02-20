# ECEN 595R - AUV System ID

## Motivation

Autonomous Underwater Vehicle (AUV) localization in GPS-denied, feature-poor environments is hard, especially in the presence of acoustic spoofing (i.e. military applications). In these scenarios, AUVs often have to rely on noisy and drifting internal sensor data (dead reckoning) to predict their position. As such, as part of our current research we've been exploring effective odometry methods for GPS-denied, acoustic-challenged environments. Typically, common approaches to this problem would include implementing an EKF, UKF, or some specialized variant of the two. Instead, we've opted for a fixed-lag smoothing approach using factor graphs, which allows us to optimize over a window of past states (i.e. 10 seconds) at each timestep. This improves our estimate significantly compared to filters like the EKF, which only consider the current state and have no way to use new information to correct past linearization errors. An diagram of our factor graph structure is included below:

<img width="500" alt="fgo_dvl_binary" src="https://github.com/user-attachments/assets/29837c5b-056f-4aac-865d-4602751bc007" />

When running in real time, the graph adds a new column of variables and measurements to the left at each time step. The light blue circles ($x, v, b$) represent the variables we're optimizing for at a specific point in time -- robot position/orientation ($x$), linear velocity ($v$), and IMU accel/gyro bias ($b$) -- and the colored dots represent measurements from sensors with some associated Gaussian probability. The dark blue dots on the right anchor the system with a prior estimate for each variable.

We've seen good results with this approach, outperforming both alternative factor graph formulations and traditional filtering methods in simulation and on real world data. However, all the methods we've explored rely extensively on availability of a particular underwater sensor called a Doppler Velocity Log (DVL) to provide linear velocity measurements relative to the seafloor (represented by the light purple dot in the graph above). In our approach, DVL velocity measurements effectively constrain the body-frame velocity of the AUV, pinning it to a measured value (with some Gaussian approximation) at each point in time. When DVL goes offline, the optimizer really struggles distiguishing between changing linear velocities and changing IMU acceleration biases, which can lead to extensive estimation drift upon misclassification. To illustrate this, attached is a gif of the position estimate of a CougUV vehicle (given by the green arrow/lines and 3D model) relative to simulation ground truth (white arrow/lines). When DVL drops out (for 5 seconds every 30 seconds), the estimator REALLY struggles to solve for the location of the AUV.

![Screencastfrom02-19-2026091742PM-ezgif com-optimize](https://github.com/user-attachments/assets/c16e8849-cdce-4414-aafa-15a81d994dc1)

One of the approaches we've explored to mitigate the problem is to use a simple dynamic model to constrain the change in velocity between timesteps. With this added velocity constraint, the optimizer should be able to more accurately distinguish between changing velocities and IMU acceleration bias, preventing the DVL dropouts from corrupting the state estimate. A diagram of our approach augmented with the vehicle dynamic constraint (in orange) is attached below:

<img width="500" height="824" alt="fgo_dynamics" src="https://github.com/user-attachments/assets/7d045ca6-f092-42e1-b7d1-641ac0fc2808" />

Before this project, we had done some work with super-simple dynamic models (i.e. assuming constant velocity) to illustrate the proof of concept. With this project though, we wanted to take the opportunity to explore some more sophisticated models, which requires performing system identification to estimate vehicle parameters.

## Problem Description

The objective of this project is to estimate hydrodynamic parameters of an underwater vehicle -- specifically linear and quadratic damping coefficients and effective mass (added and rigid body) terms in the body-frame $x, y, z$ directions -- using experimental data collected during vehicle operation or simulation.

The available measurements consist of:

- Linear accelerations from an IMU (high rate, noisy)
- Body-frame linear velocities from a DVL (low rate, noisy)
- Commanded thruster forces (subject to modeling uncertainty and velocity degradation)

We made several important assumptions in order to construct a simplified AUV dynamics model capable of being estimated by this data. First, we assumed the vehicle was operated in a stabilized mode with low angular rates, allowing rotational dynamics and Coriolis coupling to be neglected. Second, we ignored hardware effects such as thruster degradation, spool up time, etc. It is a VERY simplified model, but under these assumptions, the translational dynamics along each axis can be modeled and solved for independently.

### Dynamic Model

For a single translational axis, the continuous-time dynamics are modeled as:

$$
F(t) = (m + m_a) \ a(t) + d_l \ v(t) + d_q \ |v(t)| \ v(t)
$$

where:

- $F(t)$: net force from thrusters
- $m$: rigid-body mass
- $m_a$: added mass
- $a(t)$: linear acceleration
- $v(t)$: body-frame velocity
- $d_l$: linear damping coefficient
- $d_q$: quadratic damping coefficient

This equation is linear with respect to the unknown parameters and can be written as:

$$
F_i = \phi_i^T \theta + \varepsilon_i
$$

with

$$
\phi_i = \begin{bmatrix}
 a_i & v_i & |v_i|v_i
\end{bmatrix},
\quad
\theta = \begin{bmatrix}
 m + m_a \\ d_l \\ d_q
\end{bmatrix}
$$

and measurement noise $\varepsilon_i$.

### Why Maximum A Posteriori?

To solve for the unknown parameter vector $\theta$, we formulate a Maximum A Posteriori (MAP) estimation problem. This builds directly upon standard Maximum Likelihood Estimation (MLE) that we talked about in class, with the addition of Bayesian priors.

#### Covariance Weighting

With standard MLE, we are attempting to find the parameter vector $\hat{\theta}$ that maximizes the likelihood of observing our measured data. Assuming the measurement noise is independent and zero-mean Gaussian, $\varepsilon_i \sim \mathcal{N}(0, \sigma_i^2)$, the likelihood of a single force observation is:

$$P(F_i | \theta) = \mathcal{N}(F_i ; \phi_i^T \theta, \sigma_i^2)$$

Maximizing the overall probability of the dataset is mathematically equivalent to maximizing the log-likelihood:

$$\hat{\theta}_{MLE} = \arg\max_{\theta} \sum_{i=1}^{N} \log P(F_i | \theta)$$

Because the accuracy of our IMU and DVL data can change drastically over time, we can't treat all measurements equally. A sudden noise spike or DVL dropout (which we have seen many a time in testing) could potentially have a major effect on skewing the dynamic parameters. To approach this problem, we used Weighted Linear Least Squares (WLLS) from ECEN 671. In this solution, we scale the contribution of each data point by the inverse of its reported covariance ($\sigma_i^2$), effectively weighting confident sensor measurements heavily while down-weighting highly uncertain data:

$$J_{MLE}(\theta) = \sum_{i=1}^{N} \frac{1}{\sigma_i^2} (F_i - \phi_i^T \theta)^2$$

#### Bayesian Priors

From experience, when the target dataset doesn't contain enough "excitement" or information useful for estimating the parameters, we can get really bad estimates. In these states, the least-squares problem lacks the information needed to constrain the parameters, which often leads to physically impossible results like negative mass or inverted drag (that somehow still tracked the data well!). To prevent this, MAP estimation incorporates a prior probability over the parameters, $P(\theta)$. We introduced a Bayesian prior ($\theta_{prior}$) modeled as a Gaussian with an associated confidence ($\Sigma_{prior}$). This allows us to give a "ballpark" physical estimate to regularize the cost function:

$$J_{MAP}(\theta) = J_{MLE}(\theta) + (\theta - \theta_{prior})^T \Sigma_{prior}^{-1} (\theta - \theta_{prior})$$

#### Matrix Formulation and Solution

To solve this in code, we stack our weighted data samples into a regression matrix $\mathbf{X}_w$ and measurement vector $\mathbf{y}_w$, and append our prior constraints to form a single overdetermined linear system:

$$\mathbf{y}_{final} = \mathbf{X}_{final}\theta + \epsilon$$

By solving this system, the parameter estimation is able to take into account strictly data-driven information when sensor confidence is high, but also converges somewhat near our expected baseline when data is sparse. The final MAP parameter estimate is then evaluated using the standard least-squares solution:

$$\hat{\theta} = (\mathbf{X}_{final}^T \mathbf{X}_{final})^{-1}\mathbf{X}_{final}^T\mathbf{y}_{final}$$

## Simulation Results

We used the HoloOcean simulator to benchmark our parameter estimation against two different AUVs with known(ish) parameters.

### BlueROV2

The BlueROV2 employs a fairly simple dynamics model in the simulator. It has some rotation torque and drag effects that our model doesn't capture.

#### Actual Parameters

$$
\theta = \begin{bmatrix}
 11.5 \\ 11.5 \\ 1.80
\end{bmatrix}
$$

#### Estimated Parameters (Noiseless)

$$
\hat{\theta} = \begin{bmatrix}
 11.588 \\ 15.071 \\ - 0.444
\end{bmatrix}
$$


#### Estimated Parameters (Noisy)

$$
\hat{\theta} = \begin{bmatrix}
 12.732 \\ 12.392 \\ 2.846
\end{bmatrix}
$$


### CougUV

The CougUV, in constrast to the BlueROV2, uses a more sophisticated dynamics model from Thor Fossen. It explicitly models processes such as spool up time, thruster velocity degredation, and hydrostatic forces.

#### Actual Parameters

$$
\theta = \begin{bmatrix}
 31.87 \\ 1.593 \\ 6.109
\end{bmatrix}
$$

#### Estimated Parameters (Noiseless)

$$
\hat{\theta} = \begin{bmatrix}
 30.624 \\ 1.494 \\ 5.999
\end{bmatrix}
$$

#### Estimated Parameters (Noisy)

$$
\hat{\theta} = \begin{bmatrix}
 14.625 \\ 7.312 \\ -4.600
\end{bmatrix}
$$

## Real World Results

As part of the project, we took a BlueROV2 to the RB pool on campus to collect some real-world data.

<img width="500" src="https://github.com/user-attachments/assets/f5b5c484-6848-4de7-9218-4e263215d28e" />

#### Estimated Parameters

$$
\hat{\theta}
=
\begin{bmatrix}
28.746 \\
140.948 \\
-30.397
\end{bmatrix}

$$

These parameters don't match what we expected nearly as well as our results from simulation, but we expected that for the most part -- real world testing introduces time synchronization issues, tether dynamics, thruster power variations, accidental hits against the bottom, and more that isn't modeled in simulation. In particular, a setting preset was overriden and we were only able to poll the PWM for the sensors at 2 Hz instead of the expected 50 Hz, which reduced the resolution of our data significantly. There are definitely some more research possibilities here.

## Conclusion

To tie it back into the project motivation, we revisited the DVL dropout simulation from the first section. This time we added the dynamic constraint on the change in velocity using our simplified dynamic model, and populated the values based on our parameter estimation results for the CougUV in HoloOcean. Attached is the gif of the improved state estimate during the same DVL dropouts:

![ezgif com-optimize](https://github.com/user-attachments/assets/40290898-3cb9-4685-a802-b25ed5d429b4)

It works pretty well!
