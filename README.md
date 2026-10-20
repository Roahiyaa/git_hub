# Mathematical Modelling and Derivations

## 1. System Overview

The UAV target-tracking system consists of target motion generation, camera-based target detection, image-coordinate error calculation, feedback control, and UAV motion.

Let the target position in world coordinates be

\[
\mathbf{p}_T(t)=
\begin{bmatrix}
x_T(t)\\
y_T(t)\\
z_T(t)
\end{bmatrix}
\]

and the UAV position be

\[
\mathbf{p}_D(t)=
\begin{bmatrix}
x_D(t)\\
y_D(t)\\
z_D(t)
\end{bmatrix}.
\]

The detected target location in the image is

\[
\mathbf{q}_k=
\begin{bmatrix}
c_{x,k}\\
c_{y,k}
\end{bmatrix}.
\]

The target-tracking process follows the feedback relationship

\[
\boxed{
\text{Image Acquisition}
\rightarrow
\text{Target Detection}
\rightarrow
\text{Error Calculation}
\rightarrow
\text{Control}
\rightarrow
\text{UAV Motion}
\rightarrow
\text{Next Image}
}
\]

## 2. Target Motion Modelling

The target trajectory is represented by

\[
\mathbf{p}_T(t)=f(t).
\]

### 2.1 Circular Trajectory

For a circular trajectory with radius \(R\), centre \((x_c,y_c)\), angular velocity \(\omega\), and initial angle \(\theta_0\),

\[
\theta(t)=\omega t+\theta_0.
\]

The target coordinates are

\[
\boxed{x_T(t)=x_c+R\cos(\omega t+\theta_0)}
\]

\[
\boxed{y_T(t)=y_c+R\sin(\omega t+\theta_0)}
\]

For constant altitude,

\[
z_T(t)=z_0.
\]

The target velocity is obtained by differentiating its position:

\[
\boxed{v_{Tx}(t)=-R\omega\sin(\omega t+\theta_0)}
\]

\[
\boxed{v_{Ty}(t)=R\omega\cos(\omega t+\theta_0)}
\]

\[
v_{Tz}(t)=0.
\]

The horizontal speed is

\[
\boxed{v_T=R|\omega|}.
\]

These equations represent a standard circular trajectory model; the actual trajectory equations depend on the target generator implementation.

## 3. HSV-Based Target Detection

Let an image pixel in HSV colour space be

\[
I_{\mathrm{HSV}}(x,y)=
\begin{bmatrix}
H(x,y)\\
S(x,y)\\
V(x,y)
\end{bmatrix}.
\]

Let the lower and upper HSV thresholds be

\[
\mathbf{L}=(H_L,S_L,V_L)
\]

and

\[
\mathbf{U}=(H_U,S_U,V_U).
\]

The binary colour mask is defined as

\[
M(x,y)=
\begin{cases}
1, & H_L\leq H(x,y)\leq H_U,\\
   & S_L\leq S(x,y)\leq S_U,\\
   & V_L\leq V(x,y)\leq V_U,\\
0, & \text{otherwise}.
\end{cases}
\]

The detected foreground pixel set is

\[
\mathcal{S}=\{(x_i,y_i)\mid M(x_i,y_i)=1\}.
\]

If the target centre is estimated using the foreground centroid,

\[
\boxed{c_x=\frac{1}{N}\sum_{i=1}^{N}x_i}
\]

\[
\boxed{c_y=\frac{1}{N}\sum_{i=1}^{N}y_i},
\]

where \(N\) is the number of detected foreground pixels.

Alternatively, if a bounding rectangle has top-left coordinate \((x,y)\), width \(w\), and height \(h\), its centre is

\[
\boxed{c_x=x+\frac{w}{2}}
\]

\[
\boxed{c_y=y+\frac{h}{2}}.
\]

The implementation determines which centre-estimation method is used.

## 4. Image-Centre Error Calculation

Let the camera image dimensions be \(W\times H\). The image centre is

\[
\boxed{
\mathbf{q}^{*}=
\begin{bmatrix}
W/2\\
H/2
\end{bmatrix}
}.
\]

For the detected target centre

\[
\mathbf{q}_k=
\begin{bmatrix}
c_{x,k}\\
c_{y,k}
\end{bmatrix},
\]

define the image-coordinate error as detected position minus desired position:

\[
\mathbf{e}_k=\mathbf{q}_k-\mathbf{q}^{*}.
\]

The horizontal error is

\[
\boxed{e_x=c_x-\frac{W}{2}}
\]

and the vertical error is

\[
\boxed{e_y=c_y-\frac{H}{2}}.
\]

The alternative convention, desired position minus detected position, is

\[
\boxed{e_x=\frac{W}{2}-c_x}
\]

\[
\boxed{e_y=\frac{H}{2}-c_y}.
\]

The selected sign convention must remain consistent with the controller's command direction.

## 5. Normalized Image Error

To express image error relative to the image dimensions, normalize each error by half of the corresponding image dimension.

The normalized horizontal error is

\[
e_x^n=\frac{e_x}{W/2}.
\]

Substituting the image-error equation,

\[
\boxed{e_x^n=\frac{2c_x-W}{W}}.
\]

The normalized vertical error is

\[
e_y^n=\frac{e_y}{H/2}.
\]

Therefore,

\[
\boxed{e_y^n=\frac{2c_y-H}{H}}.
\]

For detected coordinates within the image boundaries,

\[
\boxed{-1\leq e_x^n\leq1}
\]

\[
\boxed{-1\leq e_y^n\leq1}.
\]

Normalization makes the errors relative to image dimensions rather than dependent only on raw pixel displacement.

## 6. PID Controller Derivation

The PID controller combines proportional, integral, and derivative contributions to calculate the control output.

Define the tracking error as

\[
e(t)=r(t)-y(t),
\]

where \(r(t)\) is the desired value and \(y(t)\) is the measured value.

### 6.1 Proportional Term

The proportional contribution depends on the instantaneous error:

\[
\boxed{u_P(t)=K_Pe(t)}.
\]

### 6.2 Integral Term

The accumulated error is

\[
\boxed{I(t)=\int_0^t e(\tau)\,d\tau}.
\]

The integral contribution is

\[
\boxed{u_I(t)=K_I\int_0^t e(\tau)\,d\tau}.
\]

For a discrete controller with sampling interval \(\Delta t\),

\[
\boxed{I_k=I_{k-1}+e_k\Delta t}.
\]

Thus,

\[
u_{I,k}=K_I I_k.
\]

### 6.3 Derivative Term

The derivative contribution depends on the rate of error change:

\[
\boxed{u_D(t)=K_D\frac{de(t)}{dt}}.
\]

Its discrete approximation is

\[
\boxed{D_k=\frac{e_k-e_{k-1}}{\Delta t}}.
\]

Therefore,

\[
u_{D,k}=K_DD_k.
\]

### 6.4 Complete PID Equation

Combining the three contributions gives

\[
\boxed{
u(t)=K_Pe(t)
+K_I\int_0^t e(\tau)\,d\tau
+K_D\frac{de(t)}{dt}
}.
\]

The discrete-time PID equation is

\[
\boxed{
u_k=K_Pe_k+K_I I_k+
K_D\frac{e_k-e_{k-1}}{\Delta t}
}.
\]

Here, \(K_P\), \(K_I\), and \(K_D\) represent the proportional, integral, and derivative gains, respectively.

## 7. Image Error to Control Command

A proportional image-based control law can be represented as

\[
\boxed{u_x=K_xe_x^n}
\]

\[
\boxed{u_y=K_ye_y^n}.
\]

For independent PID controllers, the horizontal and vertical image errors may be mapped to separate control outputs:

\[
\boxed{
u_x=K_{Px}e_x^n+
K_{Ix}\int e_x^n\,dt+
K_{Dx}\frac{de_x^n}{dt}
}
\]

\[
\boxed{
u_y=K_{Py}e_y^n+
K_{Iy}\int e_y^n\,dt+
K_{Dy}\frac{de_y^n}{dt}
}.
\]

These expressions describe possible control mappings. The actual control axes, command units, and controller gains must be obtained from the implementation.

## 8. Discrete-Time UAV Motion Model

Velocity is the time derivative of position:

\[
\boxed{\mathbf{v}(t)=\frac{d\mathbf{p}(t)}{dt}}.
\]

Integrating velocity over one simulation step gives

\[
\mathbf{p}(t+\Delta t)-\mathbf{p}(t)
=
\int_t^{t+\Delta t}\mathbf{v}(\tau)\,d\tau.
\]

Under a constant-velocity approximation over the interval,

\[
\boxed{\mathbf{p}_{k+1}=\mathbf{p}_k+\mathbf{v}_k\Delta t}.
\]

For individual axes,

\[
\boxed{x_{k+1}=x_k+v_{x,k}\Delta t}
\]

\[
\boxed{y_{k+1}=y_k+v_{y,k}\Delta t}
\]

\[
\boxed{z_{k+1}=z_k+v_{z,k}\Delta t}.
\]

This is a kinematic approximation. A physics-based simulation may calculate motion through its rigid-body dynamics and lower-level flight controller instead.

## 9. Pinhole Camera Model

Let a 3D point in camera coordinates be

\[
\mathbf{P}_C=
\begin{bmatrix}
X_C\\Y_C\\Z_C
\end{bmatrix}.
\]

Under the pinhole camera model, the projected image-plane coordinates are

\[
\boxed{x'=\frac{fX_C}{Z_C}}
\]

\[
\boxed{y'=\frac{fY_C}{Z_C}}.
\]

Using focal lengths in pixel units and the principal point \((c_x,c_y)\), the image coordinates are

\[
\boxed{u=f_x\frac{X_C}{Z_C}+c_x}
\]

\[
\boxed{v=f_y\frac{Y_C}{Z_C}+c_y}.
\]

The intrinsic camera matrix is

\[
\boxed{
K=
\begin{bmatrix}
f_x&0&c_x\\
0&f_y&c_y\\
0&0&1
\end{bmatrix}
}.
\]

The homogeneous projection equation is

\[
\boxed{
Z_C
\begin{bmatrix}
u\\v\\1
\end{bmatrix}
=
K
\begin{bmatrix}
X_C\\Y_C\\Z_C
\end{bmatrix}
}.
\]

If the principal point coincides with the image centre, the image-coordinate errors satisfy

\[
\boxed{e_x=f_x\frac{X_C}{Z_C}}
\]

\[
\boxed{e_y=f_y\frac{Y_C}{Z_C}}.
\]

If depth is known, the camera-frame coordinates can be recovered as

\[
\boxed{X_C=\frac{(u-c_x)Z_C}{f_x}}
\]

\[
\boxed{Y_C=\frac{(v-c_y)Z_C}{f_y}}.
\]

Image coordinates alone do not generally determine a unique 3D point when depth is unknown.

## 10. Depth Estimation from Apparent Object Size

If an object's physical height \(H_{\mathrm{obj}}\) is known, its depth may be estimated from its apparent image height using similar triangles.

Let \(f_{\mathrm{px}}\) be the focal length in pixel units and \(h_{\mathrm{px}}\) be the observed object height in pixels.

The estimated depth is

\[
\boxed{
Z\approx
\frac{f_{\mathrm{px}}H_{\mathrm{obj}}}{h_{\mathrm{px}}}
}.
\]

This estimate assumes a calibrated camera, known object dimensions, and an appropriate object orientation and bounding measurement.

## 11. Camera-to-World Coordinate Transformation

A point in camera coordinates can be transformed into world coordinates using rotation and translation:

\[
\boxed{\mathbf{P}_W=R\mathbf{P}_C+\mathbf{t}}.
\]

Here \(R\) is a \(3\times3\) rotation matrix and \(\mathbf{t}\) is a \(3\times1\) translation vector.

For rotation about the \(z\)-axis,

\[
\boxed{
R_z(\theta)=
\begin{bmatrix}
\cos\theta&-\sin\theta&0\\
\sin\theta&\cos\theta&0\\
0&0&1
\end{bmatrix}
}.
\]

The transformation convention must specify whether \(R,\mathbf{t}\) map camera coordinates to world coordinates or the reverse.

## 12. Quaternion Representation of Orientation

A quaternion is represented as

\[
\boxed{q=(w,x,y,z)}.
\]

Its norm is

\[
\boxed{\|q\|=\sqrt{w^2+x^2+y^2+z^2}}.
\]

The normalized quaternion is

\[
\boxed{q_n=\frac{q}{\|q\|}}.
\]

For a unit quaternion, the corresponding rotation matrix is

\[
\boxed{
R(q)=
\begin{bmatrix}
1-2(y^2+z^2)&2(xy-wz)&2(xz+wy)\\
2(xy+wz)&1-2(x^2+z^2)&2(yz-wx)\\
2(xz-wy)&2(yz+wx)&1-2(x^2+y^2)
\end{bmatrix}
}.
\]

Quaternion multiplication is

\[
\boxed{
q_1\otimes q_2=
\begin{bmatrix}
w_1w_2-x_1x_2-y_1y_2-z_1z_2\\
w_1x_2+x_1w_2+y_1z_2-z_1y_2\\
w_1y_2-x_1z_2+y_1w_2+z_1x_2\\
w_1z_2+x_1y_2-y_1x_2+z_1w_2
\end{bmatrix}
}.
\]

Quaternion multiplication combines rotations, with the multiplication order depending on the chosen convention.

## 13. Relative Position and Euclidean Distance

When the target and UAV positions are expressed in the same coordinate frame, the relative position is

\[
\boxed{\mathbf{p}_{\mathrm{rel}}=\mathbf{p}_T-\mathbf{p}_D}.
\]

Expanding,

\[
\boxed{
\mathbf{p}_{\mathrm{rel}}=
\begin{bmatrix}
x_T-x_D\\
y_T-y_D\\
z_T-z_D
\end{bmatrix}
}.
\]

The Euclidean distance is

\[
\boxed{
d=\sqrt{(x_T-x_D)^2+
(y_T-y_D)^2+
(z_T-z_D)^2}
}.
\]

This calculation requires both positions to be available in the same coordinate system.

## 14. Gaussian Image Smoothing

A two-dimensional Gaussian kernel is defined as

\[
\boxed{
G(x,y)=\frac{1}{2\pi\sigma^2}
\exp\left(-\frac{x^2+y^2}{2\sigma^2}\right)
}.
\]

Image smoothing can be expressed as convolution:

\[
\boxed
