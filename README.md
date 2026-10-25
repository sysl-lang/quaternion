# quaternion

Quaternions for [sysl](https://github.com/sysl-lang/sysl): 3D rotation at `real` and at `f32`, from one
body per operation, with no allocation. It runs unchanged on a Cortex-M4F.

```
sh/sysl/quaternion/
    quaternion.sysl  the type, its algebra, and rotation: vectors, matrices, Euler angles, integration
    slerp.sysl       spherical linear interpolation, short path and near-equal fallback
    tests.sysl       what all of it claims, against numpy, run by `sysl test .`
sysl.hocon           who this package is, and what it needs of the machine
```

The module is **`sh.sysl.quaternion`**. It depends on nothing outside the standard library.
`sysl.math.Float` is the whole of what it needs.

## Using it

```
dependencies {
  quaternion { git = "github.com/sysl-lang/quaternion", version = "0.1.0" }
}
```

```
import sh.sysl.quaternion.{Quaternion, slerp}

val turn = Quaternion.from_axis_angle([0.0, 0.0, 1.0], 1.5707963267948966)
val v = turn.rotate([1.0, 0.0, 0.0])                 // [0, 1, 0]

val (yaw, pitch, roll) = Quaternion.from_euler(0.3, -0.4, 1.1).to_euler()

var attitude: Quaternion[f32] = Quaternion.identity()
attitude = attitude.integrate([gx, gy, gz], 0.001)   // one gyroscope sample at 1 kHz

val halfway = slerp(a, b, 0.5)
```

## What is here

- **`Quaternion[F]`** for any `F: sysl.math.Float`, which means `real` and `f32`. It is built from
  components, from an axis and angle (the axis is normalized for you), or as `identity()`. It has
  `+ - *` with `*` the Hamilton product, which **does not commute**. It also has scaling by a number
  on either side, `conjugate`, `dot`, `norm`, `normalize`, `inverse`, `==`, `near`,
  `same_rotation` (`q` and `-q` are one rotation) and `Display` as `w+xi+yj+zk`.
- **Rotation**:
  - `rotate` a `[3]F`.
  - `to_matrix` and `from_matrix` with a row-major `[3][3]F`. `from_matrix` uses Shepperd's method,
    which picks its branch by the largest diagonal term, so a 180° rotation works.
  - `from_euler` and `to_euler` in the **intrinsic Z-Y'-X'' (yaw, pitch, roll)** convention.
  - `integrate` takes a body-frame angular velocity over `dt`.
  - `slerp` takes the short path and falls back to a normalized lerp when the two rotations are
    nearly equal.
- **Gimbal lock is handled explicitly.** At pitch ±90° only yaw−roll (at +90°) or yaw+roll (at −90°)
  is determined. `to_euler` then returns pitch exactly ±π/2 and roll exactly 0, and puts the rest in
  yaw.
- **The zero quaternion** has no inverse and no direction. `normalize` and `inverse` return NaN in
  all four parts, as `Complex.unit` does at zero in `sysl.math.complex`. A zero axis given to
  `from_axis_angle` does the same.

## Where it is used

- **Graphics and game cameras.** An orbit or free-look camera stores its orientation as a quaternion,
  interpolates keyframes with `slerp`, and turns the result into a view matrix with `to_matrix`.
  Euler angles and their gimbal lock are left out of this path.
- **Robotics.** It handles joint and end-effector orientations, converts the rotation matrices a
  kinematics chain produces with `from_matrix`, and reports angles to a person with `to_euler`.
- **IMU sensor fusion on Cortex-M.** Madgwick and Mahony filters keep the attitude as a unit
  quaternion and step it once per gyroscope sample. That step is what `integrate` does. At `f32` the
  arithmetic compiles to FPU instructions (`vmul.f32`, `vsqrt.f32`) on a Cortex-M4F
  (`--target thumbv7em-freestanding`), with no heap and no soft-double helpers. Link a libm for
  `sinf`, `cosf` and `acosf`, for example newlib's `-lm`.

## Licence

ISC. See `LICENSE`.
