
# pcg_gazebo
Procedural generation package

This module implements the client for the procedural generation plugins in
Gazebo. This interface allows using Python to control the simulation state
in runtime with the help of the specific plugins write in the
**pcg_gazebo_ros_plugins**.

Example:

Attributes:
    module_level_variable1 (int):

Todo:
    * For module TODOs


# pcg_gazebo.log


# pcg_gazebo.transformations
Homogeneous Transformation Matrices and Quaternions.

A library for calculating 4x4 matrices for translating, rotating, reflecting,
scaling, shearing, projecting, orthogonalizing, and superimposing arrays of
3D homogeneous coordinates as well as for converting between rotation matrices,
Euler angles, and quaternions. Also includes an Arcball control object and
functions to decompose transformation matrices.

:Authors:
  `Christoph Gohlke <http://www.lfd.uci.edu/~gohlke/>`__,
  Laboratory for Fluorescence Dynamics, University of California, Irvine

:Version: 20090418

Requirements
------------

* `Python 2.6 <http://www.python.org>`__
* `Numpy 1.3 <http://numpy.scipy.org>`__
* `transformations.c 20090418 <http://www.lfd.uci.edu/~gohlke/>`__
  (optional implementation of some functions in C)

Notes
-----

Matrices (M) can be inverted using numpy.linalg.inv(M), concatenated using
numpy.dot(M0, M1), or used to transform homogeneous coordinates (v) using
numpy.dot(M, v) for shape (4, \*) "point of arrays", respectively
numpy.dot(v, M.T) for shape (\*, 4) "array of points".

Calculations are carried out with numpy.float64 precision.

This Python implementation is not optimized for speed.

Vector, point, quaternion, and matrix function arguments are expected to be
"array like", i.e. tuple, list, or numpy arrays.

Return types are numpy arrays unless specified otherwise.

Angles are in radians unless specified otherwise.

Quaternions ix+jy+kz+w are represented as [x, y, z, w].

Use the transpose of transformation matrices for OpenGL glMultMatrixd().

A triple of Euler angles can be applied/interpreted in 24 ways, which can
be specified using a 4 character string or encoded 4-tuple:

  *Axes 4-string*: e.g. 'sxyz' or 'ryxy'

  - first character : rotations are applied to 's'tatic or 'r'otating frame
  - remaining characters : successive rotation axis 'x', 'y', or 'z'

  *Axes 4-tuple*: e.g. (0, 0, 0, 0) or (1, 1, 1, 1)

  - inner axis: code of axis ('x':0, 'y':1, 'z':2) of rightmost matrix.
  - parity : even (0) if inner axis 'x' is followed by 'y', 'y' is followed
    by 'z', or 'z' is followed by 'x'. Otherwise odd (1).
  - repetition : first and last axis are same (1) or different (0).
  - frame : rotations are applied to static (0) or rotating (1) frame.

References
----------

(1)  Matrices and transformations. Ronald Goldman.
     In "Graphics Gems I", pp 472-475. Morgan Kaufmann, 1990.
(2)  More matrices and transformations: shear and pseudo-perspective.
     Ronald Goldman. In "Graphics Gems II", pp 320-323. Morgan Kaufmann, 1991.
(3)  Decomposing a matrix into simple transformations. Spencer Thomas.
     In "Graphics Gems II", pp 320-323. Morgan Kaufmann, 1991.
(4)  Recovering the data from the transformation matrix. Ronald Goldman.
     In "Graphics Gems II", pp 324-331. Morgan Kaufmann, 1991.
(5)  Euler angle conversion. Ken Shoemake.
     In "Graphics Gems IV", pp 222-229. Morgan Kaufmann, 1994.
(6)  Arcball rotation control. Ken Shoemake.
     In "Graphics Gems IV", pp 175-192. Morgan Kaufmann, 1994.
(7)  Representing attitude: Euler angles, unit quaternions, and rotation
     vectors. James Diebel. 2006.
(8)  A discussion of the solution for the best rotation to relate two sets
     of vectors. W Kabsch. Acta Cryst. 1978. A34, 827-828.
(9)  Closed-form solution of absolute orientation using unit quaternions.
     BKP Horn. J Opt Soc Am A. 1987. 4(4), 629-642.
(10) Quaternions. Ken Shoemake.
     http://www.sfu.ca/~jwa3/cmpt461/files/quatut.pdf
(11) From quaternion to matrix and back. JMP van Waveren. 2005.
     http://www.intel.com/cd/ids/developer/asmo-na/eng/293748.htm
(12) Uniform random rotations. Ken Shoemake.
     In "Graphics Gems III", pp 124-132. Morgan Kaufmann, 1992.


Examples
--------

>>> alpha, beta, gamma = 0.123, -1.234, 2.345
>>> origin, xaxis, yaxis, zaxis = (0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)
>>> I = identity_matrix()
>>> Rx = rotation_matrix(alpha, xaxis)
>>> Ry = rotation_matrix(beta, yaxis)
>>> Rz = rotation_matrix(gamma, zaxis)
>>> R = concatenate_matrices(Rx, Ry, Rz)
>>> euler = euler_from_matrix(R, 'rxyz')
>>> numpy.allclose([alpha, beta, gamma], euler)
True
>>> Re = euler_matrix(alpha, beta, gamma, 'rxyz')
>>> is_same_transform(R, Re)
True
>>> al, be, ga = euler_from_matrix(Re, 'rxyz')
>>> is_same_transform(Re, euler_matrix(al, be, ga, 'rxyz'))
True
>>> qx = quaternion_about_axis(alpha, xaxis)
>>> qy = quaternion_about_axis(beta, yaxis)
>>> qz = quaternion_about_axis(gamma, zaxis)
>>> q = quaternion_multiply(qx, qy)
>>> q = quaternion_multiply(q, qz)
>>> Rq = quaternion_matrix(q)
>>> is_same_transform(R, Rq)
True
>>> S = scale_matrix(1.23, origin)
>>> T = translation_matrix((1, 2, 3))
>>> Z = shear_matrix(beta, xaxis, origin, zaxis)
>>> R = random_rotation_matrix(numpy.random.rand(3))
>>> M = concatenate_matrices(T, R, Z, S)
>>> scale, shear, angles, trans, persp = decompose_matrix(M)
>>> numpy.allclose(scale, 1.23)
True
>>> numpy.allclose(trans, (1, 2, 3))
True
>>> numpy.allclose(shear, (0, math.tan(beta), 0))
True
>>> is_same_transform(R, euler_matrix(axes='sxyz', *angles))
True
>>> M1 = compose_matrix(scale, shear, angles, trans, persp)
>>> is_same_transform(M, M1)
True



## identity_matrix
```python
identity_matrix()
```
Return 4x4 identity/unit matrix.

>>> I = identity_matrix()
>>> numpy.allclose(I, numpy.dot(I, I))
True
>>> numpy.sum(I), numpy.trace(I)
(4.0, 4.0)
>>> numpy.allclose(I, numpy.identity(4, dtype=numpy.float64))
True



## translation_matrix
```python
translation_matrix(direction)
```
Return matrix to translate by direction vector.

>>> v = numpy.random.random(3) - 0.5
>>> numpy.allclose(v, translation_matrix(v)[:3, 3])
True



## translation_from_matrix
```python
translation_from_matrix(matrix)
```
Return translation vector from translation matrix.

>>> v0 = numpy.random.random(3) - 0.5
>>> v1 = translation_from_matrix(translation_matrix(v0))
>>> numpy.allclose(v0, v1)
True



## reflection_matrix
```python
reflection_matrix(point, normal)
```
Return matrix to mirror at plane defined by point and normal vector.

>>> v0 = numpy.random.random(4) - 0.5
>>> v0[3] = 1.0
>>> v1 = numpy.random.random(3) - 0.5
>>> R = reflection_matrix(v0, v1)
>>> numpy.allclose(2., numpy.trace(R))
True
>>> numpy.allclose(v0, numpy.dot(R, v0))
True
>>> v2 = v0.copy()
>>> v2[:3] += v1
>>> v3 = v0.copy()
>>> v2[:3] -= v1
>>> numpy.allclose(v2, numpy.dot(R, v3))
True



## reflection_from_matrix
```python
reflection_from_matrix(matrix)
```
Return mirror plane point and normal vector from reflection matrix.

>>> v0 = numpy.random.random(3) - 0.5
>>> v1 = numpy.random.random(3) - 0.5
>>> M0 = reflection_matrix(v0, v1)
>>> point, normal = reflection_from_matrix(M0)
>>> M1 = reflection_matrix(point, normal)
>>> is_same_transform(M0, M1)
True



## rotation_matrix
```python
rotation_matrix(angle, direction, point=None)
```
Return matrix to rotate about axis defined by point and direction.

>>> angle = (random.random() - 0.5) * (2*math.pi)
>>> direc = numpy.random.random(3) - 0.5
>>> point = numpy.random.random(3) - 0.5
>>> R0 = rotation_matrix(angle, direc, point)
>>> R1 = rotation_matrix(angle-2*math.pi, direc, point)
>>> is_same_transform(R0, R1)
True
>>> R0 = rotation_matrix(angle, direc, point)
>>> R1 = rotation_matrix(-angle, -direc, point)
>>> is_same_transform(R0, R1)
True
>>> I = numpy.identity(4, numpy.float64)
>>> numpy.allclose(I, rotation_matrix(math.pi*2, direc))
True
>>> numpy.allclose(2., numpy.trace(rotation_matrix(math.pi/2,
...                                                direc, point)))
True



## rotation_from_matrix
```python
rotation_from_matrix(matrix)
```
Return rotation angle and axis from rotation matrix.

>>> angle = (random.random() - 0.5) * (2*math.pi)
>>> direc = numpy.random.random(3) - 0.5
>>> point = numpy.random.random(3) - 0.5
>>> R0 = rotation_matrix(angle, direc, point)
>>> angle, direc, point = rotation_from_matrix(R0)
>>> R1 = rotation_matrix(angle, direc, point)
>>> is_same_transform(R0, R1)
True



## scale_matrix
```python
scale_matrix(factor, origin=None, direction=None)
```
Return matrix to scale by factor around origin in direction.

Use factor -1 for point symmetry.

>>> v = (numpy.random.rand(4, 5) - 0.5) * 20.0
>>> v[3] = 1.0
>>> S = scale_matrix(-1.234)
>>> numpy.allclose(numpy.dot(S, v)[:3], -1.234*v[:3])
True
>>> factor = random.random() * 10 - 5
>>> origin = numpy.random.random(3) - 0.5
>>> direct = numpy.random.random(3) - 0.5
>>> S = scale_matrix(factor, origin)
>>> S = scale_matrix(factor, origin, direct)



## scale_from_matrix
```python
scale_from_matrix(matrix)
```
Return scaling factor, origin and direction from scaling matrix.

>>> factor = random.random() * 10 - 5
>>> origin = numpy.random.random(3) - 0.5
>>> direct = numpy.random.random(3) - 0.5
>>> S0 = scale_matrix(factor, origin)
>>> factor, origin, direction = scale_from_matrix(S0)
>>> S1 = scale_matrix(factor, origin, direction)
>>> is_same_transform(S0, S1)
True
>>> S0 = scale_matrix(factor, origin, direct)
>>> factor, origin, direction = scale_from_matrix(S0)
>>> S1 = scale_matrix(factor, origin, direction)
>>> is_same_transform(S0, S1)
True



## projection_matrix
```python
projection_matrix(point,
                  normal,
                  direction=None,
                  perspective=None,
                  pseudo=False)
```
Return matrix to project onto plane defined by point and normal.

Using either perspective point, projection direction, or none of both.

If pseudo is True, perspective projections will preserve relative depth
such that Perspective = dot(Orthogonal, PseudoPerspective).

>>> P = projection_matrix((0, 0, 0), (1, 0, 0))
>>> numpy.allclose(P[1:, 1:], numpy.identity(4)[1:, 1:])
True
>>> point = numpy.random.random(3) - 0.5
>>> normal = numpy.random.random(3) - 0.5
>>> direct = numpy.random.random(3) - 0.5
>>> persp = numpy.random.random(3) - 0.5
>>> P0 = projection_matrix(point, normal)
>>> P1 = projection_matrix(point, normal, direction=direct)
>>> P2 = projection_matrix(point, normal, perspective=persp)
>>> P3 = projection_matrix(point, normal, perspective=persp, pseudo=True)
>>> is_same_transform(P2, numpy.dot(P0, P3))
True
>>> P = projection_matrix((3, 0, 0), (1, 1, 0), (1, 0, 0))
>>> v0 = (numpy.random.rand(4, 5) - 0.5) * 20.0
>>> v0[3] = 1.0
>>> v1 = numpy.dot(P, v0)
>>> numpy.allclose(v1[1], v0[1])
True
>>> numpy.allclose(v1[0], 3.0-v1[1])
True



## projection_from_matrix
```python
projection_from_matrix(matrix, pseudo=False)
```
Return projection plane and perspective point from projection matrix.

Return values are same as arguments for projection_matrix function:
point, normal, direction, perspective, and pseudo.

>>> point = numpy.random.random(3) - 0.5
>>> normal = numpy.random.random(3) - 0.5
>>> direct = numpy.random.random(3) - 0.5
>>> persp = numpy.random.random(3) - 0.5
>>> P0 = projection_matrix(point, normal)
>>> result = projection_from_matrix(P0)
>>> P1 = projection_matrix(*result)
>>> is_same_transform(P0, P1)
True
>>> P0 = projection_matrix(point, normal, direct)
>>> result = projection_from_matrix(P0)
>>> P1 = projection_matrix(*result)
>>> is_same_transform(P0, P1)
True
>>> P0 = projection_matrix(point, normal, perspective=persp, pseudo=False)
>>> result = projection_from_matrix(P0, pseudo=False)
>>> P1 = projection_matrix(*result)
>>> is_same_transform(P0, P1)
True
>>> P0 = projection_matrix(point, normal, perspective=persp, pseudo=True)
>>> result = projection_from_matrix(P0, pseudo=True)
>>> P1 = projection_matrix(*result)
>>> is_same_transform(P0, P1)
True



## clip_matrix
```python
clip_matrix(left, right, bottom, top, near, far, perspective=False)
```
Return matrix to obtain normalized device coordinates from frustrum.

The frustrum bounds are axis-aligned along x (left, right),
y (bottom, top) and z (near, far).

Normalized device coordinates are in range [-1, 1] if coordinates are
inside the frustrum.

If perspective is True the frustrum is a truncated pyramid with the
perspective point at origin and direction along z axis, otherwise an
orthographic canonical view volume (a box).

Homogeneous coordinates transformed by the perspective clip matrix
need to be dehomogenized (devided by w coordinate).

>>> frustrum = numpy.random.rand(6)
>>> frustrum[1] += frustrum[0]
>>> frustrum[3] += frustrum[2]
>>> frustrum[5] += frustrum[4]
>>> M = clip_matrix(*frustrum, perspective=False)
>>> numpy.dot(M, [frustrum[0], frustrum[2], frustrum[4], 1.0])
array([-1., -1., -1.,  1.])
>>> numpy.dot(M, [frustrum[1], frustrum[3], frustrum[5], 1.0])
array([ 1.,  1.,  1.,  1.])
>>> M = clip_matrix(*frustrum, perspective=True)
>>> v = numpy.dot(M, [frustrum[0], frustrum[2], frustrum[4], 1.0])
>>> v / v[3]
array([-1., -1., -1.,  1.])
>>> v = numpy.dot(M, [frustrum[1], frustrum[3], frustrum[4], 1.0])
>>> v / v[3]
array([ 1.,  1., -1.,  1.])



## shear_matrix
```python
shear_matrix(angle, direction, point, normal)
```
Return matrix to shear by angle along direction vector on shear plane.

The shear plane is defined by a point and normal vector. The direction
vector must be orthogonal to the plane's normal vector.

A point P is transformed by the shear matrix into P" such that
the vector P-P" is parallel to the direction vector and its extent is
given by the angle of P-P'-P", where P' is the orthogonal projection
of P onto the shear plane.

>>> angle = (random.random() - 0.5) * 4*math.pi
>>> direct = numpy.random.random(3) - 0.5
>>> point = numpy.random.random(3) - 0.5
>>> normal = numpy.cross(direct, numpy.random.random(3))
>>> S = shear_matrix(angle, direct, point, normal)
>>> numpy.allclose(1.0, numpy.linalg.det(S))
True



## shear_from_matrix
```python
shear_from_matrix(matrix)
```
Return shear angle, direction and plane from shear matrix.

>>> angle = (random.random() - 0.5) * 4*math.pi
>>> direct = numpy.random.random(3) - 0.5
>>> point = numpy.random.random(3) - 0.5
>>> normal = numpy.cross(direct, numpy.random.random(3))
>>> S0 = shear_matrix(angle, direct, point, normal)
>>> angle, direct, point, normal = shear_from_matrix(S0)
>>> S1 = shear_matrix(angle, direct, point, normal)
>>> is_same_transform(S0, S1)
True



## decompose_matrix
```python
decompose_matrix(matrix)
```
Return sequence of transformations from transformation matrix.

matrix : array_like
    Non-degenerative homogeneous transformation matrix

Return tuple of:
    scale : vector of 3 scaling factors
    shear : list of shear factors for x-y, x-z, y-z axes
    angles : list of Euler angles about static x, y, z axes
    translate : translation vector along x, y, z axes
    perspective : perspective partition of matrix

Raise ValueError if matrix is of wrong type or degenerative.

>>> T0 = translation_matrix((1, 2, 3))
>>> scale, shear, angles, trans, persp = decompose_matrix(T0)
>>> T1 = translation_matrix(trans)
>>> numpy.allclose(T0, T1)
True
>>> S = scale_matrix(0.123)
>>> scale, shear, angles, trans, persp = decompose_matrix(S)
>>> scale[0]
0.123
>>> R0 = euler_matrix(1, 2, 3)
>>> scale, shear, angles, trans, persp = decompose_matrix(R0)
>>> R1 = euler_matrix(*angles)
>>> numpy.allclose(R0, R1)
True



## compose_matrix
```python
compose_matrix(scale=None,
               shear=None,
               angles=None,
               translate=None,
               perspective=None)
```
Return transformation matrix from sequence of transformations.

This is the inverse of the decompose_matrix function.

Sequence of transformations:
    scale : vector of 3 scaling factors
    shear : list of shear factors for x-y, x-z, y-z axes
    angles : list of Euler angles about static x, y, z axes
    translate : translation vector along x, y, z axes
    perspective : perspective partition of matrix

>>> scale = numpy.random.random(3) - 0.5
>>> shear = numpy.random.random(3) - 0.5
>>> angles = (numpy.random.random(3) - 0.5) * (2*math.pi)
>>> trans = numpy.random.random(3) - 0.5
>>> persp = numpy.random.random(4) - 0.5
>>> M0 = compose_matrix(scale, shear, angles, trans, persp)
>>> result = decompose_matrix(M0)
>>> M1 = compose_matrix(*result)
>>> is_same_transform(M0, M1)
True



## orthogonalization_matrix
```python
orthogonalization_matrix(lengths, angles)
```
Return orthogonalization matrix for crystallographic cell coordinates.

Angles are expected in degrees.

The de-orthogonalization matrix is the inverse.

>>> O = orthogonalization_matrix((10., 10., 10.), (90., 90., 90.))
>>> numpy.allclose(O[:3, :3], numpy.identity(3, float) * 10)
True
>>> O = orthogonalization_matrix([9.8, 12.0, 15.5], [87.2, 80.7, 69.7])
>>> numpy.allclose(numpy.sum(O), 43.063229)
True



## superimposition_matrix
```python
superimposition_matrix(v0, v1, scaling=False, usesvd=True)
```
Return matrix to transform given vector set into second vector set.

v0 and v1 are shape (3, \*) or (4, \*) arrays of at least 3 vectors.

If usesvd is True, the weighted sum of squared deviations (RMSD) is
minimized according to the algorithm by W. Kabsch [8]. Otherwise the
quaternion based algorithm by B. Horn [9] is used (slower when using
this Python implementation).

The returned matrix performs rotation, translation and uniform scaling
(if specified).

>>> v0 = numpy.random.rand(3, 10)
>>> M = superimposition_matrix(v0, v0)
>>> numpy.allclose(M, numpy.identity(4))
True
>>> R = random_rotation_matrix(numpy.random.random(3))
>>> v0 = ((1,0,0), (0,1,0), (0,0,1), (1,1,1))
>>> v1 = numpy.dot(R, v0)
>>> M = superimposition_matrix(v0, v1)
>>> numpy.allclose(v1, numpy.dot(M, v0))
True
>>> v0 = (numpy.random.rand(4, 100) - 0.5) * 20.0
>>> v0[3] = 1.0
>>> v1 = numpy.dot(R, v0)
>>> M = superimposition_matrix(v0, v1)
>>> numpy.allclose(v1, numpy.dot(M, v0))
True
>>> S = scale_matrix(random.random())
>>> T = translation_matrix(numpy.random.random(3)-0.5)
>>> M = concatenate_matrices(T, R, S)
>>> v1 = numpy.dot(M, v0)
>>> v0[:3] += numpy.random.normal(0.0, 1e-9, 300).reshape(3, -1)
>>> M = superimposition_matrix(v0, v1, scaling=True)
>>> numpy.allclose(v1, numpy.dot(M, v0))
True
>>> M = superimposition_matrix(v0, v1, scaling=True, usesvd=False)
>>> numpy.allclose(v1, numpy.dot(M, v0))
True
>>> v = numpy.empty((4, 100, 3), dtype=numpy.float64)
>>> v[:, :, 0] = v0
>>> M = superimposition_matrix(v0, v1, scaling=True, usesvd=False)
>>> numpy.allclose(v1, numpy.dot(M, v[:, :, 0]))
True



## euler_matrix
```python
euler_matrix(ai, aj, ak, axes='sxyz')
```
Return homogeneous rotation matrix from Euler angles and axis sequence.

ai, aj, ak : Euler's roll, pitch and yaw angles
axes : One of 24 axis sequences as string or encoded tuple

>>> R = euler_matrix(1, 2, 3, 'syxz')
>>> numpy.allclose(numpy.sum(R[0]), -1.34786452)
True
>>> R = euler_matrix(1, 2, 3, (0, 1, 0, 1))
>>> numpy.allclose(numpy.sum(R[0]), -0.383436184)
True
>>> ai, aj, ak = (4.0*math.pi) * (numpy.random.random(3) - 0.5)
>>> for axes in _AXES2TUPLE.keys():
...    R = euler_matrix(ai, aj, ak, axes)
>>> for axes in _TUPLE2AXES.keys():
...    R = euler_matrix(ai, aj, ak, axes)



## euler_from_matrix
```python
euler_from_matrix(matrix, axes='sxyz')
```
Return Euler angles from rotation matrix for specified axis sequence.

axes : One of 24 axis sequences as string or encoded tuple

Note that many Euler angle triplets can describe one matrix.

>>> R0 = euler_matrix(1, 2, 3, 'syxz')
>>> al, be, ga = euler_from_matrix(R0, 'syxz')
>>> R1 = euler_matrix(al, be, ga, 'syxz')
>>> numpy.allclose(R0, R1)
True
>>> angles = (4.0*math.pi) * (numpy.random.random(3) - 0.5)
>>> for axes in _AXES2TUPLE.keys():
...    R0 = euler_matrix(axes=axes, *angles)
...    R1 = euler_matrix(axes=axes, *euler_from_matrix(R0, axes))
...    if not numpy.allclose(R0, R1): print axes, "failed"



## euler_from_quaternion
```python
euler_from_quaternion(quaternion, axes='sxyz')
```
Return Euler angles from quaternion for specified axis sequence.

>>> angles = euler_from_quaternion([0.06146124, 0, 0, 0.99810947])
>>> numpy.allclose(angles, [0.123, 0, 0])
True



## quaternion_from_euler
```python
quaternion_from_euler(ai, aj, ak, axes='sxyz')
```
Return quaternion from Euler angles and axis sequence.

ai, aj, ak : Euler's roll, pitch and yaw angles
axes : One of 24 axis sequences as string or encoded tuple

>>> q = quaternion_from_euler(1, 2, 3, 'ryxz')
>>> numpy.allclose(q, [0.310622, -0.718287, 0.444435, 0.435953])
True



## quaternion_about_axis
```python
quaternion_about_axis(angle, axis)
```
Return quaternion for rotation about axis.

>>> q = quaternion_about_axis(0.123, (1, 0, 0))
>>> numpy.allclose(q, [0.06146124, 0, 0, 0.99810947])
True



## quaternion_matrix
```python
quaternion_matrix(quaternion)
```
Return homogeneous rotation matrix from quaternion.

>>> R = quaternion_matrix([0.06146124, 0, 0, 0.99810947])
>>> numpy.allclose(R, rotation_matrix(0.123, (1, 0, 0)))
True



## quaternion_from_matrix
```python
quaternion_from_matrix(matrix)
```
Return quaternion from rotation matrix.

>>> R = rotation_matrix(0.123, (1, 2, 3))
>>> q = quaternion_from_matrix(R)
>>> numpy.allclose(q, [0.0164262, 0.0328524, 0.0492786, 0.9981095])
True



## quaternion_multiply
```python
quaternion_multiply(quaternion1, quaternion0)
```
Return multiplication of two quaternions.

>>> q = quaternion_multiply([1, -2, 3, 4], [-5, 6, 7, 8])
>>> numpy.allclose(q, [-44, -14, 48, 28])
True



## quaternion_conjugate
```python
quaternion_conjugate(quaternion)
```
Return conjugate of quaternion.

>>> q0 = random_quaternion()
>>> q1 = quaternion_conjugate(q0)
>>> q1[3] == q0[3] and all(q1[:3] == -q0[:3])
True



## quaternion_inverse
```python
quaternion_inverse(quaternion)
```
Return inverse of quaternion.

>>> q0 = random_quaternion()
>>> q1 = quaternion_inverse(q0)
>>> numpy.allclose(quaternion_multiply(q0, q1), [0, 0, 0, 1])
True



## quaternion_slerp
```python
quaternion_slerp(quat0, quat1, fraction, spin=0, shortestpath=True)
```
Return spherical linear interpolation between two quaternions.

>>> q0 = random_quaternion()
>>> q1 = random_quaternion()
>>> q = quaternion_slerp(q0, q1, 0.0)
>>> numpy.allclose(q, q0)
True
>>> q = quaternion_slerp(q0, q1, 1.0, 1)
>>> numpy.allclose(q, q1)
True
>>> q = quaternion_slerp(q0, q1, 0.5)
>>> angle = math.acos(numpy.dot(q0, q))
>>> numpy.allclose(2.0, math.acos(numpy.dot(q0, q1)) / angle) or         numpy.allclose(2.0, math.acos(-numpy.dot(q0, q1)) / angle)
True



## random_quaternion
```python
random_quaternion(rand=None)
```
Return uniform random unit quaternion.

rand: array like or None
    Three independent random variables that are uniformly distributed
    between 0 and 1.

>>> q = random_quaternion()
>>> numpy.allclose(1.0, vector_norm(q))
True
>>> q = random_quaternion(numpy.random.random(3))
>>> q.shape
(4,)



## random_rotation_matrix
```python
random_rotation_matrix(rand=None)
```
Return uniform random rotation matrix.

rnd: array like
    Three independent random variables that are uniformly distributed
    between 0 and 1 for each returned quaternion.

>>> R = random_rotation_matrix()
>>> numpy.allclose(numpy.dot(R.T, R), numpy.identity(4))
True



## Arcball
```python
Arcball()
```
Virtual Trackball Control.

>>> ball = Arcball()
>>> ball = Arcball(initial=numpy.identity(4))
>>> ball.place([320, 320], 320)
>>> ball.down([500, 250])
>>> ball.drag([475, 275])
>>> R = ball.matrix()
>>> numpy.allclose(numpy.sum(R), 3.90583455)
True
>>> ball = Arcball(initial=[0, 0, 0, 1])
>>> ball.place([320, 320], 320)
>>> ball.setaxes([1,1,0], [-1, 1, 0])
>>> ball.setconstrain(True)
>>> ball.down([400, 200])
>>> ball.drag([200, 400])
>>> R = ball.matrix()
>>> numpy.allclose(numpy.sum(R), 0.2055924)
True
>>> ball.next()



### place
```python
Arcball.place(center, radius)
```
Place Arcball, e.g. when window size changes.

center : sequence[2]
    Window coordinates of trackball center.
radius : float
    Radius of trackball in window coordinates.



### setaxes
```python
Arcball.setaxes(*axes)
```
Set axes to constrain rotations.

### setconstrain
```python
Arcball.setconstrain(constrain)
```
Set state of constrain to axis mode.

### getconstrain
```python
Arcball.getconstrain()
```
Return state of constrain to axis mode.

### down
```python
Arcball.down(point)
```
Set initial cursor window coordinates and pick constrain-axis.

### drag
```python
Arcball.drag(point)
```
Update current cursor window coordinates.

### next
```python
Arcball.next(acceleration=0.0)
```
Continue rotation in direction of last drag.

### matrix
```python
Arcball.matrix()
```
Return homogeneous rotation matrix.

## arcball_map_to_sphere
```python
arcball_map_to_sphere(point, center, radius)
```
Return unit sphere coordinates from window coordinates.

## arcball_constrain_to_axis
```python
arcball_constrain_to_axis(point, axis)
```
Return sphere point perpendicular to axis.

## arcball_nearest_axis
```python
arcball_nearest_axis(point, axes)
```
Return axis, which arc is nearest to point.

## vector_norm
```python
vector_norm(data, axis=None, out=None)
```
Return length, i.e. eucledian norm, of ndarray along axis.

>>> v = numpy.random.random(3)
>>> n = vector_norm(v)
>>> numpy.allclose(n, numpy.linalg.norm(v))
True
>>> v = numpy.random.rand(6, 5, 3)
>>> n = vector_norm(v, axis=-1)
>>> numpy.allclose(n, numpy.sqrt(numpy.sum(v*v, axis=2)))
True
>>> n = vector_norm(v, axis=1)
>>> numpy.allclose(n, numpy.sqrt(numpy.sum(v*v, axis=1)))
True
>>> v = numpy.random.rand(5, 4, 3)
>>> n = numpy.empty((5, 3), dtype=numpy.float64)
>>> vector_norm(v, axis=1, out=n)
>>> numpy.allclose(n, numpy.sqrt(numpy.sum(v*v, axis=1)))
True
>>> vector_norm([])
0.0
>>> vector_norm([1.0])
1.0



## unit_vector
```python
unit_vector(data, axis=None, out=None)
```
Return ndarray normalized by length, i.e. eucledian norm, along axis.

>>> v0 = numpy.random.random(3)
>>> v1 = unit_vector(v0)
>>> numpy.allclose(v1, v0 / numpy.linalg.norm(v0))
True
>>> v0 = numpy.random.rand(5, 4, 3)
>>> v1 = unit_vector(v0, axis=-1)
>>> v2 = v0 / numpy.expand_dims(numpy.sqrt(numpy.sum(v0*v0, axis=2)), 2)
>>> numpy.allclose(v1, v2)
True
>>> v1 = unit_vector(v0, axis=1)
>>> v2 = v0 / numpy.expand_dims(numpy.sqrt(numpy.sum(v0*v0, axis=1)), 1)
>>> numpy.allclose(v1, v2)
True
>>> v1 = numpy.empty((5, 4, 3), dtype=numpy.float64)
>>> unit_vector(v0, axis=1, out=v1)
>>> numpy.allclose(v1, v2)
True
>>> list(unit_vector([]))
[]
>>> list(unit_vector([1.0]))
[1.0]



## random_vector
```python
random_vector(size)
```
Return array of random doubles in the half-open interval [0.0, 1.0).

>>> v = random_vector(10000)
>>> numpy.all(v >= 0.0) and numpy.all(v < 1.0)
True
>>> v0 = random_vector(10)
>>> v1 = random_vector(10)
>>> numpy.any(v0 == v1)
False



## inverse_matrix
```python
inverse_matrix(matrix)
```
Return inverse of square transformation matrix.

>>> M0 = random_rotation_matrix()
>>> M1 = inverse_matrix(M0.T)
>>> numpy.allclose(M1, numpy.linalg.inv(M0.T))
True
>>> for size in range(1, 7):
...     M0 = numpy.random.rand(size, size)
...     M1 = inverse_matrix(M0)
...     if not numpy.allclose(M1, numpy.linalg.inv(M0)): print size



## concatenate_matrices
```python
concatenate_matrices(*matrices)
```
Return concatenation of series of transformation matrices.

>>> M = numpy.random.rand(16).reshape((4, 4)) - 0.5
>>> numpy.allclose(M, concatenate_matrices(M))
True
>>> numpy.allclose(numpy.dot(M, M.T), concatenate_matrices(M, M.T))
True



## is_same_transform
```python
is_same_transform(matrix0, matrix1)
```
Return True if two matrices perform same transformation.

>>> is_same_transform(numpy.identity(4), numpy.identity(4))
True
>>> is_same_transform(numpy.identity(4), random_rotation_matrix())
False



# pcg_gazebo.visualization

