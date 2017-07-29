```python

>>> from __future__ import division
>>> from sympy import *

>>> c0, c1, c2, c3, c4, c5, t = symbols('c0 c1 c2 c3 c4 c5, t')
>>> p, v, a, j = symbols('p, v, a, j')

>>> p = c0 + c1 * t + c2 * (t**2) + c3 * (t**3) + c4 * (t**4) + c5 * (t**5)
>>> v = diff(p, t)
>>> a = diff(v, t)
>>> j = diff(a, t)
>>> squared_jerk = j**2
>>> total_squared_jerk = integrate(squared_jerk, t)

>>> expanded_squared_jerk = expand(j**2)
>>> expanded_total_squared_jerk = expand(integrate(expand(j**2), t))

>>> print(p)
c0 + c1*t + c2*t**2 + c3*t**3 + c4*t**4 + c5*t**5

>>> print(v)
c1 + 2*c2*t + 3*c3*t**2 + 4*c4*t**3 + 5*c5*t**4

>>> print(a)
2*c2 + 6*c3*t + 12*c4*t**2 + 20*c5*t**3

>>> print(j)
6*c3 + 24*c4*t + 60*c5*t**2

>>> print(squared_jerk)
(6*c3 + 24*c4*t + 60*c5*t**2)**2

>>> print(total_squared_jerk)
36*c3**2*t + 144*c3*c4*t**2 + 720*c4*c5*t**4 + 720*c5**2*t**5 + t**3*(240*c3*c5 + 192*c4**2)

>>> print(expanded_squared_jerk)
36*c3**2 + 288*c3*c4*t + 720*c3*c5*t**2 + 576*c4**2*t**2 + 2880*c4*c5*t**3 + 3600*c5**2*t**4

>>> print(expanded_total_squared_jerk)
36*c3**2*t + 144*c3*c4*t**2 + 240*c3*c5*t**3 + 192*c4**2*t**3 + 720*c4*c5*t**4 + 720*c5**2*t**5

```
