import warnings
with warnings.catch_warnings():
    warnings.simplefilter('always')
    warnings.warn('`import kinematics_dynamics` is deprecated, use `import roboticslab_kinematics_dynamics` instead', DeprecationWarning)
from roboticslab_kinematics_dynamics import *
