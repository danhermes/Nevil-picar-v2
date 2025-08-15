# cython: language_level=3
import numpy as np
cimport numpy as cnp

# Initialize numpy C?API
cnp.import_array()

def test():
    def cnp.ndarray[cnp.float64_t, ndim=2] arr
    print("NumPy version:", np.__version__)
