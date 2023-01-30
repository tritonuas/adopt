# if __name__ == '__main__':  

import sys
import numpy as np

from setuptools import setup, Extension
from Cython.Build import cythonize

compile_args = []

if sys.platform.startswith('darwin'):
    compile_args=['-std=c++17', '-stdlib=libc++']
else:
    compile_args=['-std=c++17']

setup(
    name="adopt",
    version="0.0.1",
    description="ADOpt",
    packages=["adopt"],
    install_requires=[
        'csdl',
        'modopt',
        'openmdao',
        'python_csdl_backend'
    ],
)
