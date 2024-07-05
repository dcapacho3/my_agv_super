from setuptools import setup
import glob
import os

package_name = 'superdev_ws'

# Encuentra todos los scripts en la carpeta 'scripts'
scripts = glob.glob('scripts/*.py')

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=[os.path.splitext(os.path.basename(script))[0] for script in scripts],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='David Capacho',
    maintainer_email='dcapacho3@gmail.com',
    description='Mi paquete de ROS 2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            f'{os.path.splitext(os.path.basename(script))[0]} = scripts.{os.path.splitext(os.path.basename(script))[0]}:main'
            for script in scripts
        ],
    },
)

