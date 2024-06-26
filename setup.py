from setuptools import setup

package_name = 'superdev_ws'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=[
        'scripts.teleop.py',
        'scripts.teleop_fix.py',
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='David Capacho',
    maintainer_email='dcapacho3@gmail.com',
    description='Mi paquete de ROS 2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop = scripts.teleop:main',
            'teleop_fix = scripts.teleop_fix:main',
        ],
    },
)

