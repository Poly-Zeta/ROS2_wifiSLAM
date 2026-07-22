
from setuptools import setup
import os
from glob import glob

package_name = 'my_robot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # ament index / resource
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        # launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),

        # urdf
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*')),

        # ekf yaml
        (os.path.join('share', package_name, 'ekf'),
            glob('ekf/*.yaml')),

        # slam yaml (どっちのディレクトリも使うなら両方入れる)
        (os.path.join('share', package_name, 'slam'),
            glob('slam/*.yaml')),

        (os.path.join('share', package_name, 'rkolio'),
            glob('rkolio/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    description='My robot bringup',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [],
    },
)

