from setuptools import find_packages, setup
import os
import glob

package_name = 'myrobot_sim_mujoco'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), [f for f in glob.glob('config/*') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.py')),
        (os.path.join('share', package_name, 'models'), [f for f in glob.glob('models/*') if os.path.isfile(f)]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chenzhaoqi',
    maintainer_email='zhaoqi.chen@spacemit.com',
    description='MuJoCo based differential drive simulation for myrobot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mujoco_diff_bridge = myrobot_sim_mujoco.mujoco_diff_bridge:main',
        ],
    },
)
