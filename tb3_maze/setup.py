from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'tb3_maze'


def files(pattern):
    return [f for f in glob(pattern) if os.path.isfile(f)]


setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), files('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), files('config/*.yaml')),
        (os.path.join('share', package_name, 'worlds'), files('worlds/*.world')),
        (os.path.join('share', package_name, 'maps'), files('maps/*.yaml') + files('maps/*.pgm')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nivakaran Shanmugabavan',
    maintainer_email='Nivakaran-S@users.noreply.github.com',
    description='TurtleBot3 Burger maze world, bringup and evaluation tooling (IE4060 A02)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'generate_maze = tb3_maze.generate_maze:main',
            'check_env = tb3_maze.check_env:main',
            'metrics_logger = tb3_maze.metrics_logger:main',
            'goal_runner = tb3_maze.goal_runner:main',
            'maze_dqn_gazebo = tb3_maze.maze_dqn_gazebo:main',
            'map_accuracy = tb3_maze.map_accuracy:main',
            'plot_results = tb3_maze.plot_results:main',
            'plot_dqn_rewards = tb3_maze.plot_dqn_rewards:main',
        ],
    },
)
