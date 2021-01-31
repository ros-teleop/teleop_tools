import os

from setuptools import find_packages
from setuptools import setup


package_name = 'mouse_teleop'
share_path = os.path.join('share', package_name)


setup(
    name=package_name,
    version='0.3.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (share_path, ['package.xml']),
        (os.path.join(share_path, 'config'), [os.path.join('config', f'{package_name}.yaml')]),
        (os.path.join(share_path, 'launch'),
         [os.path.join('launch', f'{package_name}.launch.py')]),
        (os.path.join('share', 'ament_index', 'resource_index', 'packages'),
         [os.path.join('resource', package_name)]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Enrique Fernandez',
    author_email='enrique.fernandez.perdomo@gmail.com',
    maintainer='Enrique Fernandez',
    maintainer_email='enrique.fernandez.perdomo@gmail.com',
    url='https://github.com/ros-teleop/teleop_tools',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: BSD',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='A text-based interface to send a robot movement commands.',
    long_description="""\
        key_teleop provides command-line interface to send Twist commands \
        to drive a robot around.""",
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mouse_teleop = mouse_teleop.mouse_teleop:main',
        ],
    },
)
