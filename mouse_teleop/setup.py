from setuptools import find_packages
from setuptools import setup

package_name = 'mouse_teleop'

setup(
    name=package_name,
    version='0.3.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config/', ['config/mouse_teleop.yaml']),
        ('share/' + package_name + '/launch/', ['launch/mouse_teleop.launch.py']),
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
            'mouse_teleop = scripts.mouse_teleop:main',
        ],
    },
)
