from setuptools import find_packages
from setuptools import setup

package_name = 'mouse_teleop'
share_path = 'share/' + package_name

setup(
    name=package_name,
    version='0.3.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (share_path + '/', ['package.xml']),
        (share_path + '/config/', ['config/' + package_name + '.yaml']),
        (share_path + '/launch/', ['launch/mouse_teleop.launch.py']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
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
