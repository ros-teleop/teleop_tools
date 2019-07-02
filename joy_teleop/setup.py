from setuptools import find_packages
from setuptools import setup

package_name = 'joy_teleop'

setup(
    name=package_name,
    version='0.3.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config/', ['config/joy_teleop_example.yaml']),
        ('share/' + package_name + '/launch/', ['launch/example.launch.py']),
        ('share/' + package_name + '/test/', ['test/key_teleop.test.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Paul Mathieu',
    author_email='paul.mathieu@pal-robotics.com',
    maintainer='Bence Magyar',
    maintainer_email='bence.magyar.robotics@gmail.com',
    url='https://github.com/ros-teleop/teleop_tools',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: BSD',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='A (to be) generic joystick interface to control a robot.',
    long_description="""\
        joy_teleop interfaces a joystick to control/actions sent to a robot. \
        Its flexibility allows to map any joystick button to any message/service/action.""",
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_teleop = scripts.joy_teleop:main',
            'incrementer_server = scripts.incrementer_server:main',
            'dummy = scripts.dummy:main',
        ],
    },
)
