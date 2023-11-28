from setuptools import find_packages, setup

package_name = 'teleop_haptics'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml',
                                   'launch/haptics.launch.xml',
                                   'launch/force.launch.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rohan',
    maintainer_email='rohankota@u.northwestern.edu',
    description='This package handles all of the haptic feedback of the teleop system.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'haptx_feedback = teleop_haptics.haptx_feedback:main',
            'force_feedback = teleop_haptics.kinesthetic_feedback:main',
            'hand_position = teleop_haptics.hand_position:main',
        ],
    },
)
