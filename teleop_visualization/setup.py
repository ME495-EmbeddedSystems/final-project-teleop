from setuptools import find_packages, setup

package_name = 'teleop_visualization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'launch/rings.launch.xml', 'config/rings.rviz', 'urdf/ring.urdf.xacro', 'meshes/ring.stl', 'meshes/measured_ring.stl', 'meshes/toroid.stl']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rohan',
    maintainer_email='rohankota@u.northwestern.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rings = teleop_visualization.rings:main',
            'scene = teleop_visualization.scene:main'
        ],
    },
)
