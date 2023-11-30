from setuptools import find_packages, setup
from glob import glob
import os.path

package_name = 'teleop_tasks'

data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name
                                                   ]),
    ('share/' + package_name, ['package.xml']),
    ('share/' + package_name + '/env-hooks', ['env-hooks/teleop_tasks.dsv']),
    ('share/' + package_name + '/worlds', glob('worlds/*')),
    ('share/' + package_name + '/launch', glob('launch/*')),
    ('share/' + package_name + '/urdf', glob('urdf/*')),
    ('share/' + package_name + '/config', glob('config/*')),
]

def get_data_files(package_name):
    for root, directories, files in os.walk("models"):
        for file in files:
            name = os.path.join(root, file)
            splitName = name.split(os.path.sep)
            folder = os.path.sep.join(
                name.split(os.path.sep)[:(len(splitName) - 1)])
            data_files.append(('share/' + package_name + "/" + folder, [name]))

    return data_files


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=get_data_files(package_name),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rohan',
    maintainer_email='rohankota@u.northwestern.edu',
    description='Launches user side gazebo environment',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'objects = teleop_tasks.objects:main',
            'ring_sim = teleop_tasks.ring_sim:main'
        ],
    },
)
