import pathlib

from setuptools import find_packages, setup

here = pathlib.Path(__file__).parent.resolve()

package_name = 'teleop_sensing'


def copy_dir_rec(dest: pathlib.Path, source: pathlib.Path,
                 here: pathlib.Path) -> list[tuple[pathlib.Path, list[pathlib.Path]]]:
    # Set dest.parent as root, then go through all sub files
    # and output their p.relative_to(dest.parents)
    if not source.is_dir():
        return [(str(dest / source.relative_to(here).parent), [str(source)])]
    out_list = []
    current_level_list = (str(dest / source.relative_to(here)), [])
    for item in source.iterdir():
        if item.is_dir():
            out_list.extend(copy_dir_rec(dest, item, here))
        else:
            current_level_list[1].append(str(item.relative_to(here)))

    if current_level_list[1]:
        out_list.append(current_level_list)
    return out_list


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] + copy_dir_rec(pathlib.Path('share/' + package_name), here / pathlib.Path('launch'), here) +
    copy_dir_rec(pathlib.Path('share/' + package_name), here / pathlib.Path('config'), here),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rohan',
    maintainer_email='rohankota@u.northwestern.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['cv_process = teleop_sensing.cv_process:main'],
    },
)
