from setuptools import find_packages, setup
from pathlib import Path

package_name = 'uav_sim'
this_directory = Path(__file__).parent

model_files = [
    (str(p.parent).replace(str(this_directory), f'share/{package_name}'), [str(p)])
    for p in (this_directory / 'models').rglob('*') 
    if p.is_file()
]

world_files = [
    (str(p.parent).replace(str(this_directory), f'share/{package_name}'), [str(p)])
    for p in (this_directory / 'worlds').rglob('*') 
    if p.is_file()
]

launch_files = [
    (f'share/{package_name}/launch', [str(p) for p in (this_directory / 'launch').glob('*.launch.py')])
]

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] + model_files + world_files + launch_files,

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='piotrek',
    maintainer_email='hpiotrek080@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'uav_controller = uav_sim.uav_controller:main'
        ],
    },
)