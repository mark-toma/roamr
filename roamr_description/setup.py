import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'roamr_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages'    , ['resource/' + package_name]),
        ('share/' + package_name                        , ['package.xml']),
        (os.path.join('share', package_name, 'launch')  , glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'urdf')    , glob('urdf/*')    ),
        (os.path.join('share', package_name, 'meshes')  , glob('meshes/*')  ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marktoma',
    maintainer_email='mark@mark-toma.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_publisher = roamr_description.state_publisher:main'
        ],
    },
)

